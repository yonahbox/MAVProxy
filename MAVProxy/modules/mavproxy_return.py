#!/usr/bin/env python
'''
Return Mission Module - Use an LED and Button on Beaglebone Green Wireless (Grove) to conduct a 
return mission flight. Very specific for now.

Sriram Sami & Yonah, February 2018
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

# Beaglebone-specific
try:
    import Adafruit_BBIO.GPIO as GPIO
except ImportError:
    print "Cannot import BBIO - this is probably not a Beaglebone."

# Threading and time for LED
import threading

# Constants
BUTTON_RELEASED = 0
BUTTON_PRESSED = 1

LED_ON = 0
LED_OFF = 1

LED_PIN = "P9_22"
BUTTON_PIN = "P9_19"


STATE_WAITING = 200
STATE_WAITING_DOWN = 201
STATE_WAITING_LONG_UP = 202
STATE_WAITING_LONG_DOWN = 203
STATE_TRY_LOAD_MISSION = 204
STATE_WAIT_EXECUTE = 205
STATE_EXECUTE = 206

## CHANGE THIS AFTER TESTING
TIME_TO_TAKEOFF_SEC = 20

# Timeouts for states
LONG_PRESS_REQUIRED_TIME_SEC = 4
WAITING_FOR_LONG_PRESS_TIMEOUT = 2

# Blink interval constants (sec)
BLINK_SET_INTERVAL = 1.5
BLINK_BETWEEN_INTERVAL = 0.1
BLINK_LENGTH = 0.1
BLINK_LONG_PRESS_TIMER_INTERVAL = 0.15
BLINK_WARNING_INTERVAL = 0.1
BLINK_DANGER_INTERVAL = 0.05

BLINK_DANGER_TIME_TO_TAKEOFF = 15


class ReturnModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(ReturnModule, self).__init__(mpstate, "return", "return module")
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()
        self.button_state = BUTTON_RELEASED
        self.verbose = False
        self.add_command('return', self.cmd_return, "return module", ['start', 'stop'])

        self.blink_led = False
        self.blink_thread = None
        self.signal_blink_thread_shutdown = False

        # State machine variables and timeouts
        self.system_state = STATE_WAITING
        self.waiting_long_up_start_time = time.time()
        self.long_press_start_time = time.time()

    def usage(self):
        '''show help on command line options'''
        return "Usage: return <start>|<stop>"

    def cmd_return(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "start":
            print(self.start())
        elif args[0] == "stop":
            print(self.stop_running_blink_thread())
        else:
            print(self.usage())

    
    def handle_button_edge(self, button):
        state = GPIO.input(button)
        print "Button state: " + str(state)

        if self.system_state == STATE_WAITING and state == BUTTON_PRESSED:
            self.system_state = STATE_WAITING_DOWN
            print "Waiting for button release"
        elif self.system_state == STATE_WAITING_DOWN and state == BUTTON_RELEASED:
            self.system_state = STATE_WAITING_LONG_UP
            print "Waiting for long press up"
            self.waiting_long_up_start_time = time.time()
        elif self.system_state == STATE_WAITING_LONG_UP and state == BUTTON_PRESSED:
            # We expect that the idle task would have checked this timeout for us already
            self.system_state = STATE_WAITING_LONG_DOWN
            print "Waiting for long press release"
            self.long_press_start_time = time.time()
        elif self.system_state == STATE_WAITING_LONG_DOWN and state == BUTTON_RELEASED:
            # Check if button was held for long enough
            if time.time() - self.long_press_start_time > LONG_PRESS_REQUIRED_TIME_SEC:
                self.system_state = STATE_TRY_LOAD_MISSION
                print "Trying to load mission!"
                # FOR DEBUGGING
                self.system_state = STATE_WAIT_EXECUTE
                self.takeoff_time = time.time() + TIME_TO_TAKEOFF_SEC
            else:
                self.system_state = STATE_WAITING
                print "Button was not held long enough!"
        elif (self.system_state == STATE_TRY_LOAD_MISSION or self.system_state == STATE_WAIT_EXECUTE) and state == BUTTON_PRESSED:
            print "Cancelling return mission load / execute"
            self.system_state = STATE_WAITING






    def start(self):
        '''start GPIO interrupt listening for button and enable LED flashing'''
        # Set the modes for both the led and button GPIOs
        GPIO.setup(LED_PIN, GPIO.OUT)
        GPIO.setup(BUTTON_PIN, GPIO.IN)

        # Start the LED blink routine in idle-task
        self.blink_led = True

        # Stop the thread controlling the LED if any 
        self.stop_running_blink_thread()

        self.blink_thread = threading.Thread(target=self.waiting_blink_thread)
        self.blink_thread.start()

        # Setup button interrupts
        GPIO.add_event_detect(BUTTON_PIN, GPIO.BOTH, callback=self.handle_button_edge)


    def stop_running_blink_thread(self):
        if self.blink_thread:
            # A blinking thread is already active. Signal a stop and ,join() with the running thread
            self.signal_blink_thread_shutdown = True
            self.blink_thread.join()
            self.blink_thread = None
            self.signal_blink_thread_shutdown = False


    def waiting_blink_thread(self):
        # Remember to turn off the LED on exit
        print "Blink thread started"

        while not self.signal_blink_thread_shutdown:
            self.waiting_blink_program(valid_states=[STATE_WAITING, STATE_WAITING_DOWN])
            self.long_press_program(valid_states=[STATE_WAITING_LONG_DOWN])
            self.wait_execute_program(valid_states=[STATE_WAIT_EXECUTE])
            # In case nothing matches, we don't want to eat up all the CPU
            time.sleep(0.1)
           
        
        print "Blink thread shutdown"
    
    def waiting_blink_program(self, valid_states=None):
        while True:
            if self.signal_blink_thread_shutdown or not (self.system_state in valid_states):
                break

            GPIO.output(LED_PIN, LED_ON)
            time.sleep(BLINK_LENGTH)
            GPIO.output(LED_PIN, LED_OFF)
            time.sleep(BLINK_BETWEEN_INTERVAL)

            if self.signal_blink_thread_shutdown or not (self.system_state in valid_states):
                break

            GPIO.output(LED_PIN, LED_ON)
            time.sleep(BLINK_LENGTH)
            GPIO.output(LED_PIN, LED_OFF)
            time.sleep(BLINK_BETWEEN_INTERVAL)

            if self.signal_blink_thread_shutdown or not (self.system_state in valid_states):
                break

            time.sleep(BLINK_SET_INTERVAL)

    def long_press_program(self, valid_states=None):
        while True:
            if self.signal_blink_thread_shutdown or not (self.system_state in valid_states):
                break

            if time.time() - self.long_press_start_time > LONG_PRESS_REQUIRED_TIME_SEC:
                # Button was held for long enough
                GPIO.output(LED_PIN, LED_ON)
                time.sleep(BLINK_LENGTH)
            else:
                # Button needs to be held longer
                GPIO.output(LED_PIN, LED_ON)
                time.sleep(BLINK_LONG_PRESS_TIMER_INTERVAL)
                GPIO.output(LED_PIN, LED_OFF)
                time.sleep(BLINK_LONG_PRESS_TIMER_INTERVAL)

    def wait_execute_program(self, valid_states=None):
        while True:
            if self.signal_blink_thread_shutdown or not (self.system_state in valid_states):
                break

            if self.takeoff_time - time.time() > BLINK_DANGER_TIME_TO_TAKEOFF:
                # Still in warning mode - blink less frequently
                GPIO.output(LED_PIN, LED_ON)
                time.sleep(BLINK_WARNING_INTERVAL)
                GPIO.output(LED_PIN, LED_OFF)
                time.sleep(BLINK_WARNING_INTERVAL)
            else:
                # 15 sec to takeoff - blink furiously
                GPIO.output(LED_PIN, LED_ON)
                time.sleep(BLINK_DANGER_INTERVAL)
                GPIO.output(LED_PIN, LED_OFF)
                time.sleep(BLINK_DANGER_INTERVAL)




    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if self.system_state == STATE_WAITING_LONG_UP and now - self.waiting_long_up_start_time > WAITING_FOR_LONG_PRESS_TIMEOUT:
            print "Waiting too long before long press - returning to start state"
            self.system_state = STATE_WAITING


def init(mpstate):
    '''initialise module'''
    return ReturnModule(mpstate)
