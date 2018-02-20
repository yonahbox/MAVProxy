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
TRY_LOAD_MISSION_TIMEOUT = 3

# Blink interval constants (sec)
BLINK_SET_INTERVAL = 1.5
BLINK_BETWEEN_INTERVAL = 0.1
BLINK_LENGTH = 0.1
BLINK_LONG_PRESS_TIMER_INTERVAL = 0.15
BLINK_WARNING_INTERVAL = 0.1
BLINK_DANGER_INTERVAL = 0.05

BLINK_DANGER_TIME_TO_TAKEOFF = 15

WAYPOINT_FILE_PATH = "return_waypoints.txt"


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
        self.try_load_mission_start_time = time.time()        


        # DEBUG REMOVE!        
        # self.try_load_mission()

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
                # Time is up here to prevent race condition with idle_task to check mission start time
                self.try_load_mission_start_time = time.time() 
                self.system_state = STATE_TRY_LOAD_MISSION
                print "Trying to load mission!"
                result = self.try_load_mission()
                if result:
                    # External timers will track if we've truly loaded the mission and transition us accordingly
                    print "Waiting for timeout before checking mission load state"
                    pass
                else:
                    # Failed to find the file or something - go back to start state
                    print "Failed to try load mission, possibly WP file not found"
                    self.system_state = STATE_WAITING


            else:
                self.system_state = STATE_WAITING
                print "Button was not held long enough!"
        elif (self.system_state == STATE_TRY_LOAD_MISSION or self.system_state == STATE_WAIT_EXECUTE) and state == BUTTON_PRESSED:
            print "Cancelling return mission load / execute"
            self.system_state = STATE_WAITING


    def try_load_mission(self):
        wp_module = None
        for (module, _) in self.mpstate.modules:
            if module.name == "wp":
                wp_module = module
        
        if not wp_module:
            print "Unable to load the waypoint module"
            return False

        if not os.path.isfile(WAYPOINT_FILE_PATH):
            print "Cannot find waypoint file " + str(WAYPOINT_FILE_PATH)
            return False

        print "Loading waypoints!"
        wp_module.load_waypoints(WAYPOINT_FILE_PATH)
        print "Done loading waypoints."
        print "Trying to send waypoints to aircraft"

        # Set the wp module so that we can access it later
        self.wp_module = wp_module

        # Mission loaded confirmation will be done inside idle task
        return True 




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
        self.blink_thread.daemon = True
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
        elif self.system_state == STATE_TRY_LOAD_MISSION:
            if now - self.try_load_mission_start_time > TRY_LOAD_MISSION_TIMEOUT:
                # We were trying to load the mission and the timeout has been exceeded - check if mission was loaded
                # We check loading_waypoints if we've completed sending all waypoints,
                #  and confirm that the last time a waypoint was uploaded was
                #  after WE started asking mavwp to upload the mission
                if self.wp_module and (not self.wp_module.loading_waypoints) and (self.wp_module.loading_waypoint_lasttime > self.try_load_mission_start_time):
                    # We have a handle on the wp module and we are DONE loading waypints
                    self.system_state = STATE_WAIT_EXECUTE
                    print "Waiting to execute - takeoff is " + str(TIME_TO_TAKEOFF_SEC) + " seconds from now!"
                    self.takeoff_time = now + TIME_TO_TAKEOFF_SEC
                else:
                    # We encountered some error in loading waypoints in the given timeframe
                    print "Waypoints not loaded!"
                    self.system_state = STATE_WAITING
        elif self.system_state == STATE_WAIT_EXECUTE:
            if self.takeoff_time - now < 0:
                # Takeoff time has passed - execute takeoff
                print "Takeoff!"
                self.system_state = STATE_WAITING





def init(mpstate):
    '''initialise module'''
    return ReturnModule(mpstate)
