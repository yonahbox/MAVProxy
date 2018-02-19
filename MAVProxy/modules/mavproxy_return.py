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
import time

# Constants
BUTTON_RELEASED = 0
BUTTON_PRESSED = 1

LED_PIN = "P9_22"
BUTTON_PIN = "P9_19"

BLINK_WAITING = 100


# Blink interval constants (sec)
BLINK_SET_INTERVAL = 1.5
BLINK_BETWEEN_INTERVAL = 0.1
BLINK_LENGTH = 0.1

class ReturnModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(ReturnModule, self).__init__(mpstate, "return", "return module")
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()
        self.button_state = BUTTON_RELEASED
        self.verbose = False
        self.add_command('return', self.cmd_return, "return module", ['start'])
        self.blink_led = False
        self.blink_state = BLINK_WAITING
        self.blink_thread = None
        self.signal_blink_thread_shutdown = False

    def usage(self):
        '''show help on command line options'''
        return "Usage: return <start>"

    def cmd_return(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "start":
            print(self.start())
        else:
            print(self.usage())

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

        while True:

            if self.signal_blink_thread_shutdown:
                break

            GPIO.output(LED_PIN, 0)
            time.sleep(BLINK_LENGTH)
            GPIO.output(LED_PIN, 1)
            time.sleep(BLINK_BETWEEN_INTERVAL)

            if self.signal_blink_thread_shutdown:
                break

            GPIO.output(LED_PIN, 0)
            time.sleep(BLINK_LENGTH)
            GPIO.output(LED_PIN, 1)
            time.sleep(BLINK_BETWEEN_INTERVAL)

            if self.signal_blink_thread_shutdown:
                break

            time.sleep(BLINK_SET_INTERVAL)
        
        print "Blink thread shutdown"


    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()


def init(mpstate):
    '''initialise module'''
    return ReturnModule(mpstate)
