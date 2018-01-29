#!/usr/bin/env python
'''
1. Send MAVLink messages to a connected Hologram Nova to the Hologram Cloud
2. Receive MAVLink commands over Hologram using Hologram Cloud SMS

Sriram Sami & Yonah, January 2018
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

# Hologram-specific imports
from Hologram.HologramCloud import HologramCloud
from Exceptions.HologramError import PPPError
from scripts.hologram_util import handle_polling


class HologramModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(HologramModule, self).__init__(mpstate, "hologram", "Hologram Nova command handling")
        self.status_callcount = 0
        self.pop_sms_interval = 2 # seconds
        self.last_checked = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.hologram_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('hologram', self.cmd_hologram, "hologram module", ['status', 'start DEVICEKEY'])

        # Set hologram object and credentials to nothing to indicated not initialized
        self.hologram = None
        self.hologram_credentials = None

        self.hologram_network_type = 'cellular'
        self.sms_subscribed = False

    def usage(self):
        '''show help on command line options'''
        return "Usage: hologram <status|start>"

    def cmd_hologram(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "start":
            print(self.start(args[1:]))
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        if self.hologram is None:
            # Create a non-authenticated hologram object temporarily
            self.hologram = HologramCloud(dict(), network=self.hologram_network_type)

        # Find basic information about connection and SIM 
        signal_strength = self.hologram.network.signal_strength
        rssi, qual = signal_strength.split(',')
        return("\n**** HOLOGRAM NOVA CONNECTION STATUS ****\nRSSI: " + str(rssi) + " | Quality: " + str(qual) + "\nOperator: " + str(self.hologram.network.operator) + "\nIMSI: " + str(self.hologram.network.imsi) + " | ICCID: " +  str(self.hologram.network.iccid) + "\nCloud Type: " + str(self.hologram) + " | Network Type: " + str(self.hologram.network_type) + "\n**** END STATUS ****\n")

    def handle_sms_received(self):
        recv = self.hologram.popReceivedSMS()
        if recv is not None:
            print 'Received message: ' + str(recv)

    def start(self, devicekey):
        # Initialize credentials and hologram object
        if self.hologram_credentials is None:
            #self.hologram_credentials = {'devicekey': devicekey}
            self.hologram_credentials = dict()
            self.hologram = HologramCloud(self.hologram_credentials, network=self.hologram_network_type)
            print("HologramCloud connection initialized w/ credentials")

        return "Start command run: Hologram credentials = " + str(self.hologram_credentials) + " Hologram Object: " + str(self.hologram)
    

    def idle_task(self):
        '''called rapidly by mavproxy'''

        now = time.time()
        if now-self.last_checked > self.pop_sms_interval:
            self.last_checked = now
            if self.hologram and self.hologram_credentials:
                print("Checking for sms...")
                msg_object = self.hologram.popReceivedSMS()
                print(msg_object)
                print("SMS Received: " + msg_object.message)
            else:
                print("No hologram object or no credentials")

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1

def init(mpstate):
    '''initialise module'''
    return HologramModule(mpstate)




