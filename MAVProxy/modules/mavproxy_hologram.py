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
from pymavlink.dialects.v10 import common as mavlink
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

# Hologram-specific imports
from Hologram.HologramCloud import HologramCloud
from Exceptions.HologramError import PPPError
from scripts.hologram_util import handle_polling

# For Hologram Cloud REST API
from urllib2 import Request, urlopen, URLError
import json
import textwrap
import base64

MAX_SMS_LENGTH = 150

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
        self.add_command('hologram', self.cmd_hologram, "hologram module", ['status', 'start DEVICEKEY APIKEY', 'sendsms BODY', 'sendcurrent', 'enablesms', 'disablesms'])

        # Set hologram object and credentials to nothing to indicated not initialized
        self.hologram = None
        self.hologram_credentials = None
        self.apikey = None
        self.device_id = None

        self.hologram_network_type = 'cellular'
        self.sms_subscribed = False

        self.message_to_send = None
        self.enable_sms = False

    def usage(self):
        '''show help on command line options'''
        return "Usage: hologram <status|start DEVICEID APIKEY|sendsms BODY|enablesms|disablesms>"

    def cmd_hologram(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "start" and len(args) == 3:
            print(self.start(args[1], args[2]))
        elif args[0] == "sendsms" and len(args) == 2:
            print(self.send_sms(args[1]))
        elif args[0] == "sendcurrent" and len(args) == 1:
            print(self.sendcurrent())
        elif args[0] == "enablesms":
            print("Enabling command forwarding through SMS")
            self.enable_sms = True
        elif args[0] == "disablesms":
            print("Disabling command forwarding through SMS")
            self.enable_sms = False
        else:
            print(self.usage())

    def status(self):
        '''returns information about module'''
        if self.hologram is None:
            return "The 'hologram start' method must be called first before checking status"

        # Find basic information about connection and SIM 
        signal_strength = self.hologram.network.signal_strength
        rssi, qual = signal_strength.split(',')
        return("\n**** HOLOGRAM NOVA CONNECTION STATUS ****\nRSSI: " + str(rssi) + " | Quality: " + str(qual) + "\nOperator: " + str(self.hologram.network.operator) + "\nIMSI: " + str(self.hologram.network.imsi) + " | ICCID: " +  str(self.hologram.network.iccid) + "\nCloud Type: " + str(self.hologram) + " | Network Type: " + str(self.hologram.network_type) + "\n**** END STATUS ****\n")

    def handle_sms_received(self):
        recv = self.hologram.popReceivedSMS()
        if recv is not None:
            print 'Received message: ' + str(recv)

    def start(self, device_id, apikey):
        # Initialize credentials and hologram object
        if self.hologram_credentials is None:
            #self.hologram_credentials = {'devicekey': devicekey}
            try:
                self.hologram_credentials = dict()
                self.hologram = HologramCloud(self.hologram_credentials, network=self.hologram_network_type)
                print("HologramCloud connection initialized w/ credentials")
            except Exception as e:
                print("Error with starting hologram, error: " + str(e))
                print("Hologram module will start in API-only mode")
        
        self.apikey = apikey
        self.device_id = device_id

        print("API Key set to: " + str(apikey) + ", device id set to: " + str(device_id))

        return "Start command run: Hologram credentials = " + str(self.hologram_credentials) + " Hologram Object: " + str(self.hologram)

    def send_sms(self, message_body):
        if not message_body:
            print("No message body given - ignoring command to send")
            return
        elif not self.apikey:
            print("Cannot send message without hologram start being run")
            return
        
        data = {}
        data['deviceid'] = str(self.device_id)
        data['body'] = str(message_body)


        json_payload = json.dumps(data)

        print json_payload

        headers = {
            'Content-Type': 'application/json'
        }

        request = Request('https://dashboard.hologram.io/api/1/sms/incoming?apikey=' + self.apikey, data=json_payload, headers=headers)

        print("Sending message " + str(json_payload))

        try:
            #return None
            response_body = urlopen(request).read()
            return response_body
        except URLError as uerror:
            print("Received URL Error: " + str(uerror))

    def mavlink_packet_to_base64(self, packet):
        if not packet:
            print "No packet given to encode, ignoring command"
            return
        return base64.b64encode(packet.get_msgbuf())

    def sendcurrent(self):


        #msg = mavlink.MAVLink_command_long_message(self.target_system, self.target_component, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0).pack(self.master.mav)
        #print "Msg type: " + str(type(msg))
        #print "Msg : " + str(msg.get_msgbuf())
        #return self.send_sms(base64.b64encode(msg))
        return self.send_sms(self.mavlink_packet_to_base64(self.message_to_send))
        #return self.send_sms(self.mavlink_packet_to_base64(msg))
    
    def send_mavlink_packet_through_sms(self, packet):
        return self.send_sms(self.mavlink_packet_to_base64(packet))


    def idle_task(self):
        '''called rapidly by mavproxy'''

        now = time.time()
        if now-self.last_checked > self.pop_sms_interval:
            self.last_checked = now
            #msg = mavlink.MAVLink_command_long_message(self.target_system, self.target_component, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)#.pack(self.master.mav)
            #print "Msg : " + str(msg)
            #self.master.mav.send(msg)



            if self.hologram and self.hologram_credentials:
                #print("Checking for sms...")
                msg_object = self.hologram.popReceivedSMS()
                while msg_object is not None:
                    print(msg_object)
                    print("SMS Received: " + msg_object.message)
                    decoded = base64.b64decode(msg_object.message) 
                    print("Base64 decoded: " + decoded)
                    packet = self.master.mav.parse_char(decoded)
                    print("Packet received:" + str(packet))

                    # If we really received a MAVLink packet, forward it to the aircraft
                    if packet:
                        print("Forwarding packet to the aircraft")
                        self.master.mav.send(packet)
                    
                    # Attempt to get another SMS from the queue
                    msg_object = self.hologram.popReceivedSMS()


            else:
                pass


    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        pass
        #if m.get_type() == 'ATTITUDE':
        #    self.message_to_send = m
    
    def handle_slave_command(self, m):
        # Whitelist certain packets
        if m.get_type() in ['SET_MODE', 'COMMAND_LONG']:
            if self.enable_sms:
                print "Setting message to send = " + str(m)
                self.send_mavlink_packet_through_sms(m)
            else:
                print "Received command but sms command forwarding is disabled: use hologram enablesms to start"
        # Ignore blacklisted packets but print those we might have missed
        elif m.get_type() not in ['HEARTBEAT', 'REQUEST_DATA_STREAM']:
            print "Received command: " + str(m) + " - but these are not whitelisted to forward"




def init(mpstate):
    '''initialise module'''
    return HologramModule(mpstate)




