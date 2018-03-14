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
if os.name != 'nt':
    from Hologram.HologramCloud import HologramCloud
    from Exceptions.HologramError import PPPError
    from scripts.hologram_util import handle_polling

# For Hologram Cloud REST API
from urllib2 import Request, urlopen, URLError
import json
import textwrap
import base64

MAX_SMS_LENGTH = 150

# Optimizations: 
#1. Only send messages that have changed since last send
#2. Message lists should be a config file or parameter
#3. Should have a message to lower or raise these rates

'''
MSG_WHITELISTS = [['ATTITUDE',
                  'EKF_STATUS_REPORT', 
                  'GLOBAL_POSITION_INT', 
                  'GPS_RAW_INT', 
                  'HEARTBEAT', 
                  'HWSTATUS', 
                  'MISSION_CURRENT', 
                  'NAV_CONTROLLER_OUTPUT',
                  'POSITION_TARGET_GLOBAL_INT', 
                 ],
                 ['POWER_STATUS', 
                  'RAW_IMU', 
                  'RC_CHANNELS', 
                  'SCALED_IMU2', 
                  'STATUSTEXT',
                 ],
                 ['SYSTEM_TIME', 
                  'SYS_STATUS', 
                  'VFR_HUD', 
                  'VIBRATION',
                  'WIND']]
'''
MSG_WHITELISTS = [['ATTITUDE',
                  'EKF_STATUS_REPORT', 
                  'GPS_RAW_INT', 
                  'HEARTBEAT', 
                  'MISSION_CURRENT', 
                  'NAV_CONTROLLER_OUTPUT',
                  'POWER_STATUS', 
                  'RANGEFINDER',
                  'VFR_HUD']]


class HologramModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(HologramModule, self).__init__(mpstate, "hologram", "Hologram Nova command handling")
        self.status_callcount = 0
        self.pop_sms_interval = 2 # seconds
        self.last_checked = time.time()
        self.last_checked_telem = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.hologram_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('hologram', self.cmd_hologram, "hologram module", ['status', 'start DEVICEKEY APIKEY', 'sendsms BODY', 'sendcurrent', 'enablesms', 'disablesms', 'enablereceive', 'disablereceive', 'enabletelem FREQUENCY_SECONDS', 'disabletelem'])

        # Set hologram object and credentials to nothing to indicated not initialized
        self.hologram = None
        self.hologram_credentials = None
        self.apikey = None
        self.device_id = None

        self.hologram_network_type = 'cellular'
        self.sms_subscribed = False

        self.message_to_send = None
        self.enable_sms = False
        self.enable_sms_receive = True
        self.enable_telem = False
        self.telem_frequency = 10

        # Initialize our send data thread status tracker to False (not running)
        self.send_data_message_thread_running = False

    def usage(self):
        '''show help on command line options'''
        return "Usage: hologram <status|start DEVICEID APIKEY|sendsms BODY|enablesms|disablesms|enablereceive|disablereceive|enabletelem FREQUENCY_SECONDS|disabletelem>"

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
        elif args[0] == "enablereceive":
            print("Enabling command receiving through SMS")
            self.enable_sms_receive = True
        elif args[0] == "disablereceive":
            print("Disabling command receiving through SMS")
            self.enable_sms_receive = False
        elif args[0] == "enabletelem" and len(args) == 2:
            print("Enabling telemetry output through mobile data")
            self.enable_telem = True
            self.telem_frequency = int(args[1])
        elif args[0] == "disablereceive":
            print("Disabling telemetry output through mobile data")
            self.enable_telem = False
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
        if os.name == 'nt':
            print "SMS reception not supported on Windows"
            return
        recv = self.hologram.popReceivedSMS()
        if recv is not None:
            print 'Received message: ' + str(recv)

    def start(self, device_id, apikey):
        # Initialize credentials and hologram object
        if self.hologram_credentials is None and os.name != 'nt':
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

        print("Message body length: " + str(len(message_body)))
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

    def send_data_message(self, msg):
        # For threading purposes: set a variable that this thread is currently running
        self.send_data_message_thread_running = True

        if os.name == 'nt':
            print "Hologram data sending not supported on Windows"
            return
        print("Sending hologram message: " + str(msg))
        try:
            recv = self.hologram.sendMessage(msg, timeout=10)
            print("Received hologram message response: " + str(recv))
        except Exception as e:
            print "Exception received when trying to send message through Hologram: " + str(e)

       # For threading purposes: indicate that this thread is completed
        self.send_data_message_thread_running = False

    def idle_task(self):
        '''called rapidly by mavproxy'''

        now = time.time()
        if now-self.last_checked > self.pop_sms_interval and os.name != 'nt':
            self.last_checked = now

            if self.hologram and self.hologram_credentials and self.enable_sms_receive:
                #print("Checking for sms...")
                msg_object = self.hologram.popReceivedSMS()
                while msg_object is not None:
                    print(msg_object)
                    print("SMS Received: " + msg_object.message)
                    decoded = base64.b64decode(msg_object.message) 
                    #print("Base64 decoded: " + decoded)

                    # Clear the buffer before each parse
                    self.master.mav.buf = bytearray()
                    self.master.mav.buf_index = 0
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

        if self.enable_telem and now-self.last_checked_telem > self.telem_frequency and os.name != 'nt':
            self.last_checked_telem = now
            # Gather all the messages we care about into a dict
            compact_telemetry = {}
            for msg_whitelist in MSG_WHITELISTS:
                for index, msg_type in enumerate(msg_whitelist):
                    if msg_type in self.mpstate.status.msgs:
                        compact_telemetry[str(index)] = self.mavlink_packet_to_base64(self.mpstate.status.msgs[msg_type])
                        print("MSG TYPE " + str(msg_type) + " LENGTH = " + str(len(compact_telemetry[str(index)])))
            
                print "Message length: " + str(len(json.dumps(compact_telemetry)))
                if compact_telemetry:
                    if not self.send_data_message_thread_running:
                        # The single thread that we want to be sending this telemetry message is not running
                        # We can spawn one safely
                        self.send_data_message(str(compact_telemetry))
                    else:
                        print("Cannot send message - old data message thread is still sending message")
                else:
                    print "No telemetry data to send - skipping.."
                print "-----------"


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




