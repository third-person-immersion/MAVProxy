#!/usr/bin/env python
'''quadcopter control commands'''

import time, os, struct
import threading
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class QuadcontrolsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(QuadcontrolsModule, self).__init__(mpstate, "quadcontrols", "additional controls for quadcopter handling", public = True)
        self.add_command('movez', self.cmd_movez, "Move forwards or backwards", ['-100 - 100'])
        self.add_command('strafe', self.cmd_strafe, "Strafe left or right", ['-100 - 100'])
        self.add_command('movey', self.cmd_movey, "Move up or down", ['-100 - 100'])        
        self.add_command('yaw', self.cmd_yaw, "Yaw left or right", ['-100 - 100'])
        self.add_command('hover', self.cmd_hover, "Resets the vehicle to stand still in alt_hold mode")
        self.add_command('bend', self.cmd_bend, "Moves the vehicle in a curve movement", ['-100 - 100'])
        self.add_command('setalt', self.cmd_setAlt, "Sets the altitude", [0-100])
        self.add_command('startalt', self.cmd_startAlt, 'Initializes the altitude hold thread')


    # Global variables used to control the vehicle, change these depending on your controller or APM
    chanRoll=1
    chanPitch=2
    chanThrottle=3
    chanYaw=4
    goalAltitude=-1 # Value -1 means it's not yet set
    sweetSpot=0.3 # Sweet spot for altitude control
    minValue=800  # Min value of all sticks.
    midValue=1500 # Middle value of all sticks.
    maxValue=2200 # Max value of all sticks.
    debugMode=True # If True several debug variables gets printed


    def cmd_startAlt(self, unused):
        print("Altitude thread starting...")
        t1 = threading.Thread(target=self.checkAlt)
        t1.daemon = True
        t1.start()
        print("Altitude thread started")

    # Check if the altitude is good, runs in a seperate thread    
    def checkAlt(self):
        while True:
            time.sleep(0.5)
            if self.goalAltitude != -1:
                alt=float(self.mpstate.status.altitude)
                if self.debugMode == True:
                    print("alt: ", alt)
                    print("goal ", self.goalAltitude)
                if (abs(self.goalAltitude-alt)>self.sweetSpot):
                    if (alt < self.goalAltitude):
                        if self.debugMode == True:
                            print("Altitude too low, adjusting...")
                        self.cmd_movey([14]) # 14 correspond to around 1600
                    elif (alt > self.goalAltitude):
                        if self.debugMode == True:
                            print("Altitude too high, adjusting...")
                        self.cmd_movey([-29]) # -29 correspond to around 1300
        
    def cmd_setAlt(self, altitude):
        self.goalAltitude=float(altitude[0])
        print("New altitude set")

    def calcValue(self, procent):
        valproc=float(procent)/100
        value=(self.maxValue-self.midValue)*valproc
        return value

    def cmd_movez(self, args):
        if len(args) != 1:
            print ("Usage: movez <procent value (between -100 and 100)>")
            return
        val = int(args[0])
        if (val > 100 or val < -100):
            print ("Usage: movez <procent value (between -100 and 100)>")
            return
        self.cmd_movecopter([self.chanPitch, val])
    
    def cmd_movey(self, args):
        if len(args) != 1:
            print ("Usage: movey <procent value (between -100 and 100)>")
            return
        val = int(args[0])
        if (val > 100 or val < -100):
            print ("Usage: movey <procent value (between -100 and 100)>")
            return
        self.cmd_movecopter([self.chanThrottle, val])

    def cmd_strafe(self, args):
        if len(args) != 1:
            print ("Usage: strafe <procent value (between -100 and 100)>")
            return
        val = int(args[0])
        if (val > 100 or val < -100):
            print ("Usage: strafe <procent value (between -100 and 100)>")
            return
        self.cmd_movecopter([self.chanRoll, val])

    def cmd_yaw(self, args):
        if len(args) != 1:
            print ("Usage: yaw <procent value (between -100 and 100)>")
            return
        val = int(args[0])
        if (val > 100 or val < -100):
            print ("Usage: yaw <procent value (between -100 and 100)>")
            return
        self.cmd_movecopter([self.chanYaw, val])


    def cmd_bend(self, args):
        if len(args) != 2:
            print ("Usage: bend <procent yaw> <procent pitch> (between -100 and 100)")
            return
        yawP = int(args[0])
        pitchP = int(args[1])
        if (yawP > 100 or yawP < -100 or pitchP > 100 or pitchP < -100):
            print ("Usage: bend <procent yaw> <procent pitch> (between -100 and 100)")
            return
        self.cmd_movecopter([self.chanYaw, yawP])
        self.cmd_movecopter([self.chanPitch, pitchP])

    def cmd_hover(self, args):
        print("Resetting vehicle to stand still in alt_hold mode")
        self.cmd_movecopter([self.chanRoll, 0])
        self.cmd_movecopter([self.chanPitch, 0])
        self.cmd_movecopter([self.chanThrottle, 0])
        self.cmd_movecopter([self.chanYaw, 0])
        self.mpstate.functions.process_stdin("mode alt_hold")
        print("Done")

    
    # Help function to movez, strafe, movey, hover and yaw
    def cmd_movecopter(self, args):
        if len(args) != 2:
            print("Error using cmd_movecopter, wrong number of arguments")
            return
        channel = int(args[0])
        valProcent = int(args[1])

        if valProcent > 0:
            rawValue=self.midValue+(self.calcValue(abs(valProcent)))
        else:
            rawValue=self.midValue-(self.calcValue(abs(valProcent)))
        if self.debugMode:
            print("Throttling with value ", rawValue, " on channel ", channel)
        self.mpstate.functions.process_stdin("rc %d %d" % (channel, rawValue)) 


def init(mpstate):
    '''initialise module'''
    return QuadcontrolsModule(mpstate)



