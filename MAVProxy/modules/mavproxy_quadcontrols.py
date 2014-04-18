#!/usr/bin/env python
'''quadcopter control commands'''

import time, os, struct
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class QuadcontrolsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(QuadcontrolsModule, self).__init__(mpstate, "quadcontrols", "additional controls for quadcopter handling", public = True)
        self.add_command('movez', self.cmd_movez, "Move forwards or backwards", ['-100 - 100'])
        self.add_command('strafe', self.cmd_strafe, "Strafe left or right", ['-100 - 100'])
        self.add_command('yaw', self.cmd_yaw, "Yaw left or right", ['-100 - 100'])


    # Global variables used to control the vehicle, change these depending on your controller or APM
    chanRoll=1
    chanPitch=2
    chanThrottle=3
    chanYaw=4
    #goalAltitude=-1 # Value -1 means it's not yet set
    minValue=800  # Min value of all sticks.
    midValue=1500 # Middle value of all sticks.
    maxValue=2200 # Max value.
    debugMode=True


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


#def cmd_bend(args):
#    if len(args) != 2:
#        print ("Usage: bend <procent yaw> <procent pitch> (between -100 and 100)")
#        return
#    yawP = int(args[0])
#    pitchP = int(args[1])
#    if (yawP > 100 or yawP < -100 or pitchP > 100 or pitchP < -100):
#        print ("Usage: bend <procent yaw> <procent pitch> (between -100 and 100)")
#        return
#    cmd_movecopter([chanYaw, yawP])
#    cmd_movecopter([chanPitch, pitchP])

#def cmd_hover(args):
#    print ("Resetting vehicle to stand still loiter mode")
#    cmd_movecopter([chanRoll, 0])
#    cmd_movecopter([chanPitch, 0])
#    cmd_movecopter([chanThrottle, 0])
#    cmd_movecopter([chanYaw, 0])
#    cmd_althold

    
# Help function to movez, strafe and yaw
    def cmd_movecopter(self, args):
        if len(args) != 2:
            print("Error using cmd_movecopter, wrong number of arguments")
            return
        channel = int(args[0])
        valProcent = int(args[1])
        throttle=self.midValue
   # if abs(valProcent) <= procentZone:
   #     rawValue = midValue
   # elif valProcent > 0:
        if valProcent > 0:
            rawValue=self.midValue+(self.calcValue(abs(valProcent)))
        else:
            rawValue=self.midValue-(self.calcValue(abs(valProcent)))
        if self.debugMode:
            print("Throttling at ", throttle, " with value ", rawValue, " on channel ", channel)
        rc=super(QuadcontrolsModule, self).module('rc')
        rc.cmd_rc([self.chanThrottle, throttle])
        rc.cmd_rc([channel, rawValue])
        print("Done")


def init(mpstate):
    '''initialise module'''
    return QuadcontrolsModule(mpstate)



