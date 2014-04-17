#!/usr/bin/env python
'''rc command handling'''

import time, os, struct
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class RCModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(RCModule, self).__init__(mpstate, "rc", "rc command handling", public = True)
        self.override = [ 0 ] * 8
        self.last_override = [ 0 ] * 8
        self.override_counter = 0
        self.add_command('rc', self.cmd_rc, "RC input control", ['<1|2|3|4|5|6|7|8|all>'])
        self.add_command('switch', self.cmd_switch, "flight mode switch control", ['<0|1|2|3|4|5|6>'])
        self.add_command('movez', self.cmd_movez, "move z axis", ['-100 - 100'])
        self.add_command('strafe', self.cmd_strafe, "Strafe left or right", ['-100 - 100'])
        self.add_command('yaw', self.cmd_yaw, "Yaw left or right", ['-100 - 100'])
        if self.sitl_output:
            self.override_period = mavutil.periodic_event(20)
        else:
            self.override_period = mavutil.periodic_event(1)

    def idle_task(self):
        if self.override_period.trigger():
            if (self.override != [ 0 ] * 8 or
                self.override != self.last_override or
                self.override_counter > 0):
                self.last_override = self.override[:]
                self.send_rc_override()
                if self.override_counter > 0:
                    self.override_counter -= 1

    def send_rc_override(self):
        '''send RC override packet'''
        if self.sitl_output:
            buf = struct.pack('<HHHHHHHH',
                              *self.override)
            self.sitl_output.write(buf)
        else:
            self.master.mav.rc_channels_override_send(self.target_system,
                                                           self.target_component,
                                                           *self.override)

    def cmd_switch(self, args):
        '''handle RC switch changes'''
        mapping = [ 0, 1165, 1295, 1425, 1555, 1685, 1815 ]
        if len(args) != 1:
            print("Usage: switch <pwmvalue>")
            return
        value = int(args[0])
        if value < 0 or value > 6:
            print("Invalid switch value. Use 1-6 for flight modes, '0' to disable")
            return
        if self.vehicle_type == 'copter':
            default_channel = 5
        else:
            default_channel = 8
        if self.vehicle_type == 'rover':
            flite_mode_ch_parm = int(self.get_mav_param("MODE_CH", default_channel))
        else:
            flite_mode_ch_parm = int(self.get_mav_param("FLTMODE_CH", default_channel))
        self.override[flite_mode_ch_parm - 1] = mapping[value]
        self.override_counter = 10
        self.send_rc_override()
        if value == 0:
            print("Disabled RC switch override")
        else:
            print("Set RC switch override to %u (PWM=%u channel=%u)" % (
                value, mapping[value], flite_mode_ch_parm))


# Global variables used to control the vehicle, change these depending on your controller or APM
    chanRoll=1
    chanPitch=2
    chanThrottle=3
    chanYaw=4
    goalAltitude=-1 # Value -1 means it's not yet set
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
        self.cmd_rc([self.chanThrottle, throttle])
        self.cmd_rc([channel, rawValue])


    def cmd_rc(self, args):
        '''handle RC value override'''
        if len(args) != 2:
            print("Usage: rc <channel|all> <pwmvalue>")
            return
        value = int(args[1])
        if value == -1:
            value = 65535
        if args[0] == 'all':
            for i in range(8):
                self.override[i] = value
        else:
            channel = int(args[0])
            self.override[channel - 1] = value
            if channel < 1 or channel > 8:
                print("Channel must be between 1 and 8 or 'all'")
                return
        self.override_counter = 10
        self.send_rc_override()

def init(mpstate):
    '''initialise module'''
    return RCModule(mpstate)



