#!/usr/bin/env python3
###########################################################################
# Author: Hannibal Paul
# Date: 2023-03-05
###########################################################################
# Planar (horizontal) thrust control for a 3 rotor arrangement around a UAV running MAVROS 
# -Reads RC input values (roll, pitch stick)
# -Controls actuators connected to control group 2 of MAVROS.
# -Assign gimbal roll, gimbal pitch and gimbal yaw to actuator rotors in QGC
###########################################################################
import rospy
import numpy as np
from mavros_msgs.msg import RCIn, ActuatorControl
#--------------------------------------------------------------------------

###########################################################################
class ThrustControl:
    def __init__(self):
        #Subscribers, Publishers
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback, queue_size=1)
        self.pub = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=1)

        #Variables
        self.r_chan = 0
        self.p_chan = 1
        self.stick_center = [1504, 1503, 1501, 1502]
        self.ux = 0
        self.uy = 0

        #stick min and max position for fan control
        #changing these values will change the fan rate
        self.stick_min = -200
        self.stick_max = 600

        #Physical values
        # Rotor attachment angles around the UAV (for 3 rotors)
        #             ^ 0deg
        #       R2    |      R3
        #             |
        # 270deg<------------> 90 deg
        #             |
        #           R1| 180deg
        #self.rotor_facings = [0, 120, 240] #R1, R2, R3
        self.rotor_facings = [180, 300, 60] #R1, R2, R3

        #Publisher variables
        self.actuator_control = ActuatorControl()
        self.seq = 0
    #--------------------------------------------------------------------------
    def rc_callback(self, msg):
        r_stick = msg.channels[self.r_chan]
        p_stick = msg.channels[self.p_chan]
        #  ^x (p_stick)
        #  |
        #  --->y (r_stick)
        self.ux = p_stick - self.stick_center[self.p_chan]
        self.uy = r_stick - self.stick_center[self.r_chan]
    #--------------------------------------------------------------------------
    def thrust_control(self):
        #Map
        rad_angles = np.array(self.rotor_facings) * np.pi/180
        out_xs = self.ux * np.cos(rad_angles) - self.uy * np.sin(rad_angles)
        ctrl = (1.0 - -1.0) * (out_xs - self.stick_min) / (self.stick_max - self.stick_min) + -1.0
        
        #Control
        ctrl.resize(8)
        self.actuator_control.header.stamp = rospy.Time.now()
        self.actuator_control.header.seq = self.seq
        self.actuator_control.group_mix = 2
        self.actuator_control.controls = ctrl
        self.pub.publish(self.actuator_control)
        self.seq += 1
#--------------------------------------------------------------------------

###########################################################################
def main():
    rospy.init_node('planar_thrust', anonymous=True)
    rospy.loginfo('Planar thrust control active')
    tc = ThrustControl()
    rate = rospy.Rate(20) #in Hz
    while not rospy.is_shutdown():
        tc.thrust_control()
        rate.sleep()
    rospy.loginfo("Planar thrust control disabled")
#--------------------------------------------------------------------------

###########################################################################
if __name__ == '__main__':
    try:
        main()
    except:
        print("Program terminated")
#--------------------------------------------------------------------------
