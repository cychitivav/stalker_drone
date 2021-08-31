#!/usr/bin/python

import rospy
import tf.transformations as tf
import numpy as np
import mavros
from mavros import command, setpoint as sp
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest


class move():
    def __init__(self):
        rospy.wait_for_service('/mavros/cmd/arming')        
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        rospy.wait_for_service('/mavros/cmd/takeoff')        
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        
        self.arm()
        self.takeoff()
        
    def arm(self, value=True):
        req = CommandBoolRequest()
        req.value = value
        
        return self.arm_client(req)
    
    def takeoff(self, min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=3.0):
        req = CommandTOLRequest()
        req.min_pitch = min_pitch
        req.yaw = yaw
        req.latitude = latitude
        req.longitude = longitude
        req.altitude = altitude
        
        return self.takeoff_client(req)


if __name__ == "__main__":
    rospy.init_node('sender', anonymous=True)

    rospy.loginfo("Node init")

    move()

    rospy.spin()
