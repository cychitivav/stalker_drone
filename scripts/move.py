#!/usr/bin/python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, \
    ParamSet, ParamGet


class move():
    def __init__(self):
        self.setpoint_raw = PositionTarget()
        self.rate = rospy.Rate(20)
        self.pose_stamped = PoseStamped()
        self.state = State()
        
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(5), self.stay_armed_stay_offboard_cb)
        self.stay_armed_stay_offboard_timer.shutdown()
        
        rospy.wait_for_service('/mavros/cmd/arming')        
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/mavros/set_mode')
        self.mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
        
        self.setpoint_raw_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        
        
        self.takeoff()
        
    def state_cb(self, msg):
        self.state = msg
        rospy.logdebug('State updated')
        
    def arm(self, value=True):
        req = CommandBoolRequest()
        req.value = value
        
        return self.arm_client(req)
    
    def takeoff(self, h=3, precision=0.05):
        self.arm(True)
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
        while True:
            while not (self.state.armed and self.state.mode == 'OFFBOARD'):
                self.rate.sleep()
            if self.state.mode == 'OFFBOARD':
                break
            
        self.set_point(1987, pz=h) # vx vy vz z yaw_rate
        rospy.loginfo('Taking off!!!')
        while True:
            if abs(self.pose_stamped.pose.position.z - h) < precision:
                break
        self.set_point(1987)
        
    def stay_armed_stay_offboard_cb(self, event):
        if self.state.mode != 'OFFBOARD':
            self.request_mode('OFFBOARD')
        elif not self.state.armed:
            self.arm(True)
    
    def set_point(self, mask, px=0, py=0, pz=0, vx=0, vy=0, vz=0, ax=0, ay=0, az=0, Y=0, rY=0):
        self.setpoint_raw.coordinate_frame = 8
        self.setpoint_raw.type_mask = mask
        
        self.setpoint_raw.position.x = px
        self.setpoint_raw.position.y = py
        self.setpoint_raw.position.z = pz
        
        self.setpoint_raw.velocity.x = vx
        self.setpoint_raw.velocity.y = vy
        self.setpoint_raw.velocity.z = vz      
        
        self.setpoint_raw.acceleration_or_force.x = ax  
        self.setpoint_raw.acceleration_or_force.y = ay
        self.setpoint_raw.acceleration_or_force.z = az  
        
        self.setpoint_raw.yaw = Y
        self.setpoint_raw.yaw_rate = rY

        self.setpoint_raw_publisher.publish(self.setpoint_raw)
        
    def request_mode(self, mode='OFFBOARD'):
        rospy.sleep(2)
        return self.mode_client(custom_mode=mode)    
    
    def pose_stamped_cb(self, msg):
        self.pose_stamped = msg
        rospy.logdebug('Pose updated')


if __name__ == "__main__":
    rospy.init_node('sender', anonymous=True)

    rospy.loginfo("Node init")

    move()

    rospy.spin()
