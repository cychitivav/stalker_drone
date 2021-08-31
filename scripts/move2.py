#!/usr/bin/python

import rospy
import tf.transformations as tf
import numpy as np
import mavros
from mavros import command, setpoint as sp
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, \
    ParamSet, ParamGet


class move():
    def __init__(self):
        self.takeoff()
        
    def arm(self, state=True):
        return command.arming(value=state)
    
    def set_home(self, current_gps, latitude, longitude, altitude):
        return command.set_home(current_gps=current_gps,
                                latitude=latitude,
                                longitude=longitude,
                                altitude=altitude)
        
    def takeoff(self, min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=3):
        return command.takeoff( min_pitch=min_pitch,
                                yaw=yaw,
                                latitude=latitude,
                                longitude=longitude,
                                altitude=altitude)
        
    def do_takeoff_cur_gps(self, args):
        done_evt = threading.Event()
        def fix_cb(fix):
            print("Taking-off from current coord: Lat:", fix.latitude,
                "Long:", fix.longitude)
            print_if(args.verbose, "With desired Altitude:", args.altitude,
                    "Yaw:", args.yaw, "Pitch angle:", args.min_pitch)

            try:
                ret = command.takeoff(min_pitch=args.min_pitch,
                                yaw=args.yaw,
                                latitude=fix.latitude,
                                longitude=fix.longitude,
                                altitude=args.altitude)
            except rospy.ServiceException as ex:
                fault(ex)

            _check_ret(args, ret)
            done_evt.set()

        topic = _find_gps_topic(args, "takeoff")
        if topic is None:
            fault("NavSatFix topic not exist")

        sub = rospy.Subscriber(topic, NavSatFix, fix_cb)
        if not done_evt.wait(10.0):
            fault("Something went wrong. Topic timed out.")
            
    def publish_once(self, pub, msg):
        pub.publish(msg)
        rospy.sleep(0.2)
        self.enable_offboard()


    def enable_offboard(self):
        print("Requesting OFFBOARD mode...")
        try:
            set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
            ret = set_mode(custom_mode="OFFBOARD")
        except rospy.ServiceException as ex:
            fault(ex)

        if not ret.mode_sent:
            print("Request failed. Check mavros logs")
        else:
            print("OFFBOARD mode are set.")


    def do_local_pos(self, x=0, y=0, z=0, yaw=0):
        pub = sp.get_pub_position_local(queue_size=10, latch=True)

        pos = sp.PoseStamped(header=sp.Header(frame_id='mavsetp', stamp=rospy.get_rostime()))
        pos.pose.position = sp.Point(x=x, y=y, z=z)

        q = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = sp.Quaternion(*q)

        self.publish_once(pub, pos)
        


if __name__ == "__main__":
    rospy.init_node('sender', anonymous=True)

    rospy.loginfo("Node init")

    move()

    rospy.spin()
