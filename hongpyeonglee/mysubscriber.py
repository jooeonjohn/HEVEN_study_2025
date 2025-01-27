#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo("I heard, %d" %data.data)

def listener():
    sub=rospy.Subscriber("my_msg", Int32, callback)
    rospy.init_node('listener_node', anonymous=True)
    rospy.spin()

if _name_ =="_main_":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass                   
