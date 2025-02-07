#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(f"I heard, {data.data}")

def subscriber():
    rospy.init_node('mysubscriber', anonymous=True)
    rospy.Subscriber('hello_world', Int32, callback)
    rospy.spin()  # 노드가 종료되지 않도록 유지

if __name__ == '__main__':
    subscriber()
