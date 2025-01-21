#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(data):
		rospy.loginfo("I heard: %d", data.data)
   
def listener():
    rospy.init_node("listener_node", anonymous=True)
    rospy.Subscriber("my_msg", Int32, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass