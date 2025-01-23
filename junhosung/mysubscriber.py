#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(data):
		rospy.loginfo("I heard, %d" %data.data) #데이터값이 subscribe할 수 있게 된다.
   
def listener():
    sub=rospy.Subscriber("my_msg", Int32, callback)
    rospy.init_node('listener_node', anonymous=True)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass