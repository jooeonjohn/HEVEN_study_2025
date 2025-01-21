#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32  # Import Int32 instead of String

def callback(data):
    rospy.loginfo("I heard, %d", data.data)  # Print the received integer
    
def listener():
    sub = rospy.Subscriber("my_msg", Int32, callback)  # Subscribe to my_msg topic with Int32 type
    rospy.init_node('listener_node', anonymous=True)
    rospy.spin()  # Keeps the program running and listening for messages

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
