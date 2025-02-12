#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32  # Import Int32 instead of String

def talker():
    pub = rospy.Publisher('my_msg', Int32, queue_size=10)  # Use Int32 publisher
    rospy.init_node('talker_node', anonymous=True)
    rate = rospy.Rate(10)
    
    count = 1  # Initialize the counter variable
    
    while not rospy.is_shutdown():
        pub.publish(count)  # Publish the integer count
        rospy.loginfo("==Publishing: %d", count)
        count += 1  # Increment the counter
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass