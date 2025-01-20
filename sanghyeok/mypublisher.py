#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def talker():
	pub = rospy.Publisher('my_msg', Int32, queue_size=10)
	rospy.init_node('talker_node', anonymous=True)
	rate = rospy.Rate(10)
	counter = 0
	
	while not rospy.is_shutdown():
		counter += 1
		pub.publish(counter)
		rospy.loginfo("==Pusblishing: %d",counter)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass