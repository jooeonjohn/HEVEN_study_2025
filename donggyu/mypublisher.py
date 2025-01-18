#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

count = 1

def talker():
	global count
	pub = rospy.Publisher('my_msg', Int32, queue_size=10)
	rospy.init_node('talker_node', anonymous=True)
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		count += 1
		pub.publish(count)
		rospy.loginfo("==Pusblishing...")
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass