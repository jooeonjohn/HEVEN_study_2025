#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def talker():
	pub = rospy.Publisher('my_msg', Int32, queue_size=10)
	rospy.init_node('talker_node', anonymous=True)
	rate = rospy.Rate(10)
	
	count =1
	while not rospy.is_shutdown():
		pub.publish(count)
		rospy.loginfo(f"Pusblishing... {count}")
		count +=1
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass