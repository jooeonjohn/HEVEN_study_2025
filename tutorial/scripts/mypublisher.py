#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def publisher():
    pub = rospy.Publisher('hello_world', Int32, queue_size=10)
    rospy.init_node('mypublisher', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz (1초마다 실행)
    count = 1  # 1부터 시작

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing: {count}")
        pub.publish(count)
        count += 1  # 1씩 증가
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
