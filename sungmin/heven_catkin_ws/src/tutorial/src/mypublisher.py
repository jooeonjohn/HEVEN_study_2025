#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('my_msg', String, queue_size=10)
    rospy.init_node('talker_node', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    
    counter = 1  # 숫자 카운터 초기값
    while not rospy.is_shutdown():
        msg = str(counter)  # 발행할 메시지는 숫자
        pub.publish(msg)
        rospy.loginfo("==Publishing...")  # 여기서는 숫자를 포함하지 않음
        counter += 1  # 숫자 증가
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
