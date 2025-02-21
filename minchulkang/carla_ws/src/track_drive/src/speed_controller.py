#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class SpeedController:
    def __init__(self):
        rospy.init_node('speed_controller', anonymous=False)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # 10Hz
        self.speed = 1.0  # 기본 속도 (m/s)
        self.max_speed = 2.0  # 최대 속도 (m/s)
        self.min_speed = 0.5  # 최소 속도 (m/s)

    def control_speed(self):
        while not rospy.is_shutdown():
            # 여기에 속도 제어 로직을 추가할 수 있습니다.
            # 예를 들어, 차선 정보를 받아서 속도를 조절하거나, 장애물 감지 시 속도를 줄이는 등의 로직을 추가할 수 있습니다.
            speed = self.speed  # 기본 속도 사용

            # Twist 메시지 생성 및 발행
            twist_msg = Twist()
            twist_msg.linear.x = speed
            twist_msg.angular.z = 0.0  # 조향은 스탠리 컨트롤러에서 처리하므로 0으로 설정
            self.pub.publish(twist_msg)

            self.rate.sleep()

if __name__ == "__main__":
    try:
        speed_controller = SpeedController()
        speed_controller.control_speed()
    except rospy.ROSInterruptException:
        pass