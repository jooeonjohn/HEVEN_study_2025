#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped
from track_drive.msg import laneinfo
import numpy as np

class stanley:
    def __init__(self):
        self.lane_width = 2  # 차선 너비
        self.max_speed = 15  # 최대 속도 (km/h)
        
        rospy.init_node("stanley", anonymous=False)
        self.pub = rospy.Publisher("/mobile_system_control/control_msg", Vector3Stamped, queue_size=1)
        rospy.Subscriber("/lane_info", laneinfo, self.callback, queue_size=1)
        rospy.wait_for_message("/lane_info", laneinfo)  # 메시지가 올 때까지 대기
        rate = rospy.Rate(10)  # 10 Hz 주기로 실행
        while not rospy.is_shutdown():
            self.stanley_control()
            rate.sleep()

    def callback(self, data):
        self.left_x = data.left_x
        self.left_slope = data.left_slope
        self.right_x = data.right_x
        self.right_slope = data.right_slope

    def stanley_control(self):
        try:
            # 차선이 하나만 인식되는 경우
            if self.left_x == 130:  # 왼쪽 차선만 감지
                lateral_err = (0.5 - (self.right_x / 260)) * self.lane_width
                heading_err = - np.pi / 2 + self.right_slope
            elif self.right_x == -130:  # 오른쪽 차선만 감지
                lateral_err = (-0.5 + (self.left_x / 260)) * self.lane_width
                heading_err = - np.pi / 2 + self.left_slope
            else:  # 두 차선 모두 인식된 경우
                lateral_err = ((self.left_x / (self.left_x + self.right_x)) - 0.5) * self.lane_width
                heading_err = - np.pi / 2 + ((self.left_slope + self.right_slope) / 2)

            # 커브 반경 계산 (오른쪽 차선 기울기 사용)
            curve_radius = np.abs(1 / self.right_slope) if self.right_slope != 0 else np.inf  # 기울기가 0일 경우 무한대로 설정

            # 속도 조정: 커브 반경에 따라 속도를 동적으로 조정
            if curve_radius < 10:  # 급커브일 경우 속도 제한
                velocity_profile = 5  # 5 km/h
            elif curve_radius < 50:  # 중간 정도의 커브
                velocity_profile = 10  # 10 km/h
            else:  # 완만한 커브
                velocity_profile = self.max_speed  # 최대 속도 설정

            # 제어 상수 k 값을 커브 반경에 따라 동적으로 조정
            if curve_radius > 100:  # 커브 반경이 매우 클 경우 (직선에 가까운 경우)
                k = 0.8  
                velocity_profile = self.max_speed  # 직선 구간에서는 최대 속도
            
            else:  # 커브 구간에서는 속도와 k를 동적으로 설정
                if curve_radius < 10:  # 급커브일 경우 속도 제한
                    velocity_profile = 5  # 5 km/h
                    k = 0.3  # 급커브일 경우 더 작은 k 값
                elif curve_radius < 50:  # 중간 정도의 커브
                    velocity_profile = 10  # 10 km/h
                    k = 0.5  # 중간 정도의 커브
                else:  # 완만한 커브일 경우
                    velocity_profile = self.max_speed  # 최대 속도
                    k = 0.8  # 완만한 커브일 경우 더 큰 k 값

            # 스티어링 각도 계산
            steer = heading_err + np.arctan2(k * lateral_err, (velocity_profile / 3.6))

            # 제어 메시지 생성 및 발행
            cmd_vel = Vector3Stamped()
            cmd_vel.vector.x = 1.0  # 이동 방향 (앞으로)
            cmd_vel.vector.y = -np.degrees(steer) * 1 / 28  # 조향 각도 계산 (스티어링 각도를 라디안에서 도로 변환)
            self.pub.publish(cmd_vel)

            rospy.loginfo(f'\nLateral error: {lateral_err}\nHeading error: {heading_err}\nSteer: {steer}\nPublished steer: {cmd_vel.vector.y}')
        
        except ZeroDivisionError as e:
            rospy.loginfo(f'Error: {e}')

if __name__ == "__main__":
    stanley()
