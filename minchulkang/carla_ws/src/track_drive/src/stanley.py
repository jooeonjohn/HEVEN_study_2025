#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped
from track_drive.msg import laneinfo
import numpy as np

class stanley:
    def __init__(self):
        self.lane_width = 2

        rospy.init_node("stanley", anonymous=False)
        self.pub = rospy.Publisher("/mobile_system_control/control_msg", Vector3Stamped, queue_size=1)
        rospy.Subscriber("/lane_info", laneinfo, self.callback, queue_size=1)
        rospy.wait_for_message("/lane_info", laneinfo)  # Wait until a message is received
        rate = rospy.Rate(10) #빈도 지정
        while not rospy.is_shutdown():
            self.stanley_control()
            rate.sleep()

    def callback(self, data): #4개가 중요한 변수
        self.left_x = data.left_x
        self.left_slope = data.left_slope
        self.right_x = data.right_x
        self.right_slope = data.right_slope

    def stanley_control(self):
        try:
            if self.left_x == 130:  # 차선이 하나만 감지될 때
                lateral_err = (0.5 - (self.right_x / 260)) * self.lane_width
                heading_err = -np.pi / 2 + self.right_slope
            elif self.right_x == -130:
                lateral_err = (-0.5 + (self.left_x / 260)) * self.lane_width
                heading_err = -np.pi / 2 + self.left_slope
            elif abs(self.left_x) > 130 or abs(self.right_x) > 130:  # 차선 정보가 불안정할 때
                velocity_profile *= 0.5  # 속도를 50%로 감소
            else:  # 일반적인 주행
                lateral_err = ((self.left_x / (self.left_x + self.right_x)) - 0.5) * self.lane_width
                heading_err = -np.pi / 2 + ((self.left_slope + self.right_slope) / 2)

            k = 1  # 스탠리 상수
            max_velocity = 20  # 최대 속도 (km/h)
            min_velocity = 10  # 최소 속도 (km/h)

            # 차선 기울기 차이를 계산하여 속도를 조정
            slope_diff = abs(self.left_slope - self.right_slope)
            velocity_profile = max_velocity - (slope_diff * 10)  # 차선 기울기 차이가 클수록 속도 감소
            velocity_profile = max(min_velocity, min(max_velocity, velocity_profile))  # 속도 제한 적용

            # 스티어링 계산
            steer = heading_err + np.arctan2(k * lateral_err, (velocity_profile / 3.6))

            # 📌 조향각(steer)에 따라 속도 조정 (절댓값이 클수록 속도 감소)
            steer_abs = abs(np.degrees(steer))  # 스티어링을 도(degree) 단위로 변환
            if steer_abs > 15:  # 조향이 15도 이상이면 속도를 30%로 감소
                velocity_profile *= 0.3  # 속도 감소
            elif steer_abs > 10:  # 조향이 10도 이상이면 속도를 50%로 감소
                velocity_profile *= 0.5
            elif steer_abs > 5:  # 조향이 5도 이상이면 속도를 70%로 감소
                velocity_profile *= 0.7

            # 최종 속도 제한
            velocity_profile = max(min_velocity, min(max_velocity, velocity_profile))

            cmd_vel = Vector3Stamped()
            cmd_vel.vector.x = velocity_profile / max_velocity  # 속도를 0~1 범위로 정규화하여 적용
            cmd_vel.vector.y = -np.degrees(steer) * 1 / 28  # 조향 값 적용

            self.pub.publish(cmd_vel)

            rospy.loginfo(
                f'\nlateral error: {lateral_err}\nheading error: {heading_err}\nsteer: {steer}\npubsteer: {cmd_vel.vector.y}\nvelocity: {velocity_profile}')
        except ZeroDivisionError as e:
            rospy.loginfo(e)

if __name__ == "__main__":
    stanley()
