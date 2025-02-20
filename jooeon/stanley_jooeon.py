#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped
from track_drive.msg import laneinfo
import numpy as np

class stanley :
    def __init__(self) :
        self.lane_width = 3
        self.max_steer = np.radians(28)

        rospy.init_node("stanley", anonymous=False)
        self.pub = rospy.Publisher("/mobile_system_control/control_msg", Vector3Stamped, queue_size=1)
        rospy.Subscriber("/lane_info", laneinfo, self.callback, queue_size=1)
        rospy.wait_for_message("/lane_info", laneinfo)  # Wait until a message is received
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() :
            self.stanley_control()
            rate.sleep()

    def callback(self, data) :
        self.left_x = data.left_x
        self.left_slope = data.left_slope
        self.right_x = data.right_x
        self.right_slope = data.right_slope

    def stanley_control(self) :
        try:
            if self.left_x == 130 and self.right_x == -130:
                return
            elif self.left_x == 130: # 차선이 하나만 잡히는 경우 
                lateral_err = (0.5 - (self.right_x/220))*self.lane_width
                heading_err = self.right_slope
            elif self.right_x == -130:
                lateral_err = (-0.5 + (self.left_x/220))*self.lane_width
                heading_err = self.left_slope
            else: # 일반적인 주행
                lateral_err = ((self.left_x/(self.left_x + self.right_x)) - 0.5)*self.lane_width 
                heading_err = (self.left_slope + self.right_slope)/2

            k = 1 # stanley_상수
            velocity_profile = 20 # 속도값 km/h
                    
            steer = heading_err + np.arctan2(k*lateral_err,((velocity_profile/3.6)))

            steer = max(-self.max_steer,min(self.max_steer,steer)) * 0.7 #scaling

            throttle = 0.6
            
            
            cmd_vel = Vector3Stamped()
            cmd_vel.vector.x = throttle
            cmd_vel.vector.y = -np.degrees(steer)*1/28
            self.pub.publish(cmd_vel)
            
            rospy.loginfo(f'\nlateral error : {lateral_err}\nheading error : {heading_err}\nsteer : {steer}\npubsteer : {cmd_vel.vector.y}')
        except ZeroDivisionError as e:
            rospy.loginfo(e)

if __name__ == "__main__" :
    stanley()