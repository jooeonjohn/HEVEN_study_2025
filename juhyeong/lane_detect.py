#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from track_drive.msg import laneinfo


class LaneDetect:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('lane_detection_node', anonymous=False)

        # ROS Subscriber & Publisher
        rospy.Subscriber('/carla/ego_vehicle/rgb_view/image', Image, self.camera_callback, queue_size=1)
        self.pub = rospy.Publisher("lane_info", laneinfo, queue_size=1)

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        lane_info = self.process_image(img)
        self.pub.publish(lane_info)

    def warpping(self, image):
        source = np.float32([[235, 305], [590, 305], [60, 400], [800, 400]])
        destination = np.float32([[0, 0], [250, 0], [0, 460], [250, 460]])
        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        bird_image = cv2.warpPerspective(image, transform_matrix, (250, 460))
        return bird_image

    def color_filter(self, image):
        lower = np.array([200, 200, 200])
        upper = np.array([255, 255, 255])
        white_mask = cv2.inRange(image, lower, upper)
        masked = cv2.bitwise_and(image, image, mask=white_mask)
        return masked

    def plothistogram(self, image):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int_(histogram.shape[0]/2)
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint
        return leftbase, rightbase, histogram

    def slide_window_search(self, binary_warped, left_current, right_current):
        nwindows = 15
        window_height = np.int_(binary_warped.shape[0] / nwindows) 
        nonzero = binary_warped.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        margin = 30
        minpix = 10  
        left_lane = []
        right_lane = []

        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height
            win_y_high = binary_warped.shape[0] - w * window_height
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                        (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                        (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]

            if len(good_left) > minpix:
                left_lane.append(good_left)
                left_current = np.int_(np.mean(nonzero_x[good_left]))  

            if len(good_right) > minpix:
                right_lane.append(good_right)
                right_current = np.int_(np.mean(nonzero_x[good_right]))

        left_lane = np.concatenate(left_lane) if len(left_lane) > 0 else np.array([])
        right_lane = np.concatenate(right_lane) if len(right_lane) > 0 else np.array([])
        leftx = nonzero_x[left_lane] if len(left_lane) > 0 else np.array([])
        lefty = nonzero_y[left_lane] if len(left_lane) > 0 else np.array([])
        rightx = nonzero_x[right_lane] if len(right_lane) > 0 else np.array([])
        righty = nonzero_y[right_lane] if len(right_lane) > 0 else np.array([])

        if len(leftx) > 0 and len(lefty) > 0:
            left_fit = np.polyfit(lefty, leftx, 2)
        else:
            left_fit = [0, 0]

        if len(rightx) > 0 and len(righty) > 0:
            right_fit = np.polyfit(righty, rightx, 2)
        else:
            right_fit = [0, 0]

        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        for i in range(len(ploty)):
            cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)
            cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 1, (255, 255, 0), -1)

        return {'left_fitx': left_fitx, 'right_fitx': right_fitx, 'ploty': ploty}, out_img




    def process_image(self, img):
        # Step 1: BEV 변환
        warpped_img = self.warpping(img)

        # Step 2: Blurring을 통해 노이즈를 제거
        blurred_img = cv2.GaussianBlur(warpped_img, (7, 7), 1)

        # Step 3: 색상 필터링 및 이진화
        filtered_img = self.color_filter(blurred_img)
        gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(gray_img, 160, 255, cv2.THRESH_BINARY)

        # Step 4: 히스토그램
        left_base, right_base, hist = self.plothistogram(binary_img)
        # # 히스토그램 관찰용
        # hist_img = np.zeros((450, 260, 3), dtype=np.uint8) 
        # hist_norm = hist * (450.0 / hist.max())
        # for x, y in enumerate(hist_norm):
        #     cv2.line(hist_img, (x, 450), (x, 450 - int(y)), (0, 255, 0), 1)
        # cv2.imshow("Histogram", hist_img)

        # Step 5: 슬라이딩 윈도우
        draw_info, out_img = self.slide_window_search(binary_img, left_base, right_base)

        # Step 6: ROS 메시지 생성 및 발행
        pub_msg = laneinfo()

        # 왼쪽 차선 정보
        pub_msg.left_x = 130.0 - np.float32(draw_info['left_fitx'][-1])  
        pub_msg.left_y = np.float32(draw_info['ploty'][-1])  
        slope_left = 2 * draw_info['left_fitx'][0] * pub_msg.left_y + draw_info['left_fitx'][1]  # 기울기
        pub_msg.left_slope = np.float32(np.arctan(slope_left))  # 라디안 변환

        # 오른쪽 차선 정보
        pub_msg.right_x = np.float32(draw_info['right_fitx'][-1]) - 130.0
        pub_msg.right_y = np.float32(draw_info['ploty'][-1])  
        slope_right = 2 * draw_info['right_fitx'][0] * pub_msg.right_y + draw_info['right_fitx'][1]  # 기울기
        pub_msg.right_slope = np.float32(np.arctan(slope_right))  # 라디안 변환

        # 디버깅용
        cv2.imshow("raw_img",img)
        # cv2.imshow("bird_img",warpped_img)
        # cv2.imshow('blur_img', blurred_img)
        cv2.imshow("filter_img",filtered_img)
        # cv2.imshow("gray_img",gray_img)
        # cv2.imshow("binary_img",binary_img)
        cv2.imshow("result_img", out_img)
        cv2.waitKey(1)
        return pub_msg

if __name__ == "__main__":
    LaneDetect()
    rospy.spin()
