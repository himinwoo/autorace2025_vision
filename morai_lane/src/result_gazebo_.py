    #!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import pi

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("image_processor_node", anonymous=True)
        # 카메라 토픽 구독 설정
        rospy.Subscriber("/usb_cam/image_rect_color", Image, self.cam_CB)
        self.pub = rospy.Publisher("/kuurack_left", AckermannDriveStamped, queue_size=10)

        self.original_window = "original_image"
        self.cropped_window = "croped_image"
        self.control_window = "control"

        self.create_trackbar_flag = False
        self.cam_flag = False
        self.window_num = 12
        self.margin = 30
        self.previous_point = 0
        self.img_msg = None
        self.cmd_msg = AckermannDriveStamped()
        
        self.L_H_Value = 20
        self.L_S_Value = 100
        self.L_V_Value = 100
        self.U_H_Value = 30
        self.U_S_Value = 255
        self.U_V_Value = 255
        self.L_R_Value = 0
        self.Stop_or_Go = 0
        self.Speed_Value = 1.0  # 속도 값 (m/s)
        self.Steering_Standard = 0

    def create_trackbar_init(self, cv_img):
        cv2.namedWindow(self.original_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.cropped_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.control_window)

        def hsv_track(value):
            self.L_H_Value = cv2.getTrackbarPos("Low_H", self.cropped_window)
            self.L_S_Value = cv2.getTrackbarPos("Low_S", self.cropped_window)
            self.L_V_Value = cv2.getTrackbarPos("Low_V", self.cropped_window)
            self.U_H_Value = cv2.getTrackbarPos("Up_H", self.cropped_window)
            self.U_S_Value = cv2.getTrackbarPos("Up_S", self.cropped_window)
            self.U_V_Value = cv2.getTrackbarPos("Up_V", self.cropped_window)
            self.L_R_Value = cv2.getTrackbarPos("Left/Right view", self.cropped_window)

            if self.L_R_Value != 1:
                init_left_standard = self.init_standard // 2
                cv2.setTrackbarPos("Steering Standard", self.control_window, init_left_standard)
            else:
                init_right_standard = self.init_standard + self.init_standard // 2
                cv2.setTrackbarPos("Steering Standard", self.control_window, init_right_standard)

        def control_track(value):
            self.Stop_or_Go = cv2.getTrackbarPos("Stop/Go", self.control_window)
            self.Speed_Value = cv2.getTrackbarPos("Speed", self.control_window)
            self.Steering_Standard = cv2.getTrackbarPos("Steering Standard", self.control_window)

        cv2.createTrackbar("Low_H", self.cropped_window, 0, 179, hsv_track)
        cv2.createTrackbar("Low_S", self.cropped_window, 0, 255, hsv_track)
        cv2.createTrackbar("Low_V", self.cropped_window, 0, 255, hsv_track)
        cv2.createTrackbar("Up_H", self.cropped_window, 179, 179, hsv_track)
        cv2.createTrackbar("Up_S", self.cropped_window, 255, 255, hsv_track)
        cv2.createTrackbar("Up_V", self.cropped_window, 255, 255, hsv_track)
        cv2.createTrackbar("Left/Right view", self.cropped_window, 0, 1, hsv_track)
        cv2.createTrackbar("Stop/Go", self.control_window, 0, 1, control_track)
        cv2.createTrackbar("Speed", self.control_window, 0, 30, control_track)
        cv2.createTrackbar("Steering Standard", self.control_window, self.init_standard, cv_img.shape[1] // 2, control_track)
        self.create_trackbar_flag = True

    # birds_eyeview 함수에서 src_points와 dst_points 좌표를 수정하여 이미지가 뒤집히지 않도록 수정
    def birds_eyeview(self, cv_img):
        src_points = np.float32([[90, 361-35], [0, 361], [640, 361], [640 - 90, 361 - 35]])
        dst_points = np.float32([[0, cv_img.shape[0]], [0, 0], [cv_img.shape[1], 0], [cv_img.shape[1], cv_img.shape[0]]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_img = cv2.warpPerspective(cv_img, matrix, (cv_img.shape[1], cv_img.shape[0]))
        return warped_img

    def apply_mask(self, warped_img):
        cvt_hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(cvt_hsv, (5, 5), 0)
        lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])
        mask = cv2.inRange(blur, lower, upper)
        hsv_img = cv2.bitwise_and(warped_img, warped_img, mask=mask)
        return hsv_img

    def crop_img(self, hsv_img):
        if self.L_R_Value != 1:
            croped_img = hsv_img[hsv_img.shape[0] * 2 // 4 : hsv_img.shape[0], 0 : hsv_img.shape[1] // 2]
        else:
            croped_img = hsv_img[hsv_img.shape[0] * 2 // 4 : hsv_img.shape[0], hsv_img.shape[1] // 2 : hsv_img.shape[1]]
        croped_img_shape = croped_img.shape[0:2]
        return croped_img, croped_img_shape

    def binary(self, croped_img):
        bin = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)
        binary_img = np.zeros_like(bin)
        binary_img[bin != 0] = 255
        cv2.imshow("bin", binary_img)
        return binary_img

    def sliding_window(self, warped_img, croped_img, binary_img, croped_img_shape):
        crop_layer = croped_img_shape[0] // self.window_num
        threshold = crop_layer // 2
        histogram = []
        indices = []
        middle_point = []

        for i in range(0, self.window_num):
            original_window_top = warped_img.shape[0] - (i + 1) * crop_layer
            original_window_bottom = warped_img.shape[0] - (i) * crop_layer
            cropped_window_top = croped_img_shape[0] - (i + 1) * crop_layer
            cropped_window_bottom = croped_img_shape[0] - (i) * crop_layer

            histogram.append(np.sum(binary_img[cropped_window_top:cropped_window_bottom, :], axis=0))
            indices.append(np.where(histogram[i] > threshold)[0])

            try:
                middle_point.append((min(indices[i]) + max(indices[i])) // 2)
                cropped_window_left = middle_point[i] - self.margin
                cropped_window_right = middle_point[i] + self.margin

                cv2.rectangle(croped_img, (cropped_window_left, cropped_window_top), (cropped_window_right, cropped_window_bottom), (0, 0, 255), 2)
                cv2.circle(croped_img, (middle_point[i], (cropped_window_top + cropped_window_bottom) // 2), 3, (0, 255, 0), -1)

                if self.L_R_Value != 1:
                    original_window_left = cropped_window_left
                    original_window_right = cropped_window_right
                    circle_point = middle_point[i]
                else:
                    original_window_left = cropped_window_left + warped_img.shape[1] // 2
                    original_window_right = cropped_window_right + warped_img.shape[1] // 2
                    circle_point = middle_point[i] + warped_img.shape[1] // 2

                cv2.rectangle(warped_img, (original_window_left, original_window_top), (original_window_right, original_window_bottom), (0, 0, 255), 2)
                cv2.circle(warped_img, (circle_point, (original_window_top + original_window_bottom) // 2), 3, (0, 255, 0), -1)
            except:
                print(f"line_not_detected: {i}")

        try:
            weight_point = np.zeros_like(middle_point)
            for index, point in enumerate(middle_point):
                weight_point[index] = middle_point[0] - (middle_point[0] - point) * (len(middle_point) - index) / len(middle_point)
            avg_point = int(np.average(weight_point))
            self.previous_point = avg_point
        except:
            avg_point = self.previous_point

        if self.L_R_Value != 1:
            steering_standard = self.Steering_Standard
        else:
            steering_standard = croped_img_shape[1] - self.Steering_Standard

        cv2.rectangle(
            warped_img,
            (steering_standard, warped_img.shape[0] - croped_img_shape[0]),
            (warped_img.shape[1] - steering_standard, warped_img.shape[0]),
            (255, 0, 0),
            5,
        )
        cv2.line(croped_img, (self.Steering_Standard, 0), (self.Steering_Standard, croped_img_shape[0]), (255, 0, 0), 3)
        str_standard_point = "standard x: " + str(self.Steering_Standard)
        str_avg_point = "avg point: " + str(avg_point)

        cv2.putText(warped_img, str_standard_point, (warped_img.shape[1] // 8, warped_img.shape[0] // 8), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(warped_img, str_avg_point, (warped_img.shape[1] // 8, warped_img.shape[0] // 4), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return avg_point

    def cal_angle_per_pixel(self, avg_point, cv_img):
        angle_resolution = pi / cv_img.shape[1]
        return angle_resolution

    def sense(self):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return None

        if not self.create_trackbar_flag:
            self.init_standard = cv_img.shape[1] // 4
            self.create_trackbar_init(cv_img)
        
        cv2.imshow("raw", cv_img)

        return cv_img

    def think(self, cv_img):
        warped_img = self.birds_eyeview(cv_img)
        hsv_img = self.apply_mask(warped_img)
        croped_img, croped_img_shape = self.crop_img(hsv_img)
        binary_img = self.binary(croped_img)

        avg_point = self.sliding_window(warped_img, croped_img, binary_img, croped_img_shape)
        angle_resolution = self.cal_angle_per_pixel(avg_point, cv_img)

        return warped_img, croped_img, avg_point, angle_resolution

    def act(self, cv_img, avg_point, angle_resolution):
        if self.Stop_or_Go == 0:
            #self.cmd_msg.drive.speed = 0
            self.cmd_msg.drive.steering_angle = 0
        else:
            #self.cmd_msg.drive.speed = self.Speed_Value / 10
            self.cmd_msg.drive.steering_angle = (0.66) * -angle_resolution * (avg_point - self.Steering_Standard) #조향 -면 오른쪽
            
        rospy.loginfo(f"Angle Resolution: {angle_resolution}")
        rospy.loginfo(f"Avg Point - Steering Standard: {avg_point - self.Steering_Standard}")
        rospy.loginfo(f"최종: {self.cmd_msg.drive.steering_angle}")

        str_speed = "speed: " + str(self.cmd_msg.drive.speed)
        str_steering = "steer: " + str(round(self.cmd_msg.drive.steering_angle, 2))
        cv2.putText(cv_img, str_speed, (cv_img.shape[1] // 2 + cv_img.shape[1] // 8, cv_img.shape[0] // 8), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(cv_img, str_steering, (cv_img.shape[1] // 2 + cv_img.shape[1] // 8, cv_img.shape[0] // 4), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        self.pub.publish(self.cmd_msg)

    def cam_CB(self, msg):
        self.img_msg = msg
        self.cam_flag = True

    def run(self):
        if self.cam_flag and self.img_msg is not None:
            cv_img = self.sense()

            if cv_img is not None and self.create_trackbar_flag:
                warped_img, croped_img, avg_point, angle_resolution = self.think(cv_img)
                self.act(cv_img, avg_point, angle_resolution)

                cv2.imshow(self.original_window, warped_img)
                cv2.imshow(self.cropped_window, croped_img)
                cv2.waitKey(1)

def main():
    try:
        image_processor = ImageProcessor()
        while not rospy.is_shutdown():
            image_processor.run()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()