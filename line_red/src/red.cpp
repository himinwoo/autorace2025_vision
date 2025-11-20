#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class RedDetector {
public:
    RedDetector() {
        // ROS 초기화 및 토픽 설정
        ros::NodeHandle nh;
        cam_sub = nh.subscribe("/usb_cam/image_rect_color", 10, &RedDetector::camCallback, this);
        red_pub = nh.advertise<coss_msgs::Coss>("/cam/red_detection", 10);
    }

    // 카메라 콜백 함수
    void camCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img = cv_ptr->image;
            detectRed();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not decode image: %s", e.what());
        }
    }

    // 빨간색 감지 함수
    void detectRed() {
        if (img.empty()) return;

        cv::Mat img_hsv, red_mask1, red_mask2, combined_red_mask;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        // 빨간색 HSV 범위 설정
        cv::inRange(img_hsv, cv::Scalar(0, 120, 40), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(img_hsv, cv::Scalar(170, 120, 40), cv::Scalar(180, 255, 255), red_mask2);

        // 두 개의 마스크 합치기
        combined_red_mask = red_mask1 | red_mask2;

        // ROI 설정 (이미지 하단 50%만 사용)
        int roi_start_row = img.rows / 2;
        cv::Mat roi = combined_red_mask(cv::Range(roi_start_row, img.rows), cv::Range::all());

        int red_pixel_count = cv::countNonZero(roi);
        ROS_INFO("Red pixel count in ROI: %d", red_pixel_count);

        // 빨간색 픽셀 감지 여부에 따라 상태 업데이트
        coss_msgs::Coss red_msg;
        if (red_pixel_count > 28000) { // 이 값은 상황에 맞게 조정
            ROS_INFO("Red color detected in ROI");
            red_detected = true;
        } else {
            red_detected = false;
        }

        // 퍼블리시
        red_msg.cam_red_detection = red_detected;
        //red_msg.state1 = red_detected;
        red_pub.publish(red_msg);

        // 디버깅을 위해 이미지 시각화 (옵션)
        cv::imshow("Red Detection ROI", roi);
        cv::waitKey(1);
    }

private:
    ros::Subscriber cam_sub;
    ros::Publisher red_pub;
    cv::Mat img;
    bool red_detected = false; // 상태 저장 변수
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_detector");

    RedDetector red_detector;
    ros::spin();

    return 0;
}
