#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ColorDetector {
public:
    ColorDetector() {
        // ROS 초기화 및 토픽 설정
        ros::NodeHandle nh;
        cam_sub = nh.subscribe("/usb_cam/image_rect_color", 10, &ColorDetector::camCallback, this);
        
        // 빨강과 파랑 합친 토픽
        color_pub = nh.advertise<coss_msgs::Coss>("/camera/color_detection", 10);

        // 파라미터 로드
        nh.param("red_threshold", red_threshold, 28000);
        nh.param("blue_threshold", blue_threshold, 28000);
        nh.param("debug_mode", debug_mode, true);
        
        ROS_INFO("Color Detector Node initialized");
        ROS_INFO("Publishing to: /camera/color_detection");
        ROS_INFO("Red threshold: %d, Blue threshold: %d", red_threshold, blue_threshold);
    }

    // 카메라 콜백 함수
    void camCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img = cv_ptr->image;
            detectColors();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not decode image: %s", e.what());
        }
    }

    // 빨강과 파랑 동시 감지
    void detectColors() {
        if (img.empty()) return;

        cv::Mat img_hsv;
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        // ROI 설정 (이미지 하단 50%만 사용)
        int roi_start_row = img.rows / 2;

        // 메시지 초기화
        coss_msgs::Coss color_msg;
        color_msg.cam_red_detection = false;
        color_msg.cam_blue_detection = false;

        // ========== 빨간색 감지 ==========
        cv::Mat red_mask1, red_mask2, combined_red_mask;
        cv::inRange(img_hsv, cv::Scalar(0, 120, 40), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(img_hsv, cv::Scalar(170, 120, 40), cv::Scalar(180, 255, 255), red_mask2);
        combined_red_mask = red_mask1 | red_mask2;
        
        cv::Mat red_roi = combined_red_mask(cv::Range(roi_start_row, img.rows), cv::Range::all());
        int red_pixel_count = cv::countNonZero(red_roi);
        
        if (red_pixel_count > red_threshold) {
            ROS_INFO("Red detected: %d pixels", red_pixel_count);
            color_msg.cam_red_detection = true;
        }

        // ========== 파란색 감지 ==========
        cv::Mat blue_mask;
        cv::inRange(img_hsv, cv::Scalar(100, 100, 50), cv::Scalar(130, 255, 255), blue_mask);
        
        cv::Mat blue_roi = blue_mask(cv::Range(roi_start_row, img.rows), cv::Range::all());
        int blue_pixel_count = cv::countNonZero(blue_roi);
        
        if (blue_pixel_count > blue_threshold) {
            ROS_INFO("Blue detected: %d pixels", blue_pixel_count);
            color_msg.cam_blue_detection = true;
        }
        
        // 메시지 퍼블리시
        color_pub.publish(color_msg);

        // ========== 디버깅 ==========
        if (debug_mode) {
            cv::imshow("Red Detection ROI", red_roi);
            cv::imshow("Blue Detection ROI", blue_roi);
            cv::waitKey(1);
        }
    }

private:
    ros::Subscriber cam_sub;
    ros::Publisher color_pub;
    cv::Mat img;
    
    int red_threshold;
    int blue_threshold;
    bool debug_mode;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_detector_node");
    ColorDetector color_detector;
    ros::spin();
    return 0;
}
