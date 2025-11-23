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
        ros::NodeHandle pnh("~");  // private node handle for parameters
        cam_sub = nh.subscribe("/usb_cam/image_rect_color", 10, &ColorDetector::camCallback, this);
        
        // 빨강과 파랑 합친 토픽
        color_pub = nh.advertise<coss_msgs::Coss>("/camera/color_detection", 10);

        // 파라미터 로드
        pnh.param("red_threshold", red_threshold, 28000);
        pnh.param("blue_threshold", blue_threshold, 28000);
        pnh.param("debug_mode", debug_mode, true);
        
        // HSV 파라미터 로드
        std::vector<int> red_hsv_lower1_vec = {0, 120, 40};
        std::vector<int> red_hsv_upper1_vec = {10, 255, 255};
        std::vector<int> red_hsv_lower2_vec = {170, 120, 40};
        std::vector<int> red_hsv_upper2_vec = {180, 255, 255};
        std::vector<int> blue_hsv_lower_vec = {100, 100, 50};
        std::vector<int> blue_hsv_upper_vec = {130, 255, 255};
        
        pnh.getParam("red_hsv_lower1", red_hsv_lower1_vec);
        pnh.getParam("red_hsv_upper1", red_hsv_upper1_vec);
        pnh.getParam("red_hsv_lower2", red_hsv_lower2_vec);
        pnh.getParam("red_hsv_upper2", red_hsv_upper2_vec);
        pnh.getParam("blue_hsv_lower", blue_hsv_lower_vec);
        pnh.getParam("blue_hsv_upper", blue_hsv_upper_vec);
        
        red_hsv_lower1 = cv::Scalar(red_hsv_lower1_vec[0], red_hsv_lower1_vec[1], red_hsv_lower1_vec[2]);
        red_hsv_upper1 = cv::Scalar(red_hsv_upper1_vec[0], red_hsv_upper1_vec[1], red_hsv_upper1_vec[2]);
        red_hsv_lower2 = cv::Scalar(red_hsv_lower2_vec[0], red_hsv_lower2_vec[1], red_hsv_lower2_vec[2]);
        red_hsv_upper2 = cv::Scalar(red_hsv_upper2_vec[0], red_hsv_upper2_vec[1], red_hsv_upper2_vec[2]);
        blue_hsv_lower = cv::Scalar(blue_hsv_lower_vec[0], blue_hsv_lower_vec[1], blue_hsv_lower_vec[2]);
        blue_hsv_upper = cv::Scalar(blue_hsv_upper_vec[0], blue_hsv_upper_vec[1], blue_hsv_upper_vec[2]);
        
        ROS_INFO("Red HSV Range 1: [%d-%d, %d-%d, %d-%d]", red_hsv_lower1_vec[0], red_hsv_upper1_vec[0], 
                 red_hsv_lower1_vec[1], red_hsv_upper1_vec[1], red_hsv_lower1_vec[2], red_hsv_upper1_vec[2]);
        ROS_INFO("Red HSV Range 2: [%d-%d, %d-%d, %d-%d]", red_hsv_lower2_vec[0], red_hsv_upper2_vec[0],
                 red_hsv_lower2_vec[1], red_hsv_upper2_vec[1], red_hsv_lower2_vec[2], red_hsv_upper2_vec[2]);
        ROS_INFO("Blue HSV Range: [%d-%d, %d-%d, %d-%d]", blue_hsv_lower_vec[0], blue_hsv_upper_vec[0],
                 blue_hsv_lower_vec[1], blue_hsv_upper_vec[1], blue_hsv_lower_vec[2], blue_hsv_upper_vec[2]);
        
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

        // 가우시안 블러로 노이즈 제거
        cv::Mat img_blurred;
        cv::GaussianBlur(img, img_blurred, cv::Size(5, 5), 0);

        cv::Mat img_hsv;
        cv::cvtColor(img_blurred, img_hsv, cv::COLOR_BGR2HSV);

        // ROI 설정 (이미지 하단 50%만 사용)
        int roi_start_row = img.rows / 2;

        // 메시지 초기화
        coss_msgs::Coss color_msg;
        color_msg.cam_red_detection = false;
        color_msg.cam_blue_detection = false;

        // ========== 빨간색 감지 ==========
        cv::Mat red_mask1, red_mask2, combined_red_mask;
        cv::inRange(img_hsv, red_hsv_lower1, red_hsv_upper1, red_mask1);
        cv::inRange(img_hsv, red_hsv_lower2, red_hsv_upper2, red_mask2);
        combined_red_mask = red_mask1 | red_mask2;
        
        // 모폴로지 연산 (노이즈 제거 및 영역 보완)
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(combined_red_mask, combined_red_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(combined_red_mask, combined_red_mask, cv::MORPH_OPEN, kernel);
        
        cv::Mat red_roi = combined_red_mask(cv::Range(roi_start_row, img.rows), cv::Range::all());
        int red_pixel_count = cv::countNonZero(red_roi);
        
        if (red_pixel_count > red_threshold) {
            ROS_INFO("Red detected: %d pixels", red_pixel_count);
            color_msg.cam_red_detection = true;
        }

        // ========== 파란색 감지 ==========
        cv::Mat blue_mask;
        cv::inRange(img_hsv, blue_hsv_lower, blue_hsv_upper, blue_mask);
        
        // 모폴로지 연산 (노이즈 제거 및 영역 보완)
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);
        
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
    
    // HSV 범위
    cv::Scalar red_hsv_lower1, red_hsv_upper1;
    cv::Scalar red_hsv_lower2, red_hsv_upper2;
    cv::Scalar blue_hsv_lower, blue_hsv_upper;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_detector_node");
    ColorDetector color_detector;
    ros::spin();
    return 0;
}
