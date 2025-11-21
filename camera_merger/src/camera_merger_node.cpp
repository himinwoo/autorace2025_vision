#include <ros/ros.h>
#include <coss_msgs/Coss.h>
#include <mutex>

class CameraMerger
{
public:
    CameraMerger() : nh_("~")
    {
        // 통합 Coss 메시지 초기화
        merged_msg_.cam_steer = 0.0;
        merged_msg_.mission_state = 0;
        merged_msg_.cam_red_detection = false;
        merged_msg_.cam_blue_detection = false;

        // Publisher 설정
        merged_pub_ = nh_.advertise<coss_msgs::Coss>("/camera", 10);

        // Subscribers 설정
        // stopline_pkg에서 발행하는 토픽
        stopline_sub_ = nh_.subscribe("/camera/stopline/count", 10, 
                                      &CameraMerger::stoplineCallback, this);
        
        // onelane_detection에서 발행하는 토픽 (토픽명 확인 필요)
        lane_sub_ = nh_.subscribe("/camera/steering", 10, 
                                  &CameraMerger::laneCallback, this);
        
        // color_detector에서 발행하는 토픽 (토픽명 확인 필요)
        color_sub_ = nh_.subscribe("/camera/color_detection", 10, 
                                   &CameraMerger::colorCallback, this);

        // 파라미터 로드
        publish_rate_ = nh_.param("publish_rate", 20.0);

        ROS_INFO("===== Camera Merger Node Started =====");
        ROS_INFO("Subscribing to:");
        ROS_INFO("  - /camera/stopline/count");
        ROS_INFO("  - /camera/steering");
        ROS_INFO("  - /camera/color_detection");
        ROS_INFO("Publishing to: /camera");
        ROS_INFO("Publish rate: %.1f Hz", publish_rate_);

        // 타이머 설정 (주기적으로 통합 메시지 발행)
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                 &CameraMerger::timerCallback, this);
    }

private:
    // Stopline 패키지 콜백
    void stoplineCallback(const coss_msgs::Coss::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // mission_state 업데이트
        merged_msg_.mission_state = msg->mission_state;
        
        ROS_DEBUG("Stopline: mission_state=%d", msg->mission_state);
    }

    // Lane detection 패키지 콜백
    void laneCallback(const coss_msgs::Coss::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // cam_steer 업데이트 (차선 추종 조향각)
        merged_msg_.cam_steer = msg->cam_steer;

        ROS_DEBUG("Lane: cam_steer=%.3f", msg->cam_steer);
    }

    // Color detector 패키지 콜백
    void colorCallback(const coss_msgs::Coss::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 색상 감지 결과 업데이트
        merged_msg_.cam_red_detection = msg->cam_red_detection;
        merged_msg_.cam_blue_detection = msg->cam_blue_detection;
        
        ROS_DEBUG("Color: red=%d, blue=%d", 
                  msg->cam_red_detection, msg->cam_blue_detection);
    }

    // 주기적으로 통합 메시지 발행
    void timerCallback(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 통합 메시지 발행
        merged_pub_.publish(merged_msg_);
    }

    // ROS 핸들
    ros::NodeHandle nh_;
    
    // Publisher
    ros::Publisher merged_pub_;
    
    // Subscribers
    ros::Subscriber stopline_sub_;
    ros::Subscriber lane_sub_;
    ros::Subscriber color_sub_;
    
    // Timer
    ros::Timer timer_;
    
    // 통합 메시지
    coss_msgs::Coss merged_msg_;
    
    // Mutex for thread safety
    std::mutex mutex_;
    
    // Parameters
    double publish_rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_merger_node");
    
    CameraMerger merger;
    
    ros::spin();
    
    return 0;
}
