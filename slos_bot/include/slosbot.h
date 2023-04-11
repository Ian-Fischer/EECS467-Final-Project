#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "lcm_to_ros/mbot_motor_command_t.h"
#include "lcm_to_ros/odometry_t.h"
#include "detection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <chrono>

//#define DEBUG

class SLOSBot 
{
    public:
        SLOSBot();

        enum class State {
            SEARCH_FOR_OBJECT,
            MATCH_OBJECT, 
            DRIVE_TO_OBJECT,
            SEARCH_FOR_ZONE,
            DRIVE_TO_ZONE,
            REVERSE
        };

        void execute_sm();
    private:
        // Robot state
        State state = State::SEARCH_FOR_OBJECT;
        State prev_state;
        bool rising_edge = true;
        DetectionManager object_detection;
        DetectionManager april_detection;
        cv::Mat cur_rgb;
        cv::Mat cur_depth;
        lcm_to_ros::odometry_t cur_odom;
        bool april_detected = false;
        std::chrono::time_point<std::chrono::system_clock> reverse_time;
        int desired_tag_id = 0;

        // Ros stuff
        ros::NodeHandle nh;
        ros::Publisher motor_command_pub; 
        ros::Subscriber odom_sub;
        ros::Subscriber depth_img_sub;
        ros::Subscriber rgb_img_sub;
        ros::Subscriber april_sub;
        ros::ServiceClient claw_client;

        // State functions
        SLOSBot::State search_for_object();
        SLOSBot::State drive_to_object();
        SLOSBot::State search_for_zone();
        SLOSBot::State drive_to_zone();
        SLOSBot::State reverse(bool);

        // Shared state functionality and helpers
        bool run_obj_detection(bool blue=false);
        bool run_drive_ctrl(DetectionManager &detection, float error=0.04);

        // Callbacks
        void odom_cb(lcm_to_ros::odometry_t);
        void depth_img_cb(sensor_msgs::ImageConstPtr);
        void rgb_img_cb(sensor_msgs::ImageConstPtr);
        void april_cb(apriltag_ros::AprilTagDetectionArray);
};
