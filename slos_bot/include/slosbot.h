#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "lcm_to_ros/mbot_motor_command_t.h"
#include "lcm_to_ros/odometry_t.h"
#include "detection.h"

//#define DEBUG

class SLOSBot 
{
    public:
        SLOSBot();

        enum class State {
            SEARCH_FOR_OBJECT,
            MATCH_OBJECT, 
            DRIVE_TO_OBJECT,
        };

        void execute_sm();
    private:
        State state = State::SEARCH_FOR_OBJECT;

        DetectionManager object_detection;

        ros::NodeHandle nh;
        ros::Publisher motor_command_pub; 

        ros::Subscriber odom_sub;
        ros::Subscriber depth_img_sub;
        ros::Subscriber rgb_img_sub;

        cv::Mat cur_rgb;
        cv::Mat cur_depth;
        lcm_to_ros::odometry_t cur_odom;

        SLOSBot::State search_for_object();
        SLOSBot::State drive_to_object();

        // Runs detection pipeline and returns true if there was a detection
        bool run_obj_detection();

        void odom_cb(lcm_to_ros::odometry_t);
        void depth_img_cb(sensor_msgs::ImageConstPtr);
        void rgb_img_cb(sensor_msgs::ImageConstPtr);
};
