#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

class SLOSBot 
{
    public:
        SLOSBot();

        void execute_sm();
    private:
        enum class State {
            SEARCH_FOR_OBJECT,
            MATCH_OBJECT
        };
        State state = State::SEARCH_FOR_OBJECT;

        ros::NodeHandle nh;
        ros::Publisher motor_command_pub; 

        ros::Subscriber depth_img_sub;
        ros::Subscriber rgb_img_sub;

        cv::Mat cur_rgb;
        cv::Mat cur_depth;

        void search_for_object();


        void depth_img_cb(sensor_msgs::ImageConstPtr img);
        void rgb_img_cb(sensor_msgs::ImageConstPtr img);
};
