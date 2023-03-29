#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "lcm_to_ros/mbot_motor_command_t.h"
#include "lcm_to_ros/odometry_t.h"

//#define DEBUG

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

        ros::Subscriber odom_sub;
        ros::Subscriber depth_img_sub;
        ros::Subscriber rgb_img_sub;
        ros::Subscriber pointcloud_sub;

        cv::Mat cur_rgb;
        cv::Mat cur_depth;
        //sensor_msgs::PointCloud2 cur_pc;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_pc;

        pcl::visualization::PCLVisualizer::Ptr viewer;

        void search_for_object();

        void odom_cb(lcm_to_ros::odometry_t);
        void depth_img_cb(sensor_msgs::ImageConstPtr);
        void rgb_img_cb(sensor_msgs::ImageConstPtr);
        void pointcloud_cb(sensor_msgs::PointCloud2Ptr);
};
