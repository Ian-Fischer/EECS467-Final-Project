///////////////////////////////////////////////////////////////////////
// This source was automatically generated by the lcm_to_ros package
// https://github.com/nrjl/lcm_to_ros, nicholas.lawrance@oregonstate.edu
///////////////////////////////////////////////////////////////////////
//
// Source message:    example_type.msg
// Creation:          Tue 21 Mar 2023 02:14:09 PM EDT
//
///////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include <lcm/lcm-cpp.hpp>

#include "lcm_to_ros/example_type.h"
#include "exlcm_rehash/example_type.hpp"

class ROSToLCMRepublisher
{
    public:
        ROSToLCMRepublisher(ros::NodeHandle& n, lcm::LCM* lh);
        ~ROSToLCMRepublisher();
        void rosCallback(const lcm_to_ros::example_type::ConstPtr& msg);
        
    private:        
        lcm::LCM* lcm_handle;
        ros::NodeHandle nh;
        ros::Subscriber ros_sub;   
};

ROSToLCMRepublisher::ROSToLCMRepublisher(ros::NodeHandle& n, lcm::LCM* lh) 
{
    nh = n;
    lcm_handle = lh;
   
    // Subscribers
    ros_sub = nh.subscribe<lcm_to_ros::example_type>("other_topic", 10, &ROSToLCMRepublisher::rosCallback, this);
};

ROSToLCMRepublisher::~ROSToLCMRepublisher() 
{    
    ROS_INFO("other_topic ROSToLCMRepublisher destructor.");
}

void ROSToLCMRepublisher::rosCallback(const lcm_to_ros::example_type::ConstPtr& ros_msg)
{
    const exlcm_rehash::example_type* lcm_msg = reinterpret_cast<const exlcm_rehash::example_type*>( ros_msg.get() );
    lcm_handle->publish("other_topic", lcm_msg);
};
        

int main(int argc, char** argv)
{
    lcm::LCM* lcm_handle = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
    if(!lcm_handle->good())
        return 1;

    ros::init(argc, argv, "other_topic_ros2lcm");
    ros::NodeHandle nh;   

    ROSToLCMRepublisher handlerObject(nh, lcm_handle);
    
    ros::spin();

    delete lcm_handle;
    return 0;
}

