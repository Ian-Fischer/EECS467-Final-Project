#include <iostream>
#include "ros/init.h"
#include "ros/publisher.h"
#include "slosbot.h"
#include "lcm_to_ros/mbot_motor_command_t.h"

SLOSBot::SLOSBot() :
 motor_command_pub( nh.advertise<lcm_to_ros::mbot_motor_command_t>("lcm_to_ros/MBOT_MOTOR_COMMAND", 1)),
 depth_img_sub( nh.subscribe("camera/depth_registered/image_raw", 1, &SLOSBot::depth_img_cb, this) ),
 rgb_img_sub( nh.subscribe("camera/rgb/image_rect_color", 1, &SLOSBot::rgb_img_cb, this) ) 
{} 

void SLOSBot::depth_img_cb(sensor_msgs::ImageConstPtr img) {}
void SLOSBot::rgb_img_cb(sensor_msgs::ImageConstPtr img) {
    std::cout << "recieved im" << std::endl;
}

void SLOSBot::search_for_object() {
    // get image

    // hsv segmentation

    // tell the mbot to rotate
    lcm_to_ros::mbot_motor_command_t msg;
    msg.angular_v = 0.3;
    motor_command_pub.publish(msg);

}

void SLOSBot::execute_sm() {
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        switch(state) {
            case State::SEARCH_FOR_OBJECT:
                search_for_object();
                break;
            default:
                std::cout << "invalid state" << std::endl;
        };

        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slosbot");
    std::cout << "hi " << std::endl;
    SLOSBot bot;
    bot.execute_sm();
}
