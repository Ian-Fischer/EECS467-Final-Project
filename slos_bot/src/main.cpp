#include <algorithm>
#include <cmath>
#include <iostream>
#include "ros/init.h"
#include "ros/publisher.h"
#include "sensor_msgs/image_encodings.h"
#include "slosbot.h"
#include "cv_bridge/cv_bridge.h"
#include <math.h>   

using namespace cv;
using namespace std;

SLOSBot::SLOSBot() :
 motor_command_pub( nh.advertise<lcm_to_ros::mbot_motor_command_t>("lcm_to_ros/MBOT_MOTOR_COMMAND", 1)),
 depth_img_sub( nh.subscribe("camera/depth_registered/image_raw", 1, &SLOSBot::depth_img_cb, this) ),
 rgb_img_sub( nh.subscribe("camera/rgb/image_rect_color", 1, &SLOSBot::rgb_img_cb, this) ),
 odom_sub( nh.subscribe("lcm_to_ros/ODOMETRY", 1, &SLOSBot::odom_cb, this) ),
 april_sub( nh.subscribe("tag_detections", 1, &SLOSBot::april_cb, this) ) {

} 


void SLOSBot::april_cb(apriltag_ros::AprilTagDetectionArray a) {
    std::cout << a.detections.size() << std::endl;
}

void SLOSBot::odom_cb(lcm_to_ros::odometry_t odom) {
    cur_odom = odom;
}

void SLOSBot::depth_img_cb(sensor_msgs::ImageConstPtr img) {
    try {
        cv_bridge::CvImagePtr im;
        im = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
        cur_depth = im->image;
    } catch(cv_bridge::Exception) {
        std::cout << "Could not convert image" << std::endl;
    }

}

void SLOSBot::rgb_img_cb(sensor_msgs::ImageConstPtr img) {
    //std::cout << "recieved im" << std::endl;
    try {
        cv_bridge::CvImagePtr im;
        im = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        cur_rgb = im->image;
    } catch(cv_bridge::Exception) {
        std::cout << "Could not convert image" << std::endl;
    }
}


bool SLOSBot::run_obj_detection() {
    // Make sure there is a current image
    if(cur_rgb.rows == 0 || cur_rgb.cols == 0 || cur_depth.rows == 0) return false;
    Mat debug_img = cur_rgb.clone();

    // HSV segmentation
    Mat hsv_img;
    cvtColor(cur_rgb, hsv_img, COLOR_BGR2HSV);

    Mat bin_img(hsv_img.size(), 0); 
    //inRange(hsv_img, Scalar(100, 120, 100), Scalar(130, 255, 255), bin_img);
    inRange(hsv_img, Scalar(0, 170, 170), Scalar(20, 255, 255), bin_img);// can probably be 10

    // Noise removal with morphology
    Mat kernel;
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(bin_img, bin_img, MORPH_CLOSE, kernel);

    // Contour detection
    vector<vector<Point> > contours; 
    vector<Vec4i> hierarchy; 
    findContours(bin_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    auto hulls = contours;

    //Compute object sizes using moments and pick the largest object 
    vector<tuple<Moments, vector<Point>>> detections;
    for (auto& hull : hulls) {
        detections.push_back(std::make_tuple(moments(hull, true), std::move(hull)));
    }
    auto v = std::max_element(detections.begin(), detections.end(), 
        [](auto& t1, auto& t2) {
            auto &[m1, h1] = t1;
            auto &[m2, h2] = t2;
            return m1.m00 < m2.m00;
        }
    );
    bool found = false;

    if(v != detections.end()) {
        auto& [m, h] = *v;
        Point detectionCenter(m.m10/m.m00, m.m01/m.m00);
        circle(debug_img, detectionCenter, 4, Scalar(255, 255, 0), -1); 


        std::vector<vector<Point>> temp = {h};
        drawContours(debug_img, temp, 0, Scalar(0, 0, 255), 3);

        // search for max min
        auto min_pt = *std::min_element(h.begin(), h.end(), [] (auto& p1, auto& p2) { return p1.x < p2.x; });
        auto max_pt = *std::max_element(h.begin(), h.end(), [] (auto& p1, auto& p2) { return p1.x < p2.x; });
        line(debug_img, min_pt, max_pt, Scalar(255, 0, 0), 3);

        // compute points that mark the boundaries
        Point vec = max_pt - min_pt;
        vec = 1* (vec/norm(vec));
        auto search_pt_left = min_pt + vec;
        auto search_pt_right = max_pt - vec;
        circle(debug_img, search_pt_left, 2, Scalar(255, 255, 0), -1); 
        circle(debug_img, search_pt_right, 2, Scalar(255, 0, 255), -1); 

        // Attempt to get depth at the center
        if(m.m00 > 1000 && !cur_depth.empty()) {
            float center_depth = cur_depth.at<float>((int)detectionCenter.y, (int)detectionCenter.x);
            if(!isnan(center_depth)) {
                object_detection.update_detection(detectionCenter.x, detectionCenter.y, center_depth, cur_odom);
                found = true;
                std::cout << "stopping " << center_depth << std::endl;
            }
        }
    }

    imshow("post filter", bin_img);
    imshow("pre filter", debug_img);
    imshow("cur depth", cur_depth);
    //imwrite("/home/ashwin/Desktop/blah.jpg", cur_rgb);
    waitKey(10);

    return found;
}

SLOSBot::State SLOSBot::search_for_object() {
    if(!run_obj_detection()) { 
        // tell the mbot to rotate
        lcm_to_ros::mbot_motor_command_t msg;
        msg.angular_v = 0.9;
        motor_command_pub.publish(msg);
        return State::SEARCH_FOR_OBJECT;
    } else {
        lcm_to_ros::mbot_motor_command_t msg;
        motor_command_pub.publish(msg);
        return State::DRIVE_TO_OBJECT;
    }
}

SLOSBot::State SLOSBot::drive_to_object() {
    run_obj_detection();

    auto pt = object_detection.get_point_in_odom();
    auto odom = Eigen::Vector2d(cur_odom.x, cur_odom.y);

    lcm_to_ros::mbot_motor_command_t msg;
    auto drive_error = pt - odom;
    //double ang_error_dp = (drive_error/drive_error.norm()).dot(Eigen::Vector2d(cos(cur_odom.theta), sin(cur_odom.theta)));
    auto drive_dir = drive_error/drive_error.norm();
    auto cur_dir = Eigen::Vector2d(cos(cur_odom.theta), sin(cur_odom.theta));
    double ang_error = cur_dir.x()*drive_dir.y() - drive_dir.x()*cur_dir.y();
    //auto angular_error = acos(ang_error_dp);
    std::cout << "drive error norm " << drive_error.norm() << "ang_error " << ang_error << std::endl;

    State ret_state;
    if(drive_error.norm() < 0.04) {
        msg.trans_v = 0.0;
        msg.angular_v = 0.0;
        ret_state =  State::MATCH_OBJECT; // TODO change
    } else {
        msg.trans_v = 0.2;
        msg.angular_v = ang_error;

        ret_state= State::DRIVE_TO_OBJECT;
    }

    motor_command_pub.publish(msg);
    return ret_state;
}

void SLOSBot::execute_sm() {
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        switch(state) {
            case State::SEARCH_FOR_OBJECT:
                state = search_for_object();
                break;
            case State::DRIVE_TO_OBJECT:
                state = drive_to_object();
                break;
            default:
                std::cout << "invalid state" << std::endl;
        };

        ros::spinOnce();
        loop_rate.sleep();

        #ifdef DEBUG
        viewer->spinOnce(10);
        #endif
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slosbot");
    std::cout << "hi " << std::endl;
    SLOSBot bot;
    bot.execute_sm();
}
