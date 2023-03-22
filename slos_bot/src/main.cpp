#include <iostream>
#include "ros/init.h"
#include "ros/publisher.h"
#include "slosbot.h"
#include "lcm_to_ros/mbot_motor_command_t.h"
#include "cv_bridge/cv_bridge.h"

SLOSBot::SLOSBot() :
 motor_command_pub( nh.advertise<lcm_to_ros::mbot_motor_command_t>("lcm_to_ros/MBOT_MOTOR_COMMAND", 1)),
 depth_img_sub( nh.subscribe("camera/depth_registered/image_raw", 1, &SLOSBot::depth_img_cb, this) ),
 rgb_img_sub( nh.subscribe("camera/rgb/image_rect_color", 1, &SLOSBot::rgb_img_cb, this) ) 
{} 

void SLOSBot::depth_img_cb(sensor_msgs::ImageConstPtr img) {}
void SLOSBot::rgb_img_cb(sensor_msgs::ImageConstPtr img) {
    std::cout << "recieved im" << std::endl;
    try {
        cv_bridge::CvImagePtr im;
        im = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        cur_rgb = im->image;
    } catch(cv_bridge::Exception) {
        std::cout << "Could not convert image" << std::endl;
    }

    
}

void SLOSBot::search_for_object() {
    using namespace cv;
    using namespace std;

    // get image
    if(cur_rgb.rows == 0 || cur_rgb.cols == 0) return;

    Mat debug_img = cur_rgb.clone();

    // hsv segmentation //cv::imshow("img", cur_rgb);
    //cv::waitKey(0);
    Mat hsv_img;
    cvtColor(cur_rgb, hsv_img, COLOR_BGR2HSV);

    Mat bin_img(hsv_img.size(), 0); 
    inRange(hsv_img, Scalar(0, 120, 150), Scalar(10, 255, 255), bin_img);

    // Noise removal with morphology
    Mat kernel;
    kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(bin_img, bin_img, MORPH_CLOSE, kernel);

    // each object's contour is a list of points. We need a list of contours
    vector<vector<Point> > contours; 
    // The hiearchy contains informations about how objects are nested, we don't need it
    vector<Vec4i> hierarchy; 
    // Find the contours and draw all of them (-1)
    findContours(bin_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    //drawContours(debug_img, contours, -1, Scalar(0, 0, 255), 3);

    vector<vector<Point> > hulls(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        convexHull(contours[i], hulls[i]);
    }

    //Compute object angles using moments 
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
    if(v != detections.end()) {
        auto& [m, h] = *v;
        circle(debug_img, Point(m.m10/m.m00, m.m01/m.m00), 4, Scalar(255, 255, 0), -1);

        // tell the mbot to stop
        lcm_to_ros::mbot_motor_command_t msg;
        motor_command_pub.publish(msg);

    } else {
        // tell the mbot to rotate
        lcm_to_ros::mbot_motor_command_t msg;
        msg.angular_v = 1.0;
        motor_command_pub.publish(msg);
    }

    //imshow("post filter", bin_img);
    //imshow("pre filter", debug_img);
    //imwrite("/home/ashwin/Desktop/peepee.jpg", cur_rgb);
    //waitKey(10);



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
