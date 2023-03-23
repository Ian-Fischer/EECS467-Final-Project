#include <algorithm>
#include <iostream>
#include "ros/init.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"
#include "slosbot.h"
#include "lcm_to_ros/mbot_motor_command_t.h"
#include "cv_bridge/cv_bridge.h"


SLOSBot::SLOSBot() :
 motor_command_pub( nh.advertise<lcm_to_ros::mbot_motor_command_t>("lcm_to_ros/MBOT_MOTOR_COMMAND", 1)),
 depth_img_sub( nh.subscribe("camera/depth_registered/image_raw", 1, &SLOSBot::depth_img_cb, this) ),
 rgb_img_sub( nh.subscribe("camera/rgb/image_rect_color", 1, &SLOSBot::rgb_img_cb, this) ),
 pointcloud_sub( nh.subscribe("camera/depth_registered/points", 1, &SLOSBot::pointcloud_cb, this) ),
 cur_pc(new pcl::PointCloud<pcl::PointXYZRGB>)
 #ifdef DEBUG
 ,viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
 #endif
{
    #ifdef DEBUG
    viewer->addPointCloud (cur_pc, "sample cloud");
    viewer->addCoordinateSystem();
    #endif
   //viewer->addSphere(pcl::PointXYZ(0.0, 0.0, 0.0), 0.01, 1.0, 1.0, 0.0);
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

void SLOSBot::pointcloud_cb(sensor_msgs::PointCloud2Ptr pc) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc,pcl_pc2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cur_pc);
    #ifdef DEBUG
    viewer->removePointCloud("sample cloud");
    viewer->addPointCloud(cur_pc, "sample cloud");
    #endif
}

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

    // Make sure there is a current image
    if(cur_rgb.rows == 0 || cur_rgb.cols == 0) return;
    Mat debug_img = cur_rgb.clone();

    // HSV segmentation
    Mat hsv_img;
    cvtColor(cur_rgb, hsv_img, COLOR_BGR2HSV);

    Mat bin_img(hsv_img.size(), 0); 
    inRange(hsv_img, Scalar(0, 120, 150), Scalar(10, 255, 255), bin_img);

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
    if(v != detections.end()) {
        auto& [m, h] = *v;
        circle(debug_img, Point(m.m10/m.m00, m.m01/m.m00), 4, Scalar(255, 255, 0), -1); 


        std::vector<vector<Point>> temp = {h};
        drawContours(debug_img, temp, 0, Scalar(0, 0, 255), 3);

        // search for max min
        auto min_pt = *std::min_element(h.begin(), h.end(), [] (auto& p1, auto& p2) { return p1.x < p2.x; });
        auto max_pt = *std::max_element(h.begin(), h.end(), [] (auto& p1, auto& p2) { return p1.x < p2.x; });
        line(debug_img, min_pt, max_pt, Scalar(255, 0, 0), 3);

        // compute points that mark the boundaries
        Point vec = max_pt - min_pt;
        vec = 10 * (vec/norm(vec));
        auto search_pt_left = min_pt + vec;
        auto search_pt_right = max_pt - vec;
        circle(debug_img, search_pt_left, 2, Scalar(255, 255, 0), -1); 
        circle(debug_img, search_pt_right, 2, Scalar(255, 0, 255), -1); 

        // attempt to compute the width of the object
        try {

            auto &p = cur_pc->at(m.m10/m.m00, m.m01/m.m00);

            auto min_pt_3d = cur_pc->at(search_pt_left.x, search_pt_left.y);
            auto max_pt_3d = cur_pc->at(search_pt_right.x, search_pt_right.y);
            float width = max_pt_3d.x - min_pt_3d.x;
            //std::cout << "apparent width " << max_pt_3d.x - min_pt_3d.x << std::endl;

            // tell the mbot to stop
            if(width > 0.35 && width < 0.45) {
                lcm_to_ros::mbot_motor_command_t msg;
                motor_command_pub.publish(msg);
            }

            #ifdef DEBUG
            viewer->removeShape("sphere");
            viewer->addSphere(pcl::PointXYZ(p.x, p.y, p.z), 0.01, 1.0, 1.0, 0.0);
            viewer->removeShape("sphere1");
            viewer->addSphere(min_pt_3d, 0.01, 1.0, 1.0, 0.0, "sphere1");
            viewer->removeShape("sphere2");
            viewer->addSphere(max_pt_3d, 0.01, 1.0, 1.0, 0.0, "sphere2");
            #endif

        } catch(...) {
        }



    } else {
        // tell the mbot to rotate
        lcm_to_ros::mbot_motor_command_t msg;
        msg.angular_v = 1.5;
        motor_command_pub.publish(msg);
    }

    #ifdef DEBUG
    imshow("post filter", bin_img);
    imshow("pre filter", debug_img);
    imshow("cur depth", cur_depth);
    //imwrite("/home/ashwin/Desktop/peepee.jpg", cur_rgb);
    waitKey(10);
    #endif


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
