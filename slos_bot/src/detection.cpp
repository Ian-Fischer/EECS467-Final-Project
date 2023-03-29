#include "detection.h"

// Returns the position of the detection relative to the camera
Eigen::Vector3d DetectionManager::get_point_in_cam() {
    Eigen::Matrix3d K;
    K << 570.3422241210938, 0.0, 319.5, 0.0, 570.3422241210938, 239.5, 0.0, 0.0, 1.0;

    Eigen::Vector3d px;
    px << u * depth, v * depth, depth;

    Eigen::Vector3d pt = K.colPivHouseholderQr().solve(px);
    //std::cout << pt.x() << "," << pt.y() << "," << pt.z() << std::endl;
    return pt;
}

// Returns the position of the [cached] detection in odometry frame
Eigen::Vector2d DetectionManager::get_point_in_odom() {
    return get_point_in_cam().head<2>() + this->odom;
}


void DetectionManager::update_detection(float u, float v, float depth, lcm_to_ros::odometry_t odom) {
    std::cout << "update detection called " << std::endl;
    this->u = u;
    this->v = v;
    this->depth = depth;
    this->odom = Eigen::Vector2d(odom.x, odom.y);
    //get_point_in_cam();
    //K: [570.3422241210938, 0.0, 319.5, 0.0, 570.3422241210938, 239.5, 0.0, 0.0, 1.0]
}
//void reset_detection();
