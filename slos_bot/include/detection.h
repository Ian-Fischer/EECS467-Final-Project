#include <Eigen/Dense>
#include <chrono>
#include "lcm_to_ros/odometry_t.h"

class DetectionManager {
    public: 
        // Returns the position of the detection relative to the camera
        Eigen::Vector3d get_point_in_cam();

        // Returns the position of the [cached] detection in odometry frame
        Eigen::Vector2d get_point_in_odom();

        
        void update_detection(float u, float v, float depth, lcm_to_ros::odometry_t odom);
        void reset_detection();

    private:
        float u, v;
        float depth;
        Eigen::Vector2d odom;
         
        int hit_count = 0;
        std::chrono::time_point<std::chrono::system_clock> last_detection_time;
};