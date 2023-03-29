
#include "Eigen/src/Core/Matrix.h"

class DetectionManager {
    public: 
        Eigen::Vector3d get_point();
        void update_detection(float u, float v, float depth);
        void reset_detection(float u, float v, float depth);

    private:
        float u, v;
        float depth;
        int hit_count = 0;
};