#include <iostream>
#include <Eigen/Dense>


namespace Config{
    float dt = 1;
    float n_steps = 5;
    Eigen::Matrix2f track_waypoints{
        {0, 0},
        {2.5, 5},
        {4, 0},
        {6, 9},
        {1, 8},
        {0, 0}
    };
    float track_width = 0.8;
    float max_curvature = 1.0;
    
    float collision_radius = 0.01;
    float v_max = 0.2;
    float a_max = 1;
};