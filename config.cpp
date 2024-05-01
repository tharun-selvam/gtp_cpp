#include <iostream>
#include <Eigen/Dense>


class Config{
    public: 
        static float dt;
        static float n_steps;
        static Eigen::Matrix2f track_waypoints;
        static float track_width;
        static float max_curvature;
        
        static float collision_radius;
        static float v_max;
        static float a_max;
};


float Config::dt = 1;
float Config::n_steps = 5;
Eigen::Matrix2f Config::track_waypoints{
    {0, 0},
    {2.5, 5},
    {4, 0},
    {6, 9},
    {1, 8},
    {0, 0}
};
float Config::track_width = 0.8;
float Config::max_curvature = 1.0;

float Config::collision_radius = 0.01;
float Config::v_max = 0.2;
float Config::a_max = 1;
