#include <iostream>
#include <Eigen/Dense>
#include "spline.h"
#include "config.cpp"
#include "track.cpp"
#include "PolyfitEigen.hpp"

using namespace std;
using namespace Eigen;

class SE_IBR{
    public:
        SE_IBR(const Config& config){
            // 


            this->config = config;
            this->dt = config.dt;
            this->track = Track('track_centers.csv', 'track_tangent.csv', 'track_normals.csv');
            this->n_steps = config.n_steps;
            // init trajectory here
            this->i_ego = 0;
            this->nc_weight = 1.0;
            this->nc_relax_weight = 128.0;
            this->track_relax_weight = 128.0;
            this->max_curvature = config.max_curvature;

        };

        pair<ArrayXd, ArrayXd> init_traj(int i, ArrayXd& p_0);

        Config config;
        int dt;
        Track track;
        int n_steps;
        pair<ArrayXd, ArrayXd> traj;
        int i_ego;
        float nc_weight;
        float nc_relax_weight;
        float track_relax_weight;
        float max_curvature;
};

pair<ArrayXd, ArrayXd> SE_IBR::init_traj(int i, ArrayXd& p_0){
    // Initialize the trajectory at the start of the race.
    // Assuming that the 1st waypoint is the start point.
    // Simply a line following the tangent at the start point.

    // :param i: Index of the current track frame
    // :return: Initial trajectory

    ArrayX3d Ai(n_steps, 3); // initialised to zero automatically
    ArrayX3d Bi(n_steps, 3);

    for(int k=0; k<n_steps; k++){
        vector<double> ans = track.nearest_trackpoint(p_0);
        double t0 = ans[3];
        ArrayXd p1 = p_0 + config.v_max * t0;
        ans = track.nearest_trackpoint(p1);
        double t1 = ans[3];
        ArrayXd p2 = p1 + config.v_max * t1;

        // fit a quadratic to the line between the two points give two eqn for each x and y
        vector<double> x{k, k+1, k+2};
        vector<double> y{p_0(0), p1(0), p2(0)};
        vector<double> y1{p_0(1), p1(2), p2(3)};
        vector<double> eqn_x = polyfit_Eigen(x, y, 2);
        vector<double> eqn_y = polyfit_Eigen(x, y1, 2);

        Ai(k, 0) = eqn_x[2];
        Ai(k, 1) = eqn_x[1];
        Ai(k, 2) = eqn_x[0];

        Bi(k, 0) = eqn_y[2];
        Bi(k, 1) = eqn_y[1];
        Bi(k, 2) = eqn_y[0];

        p_0 = p1;

        return make_pair(Ai, Bi);
    }
}