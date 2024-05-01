#include <iostream>
#include <Eigen/Dense>
#include "spline.h"
#include "config.cpp"

using namespace std;
using namespace Eigen;

class SE_IBR{
    public:
        SE_IBR(const Config& config){
            this->config = config;
            this->dt = config.dt;
            this->n_steps = config.n_steps;
            // init trajectory here
            this->i_ego = 0;
            this->nc_weight = 1.0;
            this->nc_relax_weight = 128.0;
            this->track_relax_weight = 128.0;
            this->max_curvature = config.max_curvature;

        };

        pair<MatrixXd, MatrixXd> init_traj(int i, ArrayXd& p_0);

        Config config;
        int dt;
        // track
        int n_steps;
        pair<MatrixXd, MatrixXd> traj;
        int i_ego;
        float nc_weight;
        float nc_relax_weight;
        float track_relax_weight;
        float max_curvature;
};

pair<MatrixXd, MatrixXd> SE_IBR::init_traj(int i, ArrayXd& p_0){
    // Initialize the trajectory at the start of the race.
    // Assuming that the 1st waypoint is the start point.
    // Simply a line following the tangent at the start point.

    // :param i: Index of the current track frame
    // :return: Initial trajectory

    // Ai = np.zeros((self.n_steps, 3))
    // Bi = np.zeros((self.n_steps, 3))
    // for k in range(self.n_steps):
    //     idx, c0, t0, n0 = self.track.nearest_trackpoint(p_0)
    //     p_1 = p_0 + self.config.v_max * t0
    //     _, c1, t1, n1 = self.track.nearest_trackpoint(p_1)
    //     p_2 = p_1 + self.config.v_max * t1

    //     # fit a quadratic to the line between the two points give two eqn for each x and y
    //     eqn_x = np.polyfit([k, k+1, k+2], [p_0[0], p_1[0], p_2[0]], 2)
    //     eqn_y = np.polyfit([k, k+1, k+2], [p_0[1], p_1[1], p_2[1]], 2)
    //     # plt.show()
    //     Ai[k, :] = [eqn_x[2], eqn_x[1], eqn_x[0]]
    //     Bi[k, :] = [eqn_y[2], eqn_y[1], eqn_y[0]]
    //     p_0 = p_1
    // return (Ai, Bi)

    ArrayX3d Ai(n_steps, 3); // initialised to zero automatically
    ArrayX3d Bi(n_steps, 3);

    for(int k=0; k<n_steps; k++){
        
    }
}