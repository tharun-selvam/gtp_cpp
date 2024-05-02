#include <iostream>
#include <Eigen/Dense>
#include <gurobi_c++.h>
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
            this->traj = init_traj(0, track.waypoints.row(0))
            this->i_ego = 0;
            this->nc_weight = 1.0;
            this->nc_relax_weight = 128.0;
            this->track_relax_weight = 128.0;
            this->max_curvature = config.max_curvature;

        };

        pair<ArrayXd, ArrayXd> init_traj(int i, ArrayXd& p_0);
        ArrayX2d best_response(int i, Array22d& state, vector<pair<ArrayXd, ArrayXd>> trajectories);

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


ArrayX2d SE_IBR::best_response(int i, Array22d& state, vector<pair<ArrayXd, ArrayXd>> trajectories){

    int j = (i + 1) % 2;
    double v_max = this->config.v_max;
    double a_max = this->config.a_max;
    double d_coll = 2 * this->config.collision_radius;
    double d_safe = 2 * this->config.collision_radius;
    Array2d p_i = state.row(i);
    Array2d p_j = state.row(j);
    // ArrayXd traj_A = trajectories[i].first;
    // ArrayXd traj_B = trajectories[i].second;
    // ArrayXd traj_A_opp = trajectories[j].first;
    // ArrayXd traj_B_opp = trajectories[j].second;
    double width = this->config.track_width;


    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    // Define decision variables
    std::vector<std::vector<GRBVar>> strat_A(n_steps, std::vector<GRBVar>(3));
    std::vector<std::vector<GRBVar>> strat_B(n_steps, std::vector<GRBVar>(3));
    for (int k = 0; k < n_steps; ++k) {
        for (int i = 0; i < 3; ++i) {
            strat_A[k][i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            strat_B[k][i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
        }
    }

    // Add continuity constraints
    for (int k = 0; k < n_steps - 1; ++k) {
        for (int i = 0; i < 3; ++i) {
            model.addConstr(
                strat_A[k][i] + strat_A[k][i+1] * (k + 1) + strat_A[k][i+2] * (k + 1) * (k + 1) -
                (strat_A[k+1][i] + strat_A[k+1][i+1] * (k + 1) + strat_A[k+1][i+2] * (k + 1) * (k + 1)) == 0
            );
            model.addConstr(
                strat_B[k][i] + strat_B[k][i+1] * (k + 1) + strat_B[k][i+2] * (k + 1) * (k + 1) -
                (strat_B[k+1][i] + strat_B[k+1][i+1] * (k + 1) + strat_B[k+1][i+2] * (k + 1) * (k + 1)) == 0
            );
        }
    }

    model.addConstr(strat_A[0][0] - p_i(0) == 0);
    model.addConstr(strat_B[0][0] - p_i(1) == 0);

    // Non-collision constraints (doubtful completely because of p_curr)
    // ahould p_curr be a GRBContr?
    std::vector<GRBConstr> nc_constraints;
    GRBLinExpr nc_obj(0.0);
    GRBLinExpr nc_relax_obj(0.0);
    double non_collision_objective_exp = 0.5; // exponentially decreasing weight

    for (int k = 0; k < n_steps; ++k) {
        ArrayXd A_opp = trajectories[j].first.row(k);
        ArrayXd B_opp = trajectories[j].second.row(k);
        ArrayXd A_ego = trajectories[i].first.row(k);
        ArrayXd B_ego = trajectories[i].second.row(k);
        ArrayXd p_ego = ArrayXd::Zero(2);
        ArrayXd p_opp = ArrayXd::Zero(2);

        for (int j = 0; j < 3; ++j) {
            p_ego(0) += A_ego(j) * pow(k, j);
            p_ego(1) += B_ego(j) * pow(k, j);
            p_opp(0) += A_opp(j) * pow(k, j);
            p_opp(1) += B_opp(j) * pow(k, j);
        }

        ArrayXd beta = p_opp - p_ego;
        if (beta.norm() >= 1e-6) {
            beta /= beta.norm();
        }

        // doubtful if we have to use .get(GRB_DoubleAttr_X)
        ArrayXd p_curr = ArrayXd::Zero(2);
        for (int j = 0; j < 3; ++j) {
            p_curr(0) += strat_A[k][j].get(GRB_DoubleAttr_X) * pow(k, j);
            p_curr(1) += strat_B[k][j].get(GRB_DoubleAttr_X) * pow(k, j);
        }

        GRBConstr nc_constraint;
        nc_constraint = model.addConstr(beta.dot(p_opp) - beta.transpose().matrix() * p_curr.matrix() >= d_coll);
        nc_constraints.push_back(nc_constraint);

        // doubtful of the below exppr
        nc_obj += pow(non_collision_objective_exp, k) * fmax(0.0, d_safe - (beta.dot(p_opp) - beta.transpose().matrix() * p_curr.matrix()));
        nc_relax_obj += pow(non_collision_objective_exp, k) * fmax(0.0, d_coll - (beta.dot(p_opp) - beta.transpose().matrix() * p_curr.matrix()));
    }

    // c_constraints
    std::vector<GRBConstr> c_constraint;

    for (int k = 0; k < n_steps; ++k) {
        c_constraint.push_back(model.addConstr(strat_A[k][0] <= 100));
        c_constraint.push_back(model.addConstr(strat_A[k][0] >= -100));
        c_constraint.push_back(model.addConstr(strat_B[k][0] <= 100));
        c_constraint.push_back(model.addConstr(strat_B[k][0] >= -100));
    }

    // velocity constraints
    std::vector<GRBConstr> vel_constraints;

    for (int k = 0; k < n_steps; ++k) {
        GRBLinExpr vel_x = strat_A[k][1] + 2 * strat_A[k][2] * k;
        GRBLinExpr vel_y = strat_B[k][1] + 2 * strat_B[k][2] * k;
        vel_constraints.push_back(model.addConstr(vel_x * vel_x + vel_y * vel_y <= v_max * v_max));
    }

    // acceleration constraints
    std::vector<GRBConstr> acc_constraints;

    for (int k = 0; k < n_steps; ++k) {
        GRBLinExpr c_A_sq = strat_A[k][2] * strat_A[k][2];
        GRBLinExpr c_B_sq = strat_B[k][2] * strat_B[k][2];
        acc_constraints.push_back(model.addConstr(c_A_sq + c_B_sq <= a_max));
    }

    // track contraints(check one; mostly correct)
    std::vector<GRBConstr> track_constraints;
    GRBQuadExpr track_obj;
    double track_objective_exp = 0.5;

    for (int k = 0; k < n_steps; ++k) {
        ArrayXd A_ego = trajectories[i].first.row(k);
        ArrayXd B_ego = trajectories[i].second.row(k);
        ArrayXd p_cur(2);
        p_cur << A_ego(0) + A_ego(1) * k + A_ego(2) * k * k,
                B_ego(0) + B_ego(1) * k + B_ego(2) * k * k;

        vector<double> track_info;
        track_info = track.nearest_trackpoint(p_cur, track_info);
        ArrayXd c = track_info.head(2);
        ArrayXd t = track_info.segment(2, 2);
        ArrayXd n = track_info.tail(2);

        ArrayXd p_new(2);
        p_new << strat_A[k][0] + strat_A[k][1] * k + strat_A[k][2] * k * k,
                strat_B[k][0] + strat_B[k][1] * k + strat_B[k][2] * k * k;

        GRBLinExpr track_constraint_1 = n.transpose() * p_new - n.transpose().matrix() * c.matrix();
        GRBLinExpr track_constraint_2 = -track_constraint_1;

        track_constraints.push_back(model.addConstr(track_constraint_1 <= width - config.collision_radius));
        track_constraints.push_back(model.addConstr(track_constraint_2 <= width - config.collision_radius));

        track_obj += pow(track_objective_exp, k) * (
            GRBmax_(0.0, track_constraint_1 - (width - config.collision_radius)) +
            GRBmax_(0.0, -track_constraint_2 - (width - config.collision_radius))
        );
    }
}

