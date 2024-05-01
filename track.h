#include <iostream>
#include <Eigen/Dense>
#include "spline.h"

class Track {
    public:
        template <typename Derived> Track(const Eigen::MatrixBase<Derived>& waypoints, float track_width) {
            this->waypoints = waypoints;
            this->track_width = track_width;

            Eigen::MatrixXd diff = waypoints.bottomRows(waypoints.rows() - 1) - waypoints.topRows(waypoints.rows() - 1);
            Eigen::VectorXd dists = diff.rowwise().norm();
            arc_length = Eigen::VectorXf::Zero(waypoints.rows());
            for(int i=1; i<waypoints.rows(); i++){
                arc_length(i) = arc_length(i-1) + dists(i-1);
            }
            // construct track here
            Eigen::VectorXd taus = Eigen::ArrayXd::LinSpaced(144, 0, arc_length(arc_length.size()-1));
            // intialise track centers here using cubic spline


            
        };


        Eigen::Matrix<double, Eigen::Dynamic, 2> waypoints;
        Eigen::VectorXf arc_length;
        tk::spline track;
        Eigen::Matrix<double, Eigen::Dynamic, 2> track_centers;
        Eigen::Matrix<double, Eigen::Dynamic, 2> track_centers;

};