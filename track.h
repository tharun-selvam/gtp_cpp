#include <iostream>
#include <Eigen/Dense>

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
            
        };


        Eigen::Matrix<double, Eigen::Dynamic, 2> waypoints;
        Eigen::VectorXf arc_length;
};