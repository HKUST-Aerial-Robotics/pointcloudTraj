#ifndef _TRAJECTORY_OPTIMIZER_H_
#define _TRAJECTORY_OPTIMIZER_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

using namespace std;
using namespace Eigen;

class TrajectoryOptimizerSOCP {
private:
        double _objective;

public:
        TrajectoryOptimizerSOCP();

        ~TrajectoryOptimizerSOCP();

        /* Use Bezier curve for the piecewise trajectory */
        Eigen::MatrixXd BezierConicOptimizer(
            const MatrixXd  &corridor_centers,
            const VectorXd  &corridor_radius,
            const VectorXd  &corridor_times,
            const vector<int>      & poly_orders,
            const vector<MatrixXd> & FMs,
            const MatrixXd &pos,
            const MatrixXd &vel,
            const MatrixXd &acc,
            const double max_vel,
            const double max_acc,
            const int minimize_order,
            const bool is_limit_vel,
            const bool is_limit_acc,
            const bool is_print_soving);  // define the order to which we minimize.   1 -- velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap  

        double getObjective(){ 
            return _objective;
        };

};

#endif
