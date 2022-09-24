#ifndef _TRAJECTORYGEN_H_
#define _TRAJECTORYGEN_H_

#include <eigen/Eigen/Dense>

/// @brief generate atan-like trajectory
/// @param steps number of simulation steps
/// @param ec number of references
/// @param xd longitudinal vehicle speed
/// @param h time interval for sample step
/// @param Ys initial value of Y
/// @param Ye final value of Y
/// @return reference trajectory for psi and Y
Eigen::MatrixXd atanTraj(double steps, int ec, double xd, double h, double Ys, double Ye);

#endif