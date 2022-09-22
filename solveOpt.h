#ifndef _SOLVEOPT_H_
#define _SOLVEOPT_H_

#include <eigen/Eigen/Dense>

/// @brief Calculate optimized input vector
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam ec length of error vector
/// @tparam uc length of input vector
/// @param invH gradient matrix
/// @param F gradient matrix
/// @param xk states vector at current step
/// @param refg reference global vector for the next n steps
/// @return input vector
template <int n, int sc, int ec, int uc>
Eigen::Matrix<float, n * uc, 1> calculateOptInputs(Eigen::Matrix<float, n * uc, n * uc> &invH, Eigen::Matrix<float, n * uc, sc + n * ec> &F, Eigen::Matrix<float, sc, 1> &xk, Eigen::Matrix<float, ec * n, 1> &refg);

Eigen::MatrixXf calculateOptInputsdyn(Eigen::MatrixXf &invH, Eigen::MatrixXf &F, Eigen::MatrixXf &xk, Eigen::MatrixXf &refg, int n, int sc, int ec);

#endif