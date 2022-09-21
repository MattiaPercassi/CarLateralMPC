#ifndef _GLOBALMATRIXES_H_
#define _GLOBALMATRIXES_H_

#include <eigen/Eigen/Dense>

// use a template function where the template parameter is the number of steps for the control

/// @brief Build global A matrix
/// @tparam n integrations steps
/// @tparam sc number of states
/// @param At Augmented matrix of the LTI discrete
/// @return Global Ag matrix
template <int n, int sc>
Eigen::Matrix<float, n * sc, sc> buildGlobalA(Eigen::Matrix<float, sc, sc> &At);

/// @brief Build global B matrix
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam uc number of inputs
/// @param At Augmented matrix of the LTI discrete
/// @param Bt Augmented matrix of the LTI discrete
/// @return Global Bg matrix
template <int n, int sc, int uc>
Eigen::Matrix<float, n * sc, n * uc> buildGlobalB(Eigen::Matrix<float, sc, sc> &At, Eigen::Matrix<float, sc, uc> &Bt);

#endif