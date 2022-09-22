#ifndef _GRADIENTMATRIXES_H_
#define _GRADIENTMATRIXES_H_

#include <eigen/Eigen/Dense>

/// @brief Build H^-1 matrix
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam uc length of input vector
/// @param Bg Global Bg matrix
/// @param Qg Global Qg matrix
/// @param Rg Global Rg matrix
/// @return Matrix H^-1 of the cost gradient
template <int n, int sc, int uc>
Eigen::Matrix<float, n * uc, n * uc> buildinverseH(Eigen::Matrix<float, n * sc, n * uc> &Bg, Eigen::Matrix<float, n * sc, n * sc> &Qg, Eigen::Matrix<float, n * uc, n * uc> &Rg);

Eigen::MatrixXf buildinverseHdyn(Eigen::MatrixXf &Bg, Eigen::MatrixXf &Qg, Eigen::MatrixXf &Rg);

/// @brief Build F matrix
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam ec length of error vector
/// @tparam uc length of input vector
/// @param Ag Global Ag matrix
/// @param Qg Global Qg matrix
/// @param Bg Global Bg matrix
/// @param Tg Global Tg matrix
/// @return Matrix F of the cost gradient
template <int n, int sc, int ec, int uc>
Eigen::Matrix<float, n * uc, sc + n * ec> buildF(Eigen::Matrix<float, n * sc, sc> &Ag, Eigen::Matrix<float, n * sc, n * sc> &Qg, Eigen::Matrix<float, n * sc, n * uc> &Bg, Eigen::Matrix<float, n * ec, n * sc> &Tg);

Eigen::MatrixXf buildFdyn(Eigen::MatrixXf &Ag, Eigen::MatrixXf &Qg, Eigen::MatrixXf &Bg, Eigen::MatrixXf &Tg, int n, int sc, int ec, int uc);

#endif