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

Eigen::MatrixXf buildGlobalAdyn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &At, int n);

/// @brief Build global B matrix
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam uc number of inputs
/// @param At Augmented matrix of the LTI discrete
/// @param Bt Augmented matrix of the LTI discrete
/// @return Global Bg matrix
template <int n, int sc, int uc>
Eigen::Matrix<float, n * sc, n * uc> buildGlobalB(Eigen::Matrix<float, sc, sc> &At, Eigen::Matrix<float, sc, uc> &Bt);

Eigen::MatrixXf buildGlobalBdyn(Eigen::MatrixXf &At, Eigen::MatrixXf &Bt, int n, int sc, int uc);

/// @brief Build global Q matrix
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam ec length of error vector
/// @param Q Weight matrix for error vectors
/// @param S Weight matrix for last error vector
/// @param Ct Augmented matrix of the discrete LTI
/// @return Global Qg matrix
template <int n, int sc, int ec>
Eigen::Matrix<float, n * sc, n * sc> buildGlobalQ(Eigen::Matrix<float, ec, ec> &Q, Eigen::Matrix<float, ec, ec> &S, Eigen::Matrix<float, ec, sc> &Ct);

Eigen::MatrixXf buildGlobalQdyn(Eigen::MatrixXf &Q, Eigen::MatrixXf &S, Eigen::MatrixXf &Ct, int n, int sc, int ec);

/// @brief Build global T matrix
/// @tparam n integration steps
/// @tparam sc number of states
/// @tparam ec length of error vector
/// @param Q Weight matrix for error vectors
/// @param S Weight matrix for last error vector
/// @param Ct Augmented matrix of the discrete LTI
/// @return Global Tg matrix
template <int n, int sc, int ec>
Eigen::Matrix<float, n * ec, n * sc> buildGlobalT(Eigen::Matrix<float, ec, ec> &Q, Eigen::Matrix<float, ec, ec> &S, Eigen::Matrix<float, ec, sc> &Ct);

Eigen::MatrixXf buildGlobalTdyn(Eigen::MatrixXf &Q, Eigen::MatrixXf &S, Eigen::MatrixXf &Ct, int n, int sc, int ec);

/// @brief Build global R matrix
/// @tparam n integration steps
/// @tparam uc length of input vector
/// @param R Weight matrix for inputs
/// @return Global Rg matrix
template <int n, int uc>
Eigen::Matrix<float, n * uc, n * uc> buildGlobalR(Eigen::Matrix<float, uc, uc> &R);

Eigen::MatrixXf buildGlobalRdyn(Eigen::MatrixXf &R, int n, int uc);

#endif