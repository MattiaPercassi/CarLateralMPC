#include <eigen/Eigen/Dense>
#include "solveOpt.h"

template <int n, int sc, int ec, int uc>
Eigen::Matrix<float, n * uc, 1> calculateOptInputs(Eigen::Matrix<float, n * uc, n * uc> &invH, Eigen::Matrix<float, n * uc, sc + n * ec> &F, Eigen::Matrix<float, sc, 1> &xk, Eigen::Matrix<float, ec * n, 1> &refg)
{
    Eigen::Matrix<float, sc + ec * n, 1> xtemp = Eigen::Matrix<float, sc + ec * n, 1>::Zero();
    xtemp.block(0, 0, sc, 1) = xk;
    xtemp.block(sc, 0, n * ec, 1) = refg;
    Eigen::Matrix<float, n * uc, 1> ug = -invH * F * xtemp.transpose();
    return ug;
};

Eigen::MatrixXf calculateOptInputsdyn(Eigen::MatrixXf &invH, Eigen::MatrixXf &F, Eigen::MatrixXf &xk, Eigen::MatrixXf &refg, int n, int sc, int ec)
{
    Eigen::MatrixXf xtemp = Eigen::MatrixXf::Zero(sc + ec * n, 1);
    xtemp.block(0, 0, sc, 1) = xk;
    xtemp.block(sc, 0, n * ec, 1) = refg;
    auto ug = -invH * F * xtemp.transpose();
    return ug;
};