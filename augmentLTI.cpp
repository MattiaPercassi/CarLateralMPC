#include <eigen/Eigen/Dense>
#include "augmentLTI.h"

Eigen::MatrixXf augmentA(Eigen::MatrixXf &Ad, Eigen::MatrixXf &Bd)
{
    Eigen::MatrixXf At = Eigen::Matrix<float, 5, 5>::Zero();
    At.block(0, 0, 4, 4) = Ad;
    At.block(0, 4, 4, 1) = Bd;
    At.block(4, 0, 1, 4) = Eigen::Matrix<float, 1, 4>::Zero();
    At.block(4, 4, 1, 1) = Eigen::Matrix<float, 1, 1>::Identity(); // or identity matrix
    return At;
};

Eigen::MatrixXf augmentB(Eigen::MatrixXf &Bd)
{
    Eigen::MatrixXf Bt = Eigen::Matrix<float, 5, 1>::Zero();
    Bt.block(0, 0, 4, 1) = Bd;
    Bt.block(4, 0, 1, 1) = Eigen::Matrix<float, 1, 1>::Identity(); // or identity matrix
    return Bt;
};

Eigen::MatrixXf augmentC(Eigen::MatrixXf &Cd)
{
    Eigen::MatrixXf Ct = Eigen::Matrix<float, 2, 5>::Zero();
    Ct.block(0, 0, 2, 4) = Cd;
    Ct.block(0, 4, 2, 1) = Eigen::Matrix<float, 2, 1>::Zero();
    return Ct;
}