#include <eigen/Eigen/Dense>
#include "augmentLTI.h"

Eigen::Matrix<float, 5, 5> augmentA(Eigen::Matrix<float, 4, 4> &Ad, Eigen::Matrix<float, 4, 1> &Bd)
{
    Eigen::Matrix<float, 5, 5> At = Eigen::Matrix<float, 5, 5>::Zero();
    At.block(0, 0, 4, 4) = Ad;
    At.block(0, 4, 4, 1) = Bd;
    At.block(4, 0, 1, 4) = Eigen::Matrix<float, 1, 4>::Zero();
    At.block(4, 4, 1, 1) = Eigen::Matrix<float, 1, 1>::Identity(); // or identity matrix
    return At;
};

Eigen::Matrix<float, 5, 1> augmentB(Eigen::Matrix<float, 4, 1> &Bd)
{
    Eigen::Matrix<float, 5, 1> Bt = Eigen::Matrix<float, 5, 1>::Zero();
    Bt.block(0, 0, 4, 1) = Bd;
    Bt.block(4, 0, 1, 1) = Eigen::Matrix<float, 1, 1>::Identity(); // or identity matrix
    return Bt;
};

Eigen::Matrix<float, 2, 5> augmentC(Eigen::Matrix<float, 2, 4> &Cd)
{
    Eigen::Matrix<float, 2, 5> Ct = Eigen::Matrix<float, 2, 5>::Zero();
    Ct.block(0, 0, 2, 4) = Cd;
    Ct.block(0, 4, 2, 1) = Eigen::Matrix<float, 2, 1>::Zero();
    return Ct;
}