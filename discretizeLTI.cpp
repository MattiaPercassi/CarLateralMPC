#include <eigen/Eigen/Dense>
#include "discretizeLTI.h"

Eigen::Matrix<float, 4, 4> discretizeA(Eigen::Matrix<float, 4, 4> &A, float h)
{
    Eigen::Matrix<float, 4, 4> Ad = Eigen::Matrix<float, 4, 4>::Identity();
    Ad += A * h;
    return Ad;
};

Eigen::Matrix<float, 4, 1> discretizeB(Eigen::Matrix<float, 4, 1> &B, float h)
{
    Eigen::Matrix<float, 4, 1> Bd = B * h;
    return Bd;
};