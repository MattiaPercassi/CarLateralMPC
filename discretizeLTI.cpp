#include <eigen/Eigen/Dense>
#include "discretizeLTI.h"

Eigen::MatrixXf discretizeA(Eigen::MatrixXf &A, float h)
{
    Eigen::MatrixXf Ad = Eigen::Matrix<float, 4, 4>::Identity();
    Ad += A * h;
    return Ad;
};

Eigen::MatrixXf discretizeB(Eigen::MatrixXf &B, float h)
{
    Eigen::MatrixXf Bd = B * h;
    return Bd;
};