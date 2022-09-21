#include <eigen/Eigen/Dense>
#include "globalMatrixes.h"

template <int n, int sc>
Eigen::Matrix<float, n * sc, sc> buildGlobalA(Eigen::Matrix<float, sc, sc> &At)
{
    Eigen::Matrix<float, n * sc, sc> Ag = Eigen::Matrix<float, n * sc, sc>::Zero();
    for (int i{0}; i < n; ++i)
    {
        Eigen::Matrix<float, sc, sc> temp = At;
        for (int j{0}; j < i; ++j)
        {
            temp *= temp;
        };
        Ag.block(i * sc, 0, sc, sc) = temp;
    }
    return Ag;
};

template <int n, int sc, int uc>
Eigen::Matrix<float, n * sc, n * uc> buildGlobalB(Eigen::Matrix<float, sc, sc> &At, Eigen::Matrix<float, sc, uc> &Bt)
{
    Eigen::Matrix<float, n * sc, n *uc> Bg = Eigen::Matrix<float, n * sc, n * uc>::Zero();
    // build longest matrix of the "first column" of Bg
    Eigen::Matrix<float, n * sc, uc> Bgcol;
    for (int i{0}; i < n; ++i)
    {
        Eigen::Matrix<float, sc, uc> temp = Bt;
        for (int j{0}; j < i; ++j)
        {
            temp = At * temp;
        };
        Bgcol.block(i * sc, 0, sc, uc) = temp;
    };

    // build each column of the Bg matrix
    for (int i{0}; i < n; ++i)
    {
        Bg.block(i * sc, i * uc, (n - i) * sc, uc) = Bgcol.block(0, 0, (n - i) * sc, uc);
    };

    return Bg;
}