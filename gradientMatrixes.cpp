#include <eigen/Eigen/Dense>
#include "gradientMatrixes.h"

template <int n, int sc, int uc>
Eigen::Matrix<float, n * uc, n * uc> buildinverseH(Eigen::Matrix<float, n * sc, n * uc> &Bg, Eigen::Matrix<float, n * sc, n * sc> &Qg, Eigen::Matrix<float, n * uc, n * uc> &Rg)
{
    Eigen::Matrix<float, n * uc, n *uc> H = Bg.transpose() * Qg * Bg + Rg;
    return H.inverse();
};

Eigen::MatrixXf buildinverseHdyn(Eigen::MatrixXf &Bg, Eigen::MatrixXf &Qg, Eigen::MatrixXf &Rg)
{
    auto H = Bg.transpose() * Qg * Bg + Rg;
    return H.inverse();
};

template <int n, int sc, int ec, int uc>
Eigen::Matrix<float, n * uc, sc + n * ec> buildF(Eigen::Matrix<float, n * sc, sc> &Ag, Eigen::Matrix<float, n * sc, n * sc> &Qg, Eigen::Matrix<float, n * sc, n * uc> &Bg, Eigen::Matrix<float, n * ec, n * sc> &Tg)
{
    Eigen::Matrix<float, sc + n * ec, n *uc> Ft = Eigen::Matrix<float, sc + n * ec, n * uc>::Zero();
    Ft.block(0, 0, sc, n * uc) = Ag.transpose() * Qg * Bg;
    Ft.block(sc, 0, n * ec, n * uc) = -Tg * Bg;
    return Ft.transpose();
};

Eigen::MatrixXf buildFdyn(Eigen::MatrixXf &Ag, Eigen::MatrixXf &Qg, Eigen::MatrixXf &Bg, Eigen::MatrixXf &Tg, int n, int sc, int ec, int uc)
{
    Eigen::MatrixXf Ft = Eigen::MatrixXf::Zero(sc + n * ec, n * uc);
    Ft.block(0, 0, sc, n * uc) = Ag.transpose() * Qg * Bg;
    Ft.block(sc, 0, n * ec, n * uc) = -Tg * Bg;
    return Ft.transpose();
};