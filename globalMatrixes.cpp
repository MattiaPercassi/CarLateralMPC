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

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> buildGlobalAdyn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &At, int n)
{
    int sc = At.rows();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Ag;
    for (int i{0}; i < n; ++i)
    {
        auto temp = At;
        for (int j{0}; j < i; ++j)
        {
            temp *= temp;
        }
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
};

template <int n, int sc, int ec>
Eigen::Matrix<float, n * sc, n * sc> buildGlobalQ(Eigen::Matrix<float, ec, ec> &Q, Eigen::Matrix<float, ec, ec> &S, Eigen::Matrix<float, ec, sc> &Ct)
{
    Eigen::Matrix<float, n * sc, n *sc> Qg = Eigen::Matrix<float, n * sc, n * sc>::Zero();
    Eigen::Matrix<float, sc, sc> temp = Ct.transpose() * Q * Ct;
    for (int i{0}; i < n; ++i)
    {
        if (i == n - 1)
            Qg.block(i * sc, i * sc, sc, sc) = Ct.transpose() * S * Ct;
        else
        {
            Qg.block(i * sc, i * sc, sc, sc) = temp;
        }
    }
    return Qg;
};

template <int n, int sc, int ec>
Eigen::Matrix<float, n * ec, n * sc> buildGlobalT(Eigen::Matrix<float, ec, ec> &Q, Eigen::Matrix<float, ec, ec> &S, Eigen::Matrix<float, ec, sc> &Ct)
{
    Eigen::Matrix<float, n * ec, n *sc> Tg = Eigen::Matrix<float, n * ec, n * sc>::Zero();
    Eigen::Matrix<float, ec, sc> temp = Q * Ct;
    for (int i{0}; i < n; ++i)
    {
        if (i == n - 1)
            Tg.block(i * ec, i * sc, ec, sc) = S * Ct;
        else
        {
            Tg.block(i * ec, i * sc, ec, sc) = temp;
        }
    }
    return Tg;
};

template <int n, int uc>
Eigen::Matrix<float, n * uc, n * uc> buildGlobalR(Eigen::Matrix<float, uc, uc> &R)
{
    Eigen::Matrix<float, n * uc, n *uc> Rg = Eigen::Matrix<float, n * uc, n * uc>::Zero();
    for (int i{0}; i < n; ++i)
    {
        Rg.block(i * uc, i * uc, uc, uc) = R;
    }
    return Rg;
};