#include <eigen/Eigen/Dense>
#include <iostream>
#include <fstream>
#include "discretizeLTI.h"
#include "augmentLTI.h"
#include "globalMatrixes.h"
#include "gradientMatrixes.h"
#include "solveOpt.h"

int main()
{
    // For simplicity matrixes dimentions are defined in initialization
    constexpr int n = 20;              // prediction steps
    constexpr int ec = 2;              // error vector size
    constexpr int uc = 1;              // number of inputs
    constexpr int sc = 5;              // number of states
    constexpr int frac = 20;           // number of fractions for fine status calculation at each step
    constexpr float h = 0.1;           // [s] integration interval, 1/sample frequency
    constexpr float smallh = h / frac; // [s] integration interval of the open loop system simulation
    constexpr int simTime = 10;        // [s] total time of simulation
    constexpr float m = 0;             //[kg] vehicle mass
    constexpr float j = 0;             //[kg*m^2] moment if inertia
    constexpr float caf = 0;           // [N] front wheel lateral force coefficient
    constexpr float car = 0;           // [N] rear wheel lateral force coefficient
    constexpr float lf = 0;            // [m] distance of center of mass and front wheel
    constexpr float lr = 0;            // [m] distance of center of mass and rear wheel
    constexpr float xd = 0;            // [m/s] longitudianl velocity
    // initial states vector
    Eigen::MatrixXf xk = Eigen::MatrixXf::Zero(sc, 1);
    // initialization input vector
    Eigen::MatrixXf ug = Eigen::MatrixXf::Zero(n * uc, 1);
    // initialization reference
    Eigen::MatrixXf ref = Eigen::MatrixXf::Zero(ec, simTime / h);
    // build matrix A
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(4, 4);
    // build matrix B
    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(4, 1);
    // build matrix C
    Eigen::MatrixXf C = Eigen::MatrixXf::Zero(2, 4);
    // Build matrix Q
    Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(2, 2);
    // Build matrix S
    Eigen::MatrixXf S = Eigen::MatrixXf::Zero(2, 2);
    // Build matrix R
    Eigen::MatrixXf R = Eigen::MatrixXf::Zero(n * uc, uc);

    // discretize
    auto Ad = discretizeA(A, h);
    auto Bd = discretizeB(B, h);
    auto Cd = C;

    // augment
    auto At = augmentA(Ad, Bd);
    auto Bt = augmentB(Bd);
    auto Ct = augmentC(Cd);

    // global matrixes
    auto Ag = buildGlobalAdyn(At, n);
    auto Bg = buildGlobalBdyn(At, Bt, n, sc, uc);
    auto Qg = buildGlobalQdyn(Q, S, Ct, n, sc, ec);
    auto Tg = buildGlobalTdyn(Q, S, Ct, n, sc, ec);
    auto Rg = buildGlobalRdyn(R, n, uc);

    // gradient matrixes
    auto invH = buildinverseHdyn(Bg, Qg, Rg);
    auto F = buildFdyn(Ag, Qg, Bg, Tg, n, sc, ec, uc);

    // steering angle
    float delta{0};

    for (float t{0}; t < simTime / h; t += h)
    {
        ug = calculateOptInputsdyn(invH, F, xk, refg, n, sc, ec);
        delta += ug.block(0, 0, uc, 1);
        for (int j{0}; j < frac; ++j)
        {
            // calculate the open loop system status until the next time step
        };
        // write to file
    };

    return 0;
};