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
    constexpr int n = 20;       // prediction steps
    constexpr int ec = 2;       // error vector size
    constexpr int uc = 1;       // number of inputs
    constexpr int sc = 5;       // number of states
    constexpr int frac = 20;    // number of fractions for fine status calculation at each step
    constexpr float h = 0.1;    // [s] integration interval, 1/sample frequency
    constexpr int simTime = 10; // [s] total time of simulation
    constexpr float m = 0;      //[kg] vehicle mass
    constexpr float j = 0;      //[kg*m^2] moment if inertia
    constexpr float caf = 0;    // [N] front wheel lateral force coefficient
    constexpr float car = 0;    // [N] rear wheel lateral force coefficient
    constexpr float lf = 0;     // [m] distance of center of mass and front wheel
    constexpr float lr = 0;     // [m] distance of center of mass and rear wheel
    constexpr float xd = 0;     // [m/s] longitudianl velocity
    // initial states vector
    Eigen::Matrix<float, sc, 1> xk = Eigen::Matrix<float, sc, 1>::Zero();
    // initialization input vector
    Eigen::Matrix<float, n * uc, 1> ug = Eigen::Matrix<float, n * uc, 1>::Zero();
    // initialization reference
    Eigen::Matrix<float, ec, static_cast<int>(simTime / h)> ref = Eigen::Matrix<float, ec, static_cast<int>(simTime / h)>::Zero();
    // build matrix A
    Eigen::Matrix<float, 4, 4> A = Eigen::Matrix<float, 4, 4>::Zero();
    // build matrix B
    Eigen::Matrix<float, 4, 1> B = Eigen::Matrix<float, 4, 1>::Zero();
    // build matrix B
    Eigen::Matrix<float, 2, 4> C = Eigen::Matrix<float, 2, 4>::Zero();

    // discretize
    auto Ad = discretizeA(A, h);
    auto Bd = discretizeB(B, h);
    auto Cd = C;

    // augment
    auto At = augmentA(Ad, Bd);
    auto Bt = augmentB(Bd);
    auto Ct = augmentC(Cd);

    // global matrixes
    Eigen::Matrix<float, n * sc, sc> Ag = buildGlobalA<n, sc>(At);

    return 0;
};