#include <eigen/Eigen/Dense>
#include <iostream>
#include <fstream>
#include "discretizeLTI.h"
#include "augmentLTI.h"
#include "globalMatrixes.h"
#include "gradientMatrixes.h"
#include "solveOpt.h"
#include "openLoop.h"

int main()
{
    // pi
    constexpr double pi = 3.1415;
    // For simplicity matrixes dimentions are defined in initialization
    constexpr int n = 20; // prediction steps
    constexpr int ec = 2; // error vector size
    constexpr int uc = 1; // number of inputs
    constexpr int sc = 5; // number of states
    // constexpr int frac = 20;  // number of fractions for fine status calculation at each step
    constexpr double h = 0.02; // [s] integration interval, 1/sample frequency
    // constexpr double smallh = h / frac; // [s] integration interval of the open loop system simulation
    constexpr int simTime = 10;   // [s] total time of simulation
    constexpr double m = 1500;    //[kg] vehicle mass
    constexpr double j = 3000;    //[kg*m^2] moment if inertia
    constexpr double caf = 19000; // [N] front wheel lateral force coefficient
    constexpr double car = 33000; // [N] rear wheel lateral force coefficient
    constexpr double lf = 2;      // [m] distance of center of mass and front wheel
    constexpr double lr = 3;      // [m] distance of center of mass and rear wheel
    constexpr double xd = 20;     // [m/s] longitudianl velocity
    // initial states vector
    Eigen::MatrixXd xk = Eigen::MatrixXd::Zero(sc, 1);
    // vector of the system outputs
    Eigen::MatrixXd yk = Eigen::MatrixXd::Zero(ec, 1);
    // initialization input vector
    Eigen::MatrixXd ug = Eigen::MatrixXd::Zero(n * uc, 1);
    // initialization reference, reference vector is extended after hte ned of the simulation time samples for allowing definiton of refg until the last simulation step
    Eigen::MatrixXd ref = Eigen::MatrixXd::Zero(ec * n + ec * (simTime / h), 1);
    Eigen::MatrixXd refg = Eigen::MatrixXd::Zero(n * ec, 1);

    // straight trajectory at Y = 10
    for (int i{0}; i < ref.rows(); ++i)
    {
        if (i & 1)
            ref(i, 0) = 1;
    };
    // build matrix A
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
    A << (-2 * lf * caf / xd + 2 * car * lr / xd) / j, (-2 * caf * lf * lf / xd - 2 * car * lr * lr / xd) / j, 0, 0, (-2 * caf / xd - 2 * car / xd) / m, (-m * xd - 2 * caf * lf / xd + 2 * car * lr / xd) / m, 0, 0, 1, 0, 0, 0, 0, 1, xd, 0;
    // build matrix B
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1);
    B << 2 * caf * lf / j, 2 * caf / m, 0, 0;
    // build matrix C
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(2, 4);
    C << 0, 0, 1, 0, 0, 0, 0, 1;
    // Build matrix Q
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2, 2);
    Q << 1, 0, 0, 1;
    // Build matrix S
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(2, 2);
    S << 1, 0, 0, 1;
    // Build matrix R
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(uc, uc);
    R << 1;
    std::cout << "A(" << A.rows() << "," << A.cols() << ") " << '\n'
              << A << std::endl;
    std::cout << "B(" << B.rows() << "," << B.cols() << ") " << '\n'
              << B << std::endl;

    // discretize
    auto Ad = discretizeA(A, h);
    auto Bd = discretizeB(B, h);
    auto Cd = C;
    std::cout << "Ad(" << Ad.rows() << "," << Ad.cols() << ") " << '\n'
              << Ad << std::endl;

    // augment
    auto At = augmentA(Ad, Bd);
    auto Bt = augmentB(Bd);
    auto Ct = augmentC(Cd);
    std::cout << "At(" << At.rows() << "," << At.cols() << ") " << '\n'
              << At << std::endl;

    // global matrixes
    auto Ag = buildGlobalAdyn(At, n);
    auto Bg = buildGlobalBdyn(At, Bt, n, sc, uc);
    auto Qg = buildGlobalQdyn(Q, S, Ct, n, sc, ec);
    auto Tg = buildGlobalTdyn(Q, S, Ct, n, sc, ec);
    auto Rg = buildGlobalRdyn(R, n, uc);
    std::cout << "Ag(" << Ag.rows() << "," << Ag.cols() << ") " << '\n'
              << Ag << std::endl;
    std::cout << "Bg(" << Bg.rows() << "," << Bg.cols() << ") " << '\n'
              << Bg << std::endl;
    std::cout << "Qg(" << Qg.rows() << "," << Qg.cols() << ") " << '\n'
              << Qg << std::endl;
    std::cout << "Rg(" << Rg.rows() << "," << Rg.cols() << ") " << '\n'
              << Rg << std::endl;
    std::cout << "Tg(" << Tg.rows() << "," << Tg.cols() << ") " << '\n'
              << Tg << std::endl;

    // gradient matrixes
    auto invH = buildinverseHdyn(Bg, Qg, Rg);
    auto F = buildFdyn(Ag, Qg, Bg, Tg, n, sc, ec, uc);
    std::cout << "invH(" << invH.rows() << "," << invH.cols() << ") " << '\n'
              << invH << std::endl;
    std::cout << "F(" << F.rows() << "," << F.cols() << ") " << '\n'
              << F << std::endl;

    // steering angle
    double delta{0};

    for (int i{0}; i < simTime / h; ++i)
    {
        refg = ref.block(i * ec, 0, n * ec, 1);
        Eigen::MatrixXd error = yk - ref.block(i * ec, 0, ec, 1);
        std::cout << "Error: " << error.transpose() << std::endl;
        ug = calculateOptInputsdyn(invH, F, xk, refg, n, sc, ec);
        std::cout << "Iteration " << i << ": " << std::endl;
        std::cout << "States (psid, yd, psi, Y, uk-1): " << xk.transpose() << std::endl;
        std::cout << "Optimal control inputs (Ddelta): " << ug.transpose() << std::endl;
        // should I apply a control here on the Ddelta??
        delta += ug(0, 0);
        // delta must be constrained between the real available steering angles +- pi/6
        if (delta < -pi / 6)
            delta = -pi / 6;
        else if (delta > pi / 6)
            delta = pi / 6;

        xk(4, 0) = delta;

        // For test purposes update the states with same refinement as controller sample frequency
        xk.block(0, 0, 4, 1) = Ad * xk.block(0, 0, 4, 1) + Bd * delta;
        yk = Ct * xk;
        std::cout << "Outputs (psi, Y): " << yk.transpose() << std::endl;
        // write to file
    };

    return 0;
};