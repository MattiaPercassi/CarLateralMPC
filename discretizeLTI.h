#ifndef _DISCRETIZELTI_H_
#define _DISCRETIZELTI_H_

#include <eigen/Eigen/Dense>

/// @brief Build the discretized Ad matrix from the system state space A matrix
/// @param A State space matrix of the continuous system (LTI)
/// @param h integration interval
/// @return Ad matrix of the disctete LTI
Eigen::MatrixXf discretizeA(Eigen::MatrixXf &A, float h);

/// @brief Build the discretized Bd matrix from the system state space B matrix
/// @param B State space matrix B of the continuous system (LTI)
/// @param h integration interval
/// @return Bd matrix of the discrete LTI
Eigen::MatrixXf discretizeB(Eigen::MatrixXf &B, float h);

#endif