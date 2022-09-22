#ifndef _AUGMENTLTI_H_
#define _AUGMENTLTI_H_

#include <eigen/Eigen/Dense>

/// @brief Creates the At (tilde) matrix of the augmented system
/// @param Ad Matrix of the discrete LTI system
/// @param Bd Matrix of the discrete LTI system
/// @return Matrix At of the augmented discrete system
Eigen::MatrixXf augmentA(Eigen::MatrixXf &Ad, Eigen::MatrixXf &Bd);

/// @brief Creates the Bt matrix of the augmented system
/// @param Bd Matrix of the discrete LTI system
/// @return Matrix Bt of the augmented discrete system
Eigen::MatrixXf augmentB(Eigen::MatrixXf &Bd);

/// @brief Creates the Ct output matrix of the augmented system
/// @param Cd Matrix of the discrete LTI system
/// @return Matrix Ct of the augmented discrete system
Eigen::MatrixXf augmentC(Eigen::MatrixXf &Cd);

#endif