#ifndef _AUGMENTLTI_H_
#define _AUGMENTLTI_H_

#include <eigen/Eigen/Dense>

/// @brief Creates the At (tilde) matrix of the augmented system
/// @param Ad Matrix of the discrete LTI system
/// @param Bd Matrix of the discrete LTI system
/// @return Matrix At of the augmented discrete system
Eigen::Matrix<float, 5, 5> augmentA(Eigen::Matrix<float, 4, 4> &Ad, Eigen::Matrix<float, 4, 1> &Bd);

/// @brief Creates the Bt matrix of the augmented system
/// @param Bd Matrix of the discrete LTI system
/// @return Matrix Bt of the augmented discrete system
Eigen::Matrix<float, 5, 1> augmentB(Eigen::Matrix<float, 4, 1> &Bd);

/// @brief Creates the Ct output matrix of the augmented system
/// @param Cd Matrix of the discrete LTI system
/// @return Matrix Ct of the augmented discrete system
Eigen::Matrix<float, 2, 5> augmentC(Eigen::Matrix<float, 2, 4> &Cd);

#endif