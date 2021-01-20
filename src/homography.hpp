//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#ifndef homography_hpp
#define homography_hpp

#include <stdio.h>
#include <Eigen/Geometry>

void homography(Eigen::Vector2f leftIn, Eigen::Vector2f rightIn, Eigen::MatrixXf picIn, Eigen::Vector2f leftOut, Eigen::Vector2f rightOut, Eigen::MatrixXf picOut);

#endif /* homography_hpp */
