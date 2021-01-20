//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#ifndef point_projection_hpp
#define point_projection_hpp

#include <vector>
#include <iostream>
#include <math.h>

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <Eigen/Dense>

Eigen::Vector2f project_point_on_cam(Eigen::Vector3f camera_location, Eigen::Vector3f camera_direction, Eigen::Vector3f camera_right, Eigen::Vector3f camera_up, const double camera_width, const double camera_height, const double camera_scale, Eigen::Vector3f origin, Eigen::Vector3f point);

#endif /* point_projection.hpp */
