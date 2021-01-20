//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//
// See https://github.com/glaba/STL-Renderer for the original implementation.

#include <vector>
#include <iostream>
#include <math.h>

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <Eigen/Dense>

#include "point_projection.hpp"

using namespace std;
using namespace Eigen;

Vector2f project_point_on_cam(Vector3f camera_location, Vector3f camera_direction, Vector3f camera_right, Vector3f camera_up, const double camera_width, const double camera_height, const double camera_scale, Vector3f origin, Vector3f point);

Vector2f project_point_on_cam(Vector3f camera_location, Vector3f camera_direction, Vector3f camera_right, Vector3f camera_up, const double camera_width, const double camera_height, const double camera_scale, Vector3f origin, Vector3f point) {
    
    // for now: projection takes into account the perspective ratio
    // TODO:
    // - to remove it, do t = 0?, the model becomes orthographic projection?
    // - add orhographic or perspective option
    
    Vector3f proj;
    proj.z() = 0;

    double t;
    
    t = camera_direction.dot(origin - camera_location) / camera_direction.dot(point - camera_location);

    Vector3f intersection;
    
    intersection <<     camera_location.x() + (point.x() - camera_location.x()) * t,
                        camera_location.y() + (point.y() - camera_location.y()) * t,
                        camera_location.z() + (point.z() - camera_location.z()) * t;

    intersection = intersection - origin;
    proj.x() = camera_right.dot(intersection);
    proj.y() = camera_up.dot(intersection);
    
    Vector2f pix;
    pix << (proj.x() + camera_width / 2) / camera_scale, (-proj.y() + camera_height / 2) / camera_scale;
    
    return pix;
}
