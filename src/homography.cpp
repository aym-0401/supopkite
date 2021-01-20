//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#include <vector>
#include <iostream>
#include <math.h>

#include <Eigen/Geometry>
#include "homography.hpp"

using namespace std;
using namespace Eigen;

void homography(Vector2f leftIn, Vector2f rightIn, MatrixXf picIn, Vector2f leftOut, Vector2f rightOut, MatrixXf picOut);

void homography(Vector2f leftIn, Vector2f rightIn, MatrixXf picIn, Vector2f leftOut, Vector2f rightOut, MatrixXf picOut) {
    
    Matrix3f translMat;
    Matrix3f rotMat;
    Matrix3f scalMat;
    Matrix3f backTranslMat;
    
    float angle;
    angle = atan( (rightOut.y() - leftOut.y()) / (rightOut.x() - leftOut.x()) ) - atan( (rightIn.y() - leftIn.y()) / (rightIn.x() - leftIn.x()) );
    angle = fmod(angle, 2*M_PI);
    
    float scale;
    scale = ((rightIn.y() - leftIn.y())*(rightIn.y() - leftIn.y()) + (rightIn.x() - leftIn.x())*(rightIn.x() - leftIn.x())) / ((rightOut.y() - leftOut.y() )*(rightOut.y() - leftOut.y() ) + (rightOut.x() - leftOut.x())*(rightOut.x() - leftOut.x()));
    
    scale = sqrt(scale);
    
    translMat << 1, 0, leftIn.x(), 0, 1, leftIn.y(), 0, 0, 1;
    rotMat << cos(angle), sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, 1;
    scalMat << scale, 0, 0, 0, scale, 0, 0, 0, 1;
    backTranslMat << 1, 0, -leftOut.x(), 0, 1, -leftOut.y(), 0, 0, 1;
    
    Vector3f vec;
    float width = (float) picIn.cols();
    float height = (float) picIn.rows();
    
    Matrix3f mat;
    mat = translMat * rotMat * scalMat * backTranslMat;
    
    for (int i = 0; i < picOut.cols(); i++) {
        for (int j = 0; j < picOut.rows(); j++) {
            vec << i,j,1;
            vec = mat * vec;
 
            if (vec.x() > 0 && vec.x() < width && vec.y() > 0 && vec.y() < height) {
                picOut(i,j) = picIn( (int) vec.y(), (int) vec.x());
            } else {
                picOut(i,j) = 0;
                
            }
        }
    }
}


