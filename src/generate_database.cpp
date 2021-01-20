//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#include <math.h>
#include <float.h>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include <Eigen/Dense>

#include "generate_database.hpp"
#include "renderer.hpp"
#include "homography.hpp"
#include "point_projection.hpp"

using namespace std;
using namespace Eigen;

void generate_database(char* const filepath) {
    // TODO: translate origin center on sail's .stl model, maybe to the "gravity center" of the 4 strings attachments
    int N = 128;
    
    int width = 64;
    int height = 64;
    double scale = 1.0; // not the scale you think of, don't touch

    float *phi;
    float *theta;
    phi = (float*) malloc(N * sizeof(float));
    theta = (float*) malloc( N * sizeof(float));
    
    for (int i = 0 ; i < N ; i++) {
        theta[i] =  M_PI * (1 + sqrt(5)) * (i+0.5);
        phi[i] = acos(1 - 2*(i+0.5)/N);
        
        theta[i] = fmod(theta[i],2*M_PI);
        phi[i] = fmod(phi[i],2*M_PI);
    }
    
    float x,y,z;
    float d = 45;
    
    MatrixXf output (width, N*height);
    MatrixXf picIn (width, height);
    MatrixXf picOut (width, height);
    
    Vector2f leftIn;
    Vector2f rightIn;
    
    Vector2f leftOut;
    leftOut << 20, 40;
    Vector2f rightOut;
    rightOut << 44, 40;
    
    const double cam_scale = 1.0 / 500.0;
    const double cam_width = width * cam_scale;
    const double cam_height = height * cam_scale;
    
    Vector3f left3D;
    left3D << -1.69207, 1.88813, -2.03978;
    Vector3f right3D;
    right3D << -1.69207, -1.88813, -2.03978;
    
    MatrixXf cam(N,3);
    
    Vector3f tmp;
    Vector3f dir, right, up;
    double right_angle;
   
    for (int i = 0 ; i < N ; i++) {
        x = d * cos(theta[i]) * sin(phi[i]);
        y = d * sin(theta[i]) * sin(phi[i]);
        z = d * cos(phi[i]);
        
        tmp << x,y,z;
        cam.row(i) = tmp;
    }
//    std::cout << cam << std::endl;
    output = renderer(filepath, scale, cam, N, height, width);
    cout<< output << endl;
    
    free(phi);
    free(theta);
}
