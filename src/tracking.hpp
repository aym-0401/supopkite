//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#ifndef tracking_hpp
#define tracking_hpp

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

using namespace cv;
using namespace std;

extern Mat addMatrices(Mat, Mat);
extern int indexContour(std::vector<std::vector<Point>>, double, double);
extern Mat padding(Mat, int, int, Size);
extern Mat contoursConvex(Mat, vector<vector<Point>>&, vector<vector<Point>>&, int, bool);
extern int indexClosestArea(double, vector<vector<Point>>);
extern void CheckParameters(int&, int&, int&, int&, Size);
extern vector<int> SetWidthAndHeight(double&, double, Rect, int);
extern Mat InfoFrame(int, Point2f&, Mat, Rect&, Rect, double&, double, bool&, int, int, int, bool, bool, int&, vector<Point2f>&);
extern void newFunction(std::vector<std::vector<Point>>, double, double, double, int&, Moments&);

extern vector<Point2f> padPoints(Point2f*, Rect);
extern float dist(Point2f, Point2f);

extern int tracking(char* const filepath);

#endif /* tracking_hpp */

