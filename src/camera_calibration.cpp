#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// Camera calibration
// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{ 6,8 };

void camera_calibration(char* const filepath) {
    // Creating vector to store vectors of 3D points for each checkerboard image
    vector<vector<Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    vector<Point3f> objp;
    for (int i = 0; i < CHECKERBOARD[1]; i++)
    {
        for (int j = 0; j < CHECKERBOARD[0]; j++)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;

    // Path of the folder containing checkerboard images
    string path = "E:\\IOGS\\3A\\Projet\\Photos_Videos\\Calibration\\gopro";

    glob(path, images);
    Mat frame, gray;

    // vector to store the pixel coordinates of detected checker board corners
    vector<Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for (int i = 0; i < images.size(); i++)
    {
        cout << "Je cherche..." << endl;
        frame = imread(images[i]);
        cout << "ok " << i << endl;
        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        cout << "success ?" << endl;
        success = findChessboardCorners(gray, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
        cout << "success !" << endl;
        // If desired number of corner are detected,
        // we refine the pixel coordinates and display

        if (success)
        {
            TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.001);
            cout << "ChessBoard trouve !" << endl;
            // refining pixel coordinates for given 2d points.
            cornerSubPix(gray, corner_pts, Size(11, 11), Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            drawChessboardCorners(frame, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        else
        {
            cout << "ChessBoard NON trouve !" << endl;
        }
        resize(frame, frame, Size(frame.cols / 4, frame.rows / 4));
        imshow("Image", frame);
        waitKey(0);
    }
    destroyAllWindows();
    Mat cameraMatrix, distCoeffs, R, T;
    
    // Performing camera calibration by
    // passing the value of known 3D points (objpoints)
    // and corresponding pixel coordinates of the
    // detected corners (imgpoints)

    calibrateCamera(objpoints, imgpoints, Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    cout << "cameraMatrix : " << endl;
    cout << cameraMatrix << std::endl;
    cout << "distCoeffs : " << endl;
    cout << distCoeffs << std::endl;
    //cout << "Rotation vector : " << R << std::endl;
    //cout << "Translation vector : " << T << std::endl;

}
