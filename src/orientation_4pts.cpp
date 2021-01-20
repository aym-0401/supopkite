//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//
// orientation_4pts.cpp
//
// Created by François Ollitrault
//
// but du programme : acquérir une vidéo de test et calculer l'orientation du repère composé des 4 points posés sur la voile
//
// Étapes:
// 1. acquérir la vidéo ok
// 2. la traiter pour isoler, mettre en valeur les points de couleur ok
// 3. tracker les 4 points ok
// 4. Identifier les points ok
// 5. calculer l'orientation du repère par reconstruction ok
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp> //notamment pour Rect
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <vector>
#include <iostream> 
#include <Eigen/Core>
#include <Eigen/Dense>
//#include <Eigen/IterativeLinearSolvers>

using namespace std;
using namespace Eigen;

// Color to track (H=hue=color info in the HSV basis) from 0 to 360°
#define MIN1_H_RED 300
#define MAX1_H_RED 360
#define MIN2_H_RED 0
#define MAX2_H_RED 25
#define pi 3.141592

void orientation_4pts(char* const filepath) {
    //Get build information
    std::cout << cv::getBuildInformation();

    // Camera frame
    cv::Mat frame_nsat;
    cv::Mat frame;
    
    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    
    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
    
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);
    
    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;
    
    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;
    
    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    //lignes = coordonnées des points dans le repère Kite
    MatrixXd Pk(4,3);
    Pk(0,0) = 1.86;
    Pk(0,1) = 0;
    Pk(0,2) = 0;
    Pk(1,0) = 1.01;
    Pk(1,1) = 0;
    Pk(1,2) = 1.12;
    Pk(2,0) = 0;
    Pk(2,1) = -2.35;
    Pk(2,2) = 0;
    Pk(2,0) = 0;
    Pk(2,1) = 2.35;
    Pk(2,2) = 0;

    // Video Capture
    cv::VideoCapture cap(filepath);
    
    cout << "\nHit 'q' to exit...\n";
    
    char ch = 0;
    double ticks = 0;
    bool found = false;
    int notFoundCount = 0;
    
    // >>>>> Main loop
    while (ch != 'q' && ch != 'Q')
    {
        double precTick = ticks;
        ticks = (double) cv::getTickCount();
        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // Frame acquisition
        cap >> frame_nsat; //non saturée

        int height = frame_nsat.rows;
        int width = frame_nsat.cols;

		if (width == 0){ //message de fin de vidéo
			cout << "end of video" << endl;
            break;
        }

        cout << "lignes :" << height << " colonnes : " << width << endl;

        double saturation = 5;
        double scale = 3;
        // dst = (uchar) ((double)src*scale+saturation);
        frame_nsat.convertTo(frame, CV_8UC1, scale, saturation);

        cv::Mat res;
        frame.copyTo( res );
        
        //Etape de prédiction Kalman
        if (found) //Au moins un patch est détecté
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A
            
            //Calcul de l'état prédit, à partir de l'état actuel du filtre
            state = kf.predict();
            
            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;
            
            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);

            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);
            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
        }
        
        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
        // <<<<< Noise smoothing
        
        // >>>>> HSV conversion
        cv::Mat frmHsv;
        cv::cvtColor(blur, frmHsv, cv::COLOR_BGR2HSV);
        // <<<<< HSV conversion
        
        // >>>>> Color Thresholding
        // definition des deux intervalles de teinte
        cv::Mat rangeRes1, rangeRes2 = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::inRange(frmHsv, cv::Scalar(MIN1_H_RED * 179/360, 51, 51), 
                cv::Scalar(MAX1_H_RED * 179/360, 255, 255), rangeRes1);
        cv::inRange(frmHsv, cv::Scalar(MIN2_H_RED * 179/360, 51, 51), 
                cv::Scalar(MAX2_H_RED * 179/360, 255, 255), rangeRes2);
        // <<<<< Color Thresholding
        
        //union des 2 intervalles
        cv::Mat1b rangeRes = rangeRes1 | rangeRes2;

        // >>>>> Improving the result
        cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
        // <<<<< Improving the result
        
        // Thresholding viewing
        cv::imshow("Threshold", rangeRes);
        
        // >>>>> Contours detection
        vector<vector<cv::Point> > all_contours;
        cv::findContours(rangeRes, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // <<<<< Contours detection
        
        cout << "all_contours.size() = " << all_contours.size() << endl;

        // >>>>> Filtering
        //vector<vector<cv::Point> > all_contours; //initialement balls //ou all_balls
        vector<cv::Rect> all_ballsBox; //initialement ballsBox //ou all_ballsBox
        vector<int> areas; //rajout
        for (size_t i = 0; i < all_contours.size(); i++)
        {
            cv::Rect bBox;
            bBox = cv::boundingRect(all_contours[i]);
            
            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;
            
            // Searching for a bBox almost square //initial values ratio > 0.75 and area >= 400
            if (ratio > 0.3 && bBox.area() >= 80)
            {
                //balls.push_back(all_contours[i]); //ou all_balls
                all_ballsBox.push_back(bBox); //ou all_ballsBox
                areas.push_back(bBox.area()); //liste des aires de contours respectant la condition
            }
        }
        // >>>>> Filtering
        

        //affichage des différents éléments :
        //contours contient les coordonnées de tous les points pour chaque contour
        cout << "all_contours.size() = " << all_contours.size() << endl;
        cout << "areas.size() = " << areas.size() << endl;
        for (int i = 0; i < all_contours.size(); ++i){
        //cout << "all_contours(" << i << ") = " << all_contours.at(i) << endl;
        //cout << "all_balls(" << i << ") = " << all_balls.at(i) << endl;
        //cout << "all_ballsBox(" << i << ") = " << all_ballsBox.at(i) << endl;
        //cout << "areas(" << i << ") = " << areas.at(i) << endl;
        }


        // >>>>> Selection des 4 points (contour + rectangle) d'aires maximale
        //selectionner dans balls et ballsBox les 4 éléments d'aire maximale
        vector<vector<cv::Point> > contours;
        vector<cv::Rect> ballsBox;
		vector<int> indexes;
        vector<int> areas2 = areas; //c'est le vector areas sur lequel on enlève petit à 
        //petit les plus grands éléments 
        for (int i = 0; i < 4; ++i) //avant contours.size()
        {
            if (areas.size() == 0)
            {
                break;
            }
            int pgt = 0; //plus grand élément
            int index = 0;
            //recherche du contours de plus grande aire parmi les contours
            //vérifiant la condition sur l'aire
            for(size_t j=0; j < areas.size() ; j++){
                cout << "tour_boucle " << j << endl;
                if(pgt<areas2.at(j)){
                    cout << "tour_boucle2" << endl;
                    pgt=areas2.at(j);
                    index = j;
                }
            }
            //mettre l'élément en question à zéro dans areas 
            areas2.at(index)=0;
            indexes.push_back(index);
            //areas.erase(areas.begin()+index);
            contours.push_back(all_contours.at(index)); //Les 4 contours
            ballsBox.push_back(all_ballsBox.at(index)); //Les 4 bounding boxes
            //classées par aires décroissantes
        }
        //affichage des listes de 4 éléments 
        cout << "ballsBox.size()" << ballsBox.size() << endl;
        cout << "contours.size()" << contours.size() << endl;
        // >>>>> Selection des 4 points


        // >>>>> Detection result : rectangles verts sur l'image
        vector<cv::Point> centers; //coordonnées des points
        for (size_t i = 0; i < all_contours.size(); i++) //avant all_contours.size()
        {
            cout << "ok1" << endl;
            cv::drawContours(res, all_contours, i, CV_RGB(20,150,20), 1);
            cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);
            
            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

            centers.push_back(center);

            cout << "center.x = " << center.x << endl;
            cout << "center.y = " << center.y << endl;
            
            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(res, sstr.str(),
            cv::Point(center.x + 3, center.y - 3),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
            cout << "ok2" << endl;
        }
        // >>>>> Detection result

        // <<<<< identification des points de la voile
        //numérotation : haut P1, arrière P2, droit P3, gauche P4 
        int max_x = 0; 
        int max_y = 0;
        int min_x = width; //1280 //640 //1920
        int min_y = height; //720 //360 //1080
        int i1, i2, i3, i4;
        //indices des points du vecteur balls selon la numérotation 
        vector<int> indiceP; //contient les positions des points P1 P2 P3 P4 respectivement dans centers
        vector<int> numeros = {0,1,2,3};
        //points latéraux
        for (int i = 0; i < 4; i++)
        {   
            cout << "ok3" << endl;
            if (centers.at(i).x < min_x){
                cout << "ok31" << endl;
                min_x = centers.at(i).x;
                i3 = i; //P3 est en position i dans centers
            }
            if (centers.at(i).x > max_x){
                cout << "ok32" << endl;
                max_x = centers.at(i).x;
                i4 = i; //P4 
            }
        }
        cout << "ok3bis" << endl;
        numeros.erase(numeros.begin() + i3); //on retire l'indice de P3
        if(i3>i4){
            numeros.erase(numeros.begin() + i4);
        }
        else{
            numeros.erase(numeros.begin() + i4-1);
        }
        cout << "ok4" << endl;
        //points centraux
        if (centers.at(numeros.at(0)).y < centers.at(numeros.at(1)).y){ //si x1 < x2
            i1 = numeros.at(0); //P1
            i2 = numeros.at(1); //P2
        }
        else {
            i1 = numeros.at(1); //P1
            i2 = numeros.at(0); //P2
        }
        cout << "ok5" << endl;

        // reordonancement
        vector<cv::Point> ord_centers; //contient P1, P2, P3, P4 respectivement 
        ord_centers.push_back(centers.at(i1));
        cout << "ord_centers.at(0)" << ord_centers.at(0) << endl;
        stringstream sstr1;
        sstr1 << "P1";
        cv::putText(res, sstr1.str(), cv::Point(ord_centers.at(0).x, ord_centers.at(0).y - 100),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(150,50,20), 2);

        ord_centers.push_back(centers.at(i2));
        cout << "ord_centers.at(1)" << ord_centers.at(1) << endl;
        stringstream sstr2;
        sstr2 << "P2";
        cv::putText(res, sstr2.str(), cv::Point(ord_centers.at(1).x, ord_centers.at(1).y + 100),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(150,50,20), 2);

        ord_centers.push_back(centers.at(i3));
        cout << "ord_centers.at(2)" << ord_centers.at(2) << endl;
        stringstream sstr3;
        sstr3 << "P3";
        cv::putText(res, sstr3.str(), cv::Point(ord_centers.at(2).x - 100, ord_centers.at(2).y),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(150,50,20), 2);

        ord_centers.push_back(centers.at(i4));
        cout << "ord_centers.at(3)" << ord_centers.at(3) << endl;
        stringstream sstr4;
        sstr4 << "P4";
        cv::putText(res, sstr4.str(), cv::Point(ord_centers.at(3).x + 100, ord_centers.at(3).y),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(150,50,20), 2);

        // >>>>> identification des points de la voile
        
        // <<<<< Calcul de l'orientation
            
        //P = R*t_Pk coordonnées dans le réf camera = rotation * coordonnées dans le ref kite
        //R = [ a, b, c, 
        //      d, e, f, 
        //      g, h, i ]
        //ce qui se réécrit : 
        // [ x1k, y1k, z1k   *   a      x1
        //   x2k, y2k, z2k       b      x2
        //   x3k, y3k, z3k       c      x3
        //   x4k, y4k, z4k ]            x4
        //et 
        // [ x1k, y1k, z1k   *   d      y1
        //   x2k, y2k, z2k       e      y2
        //   x3k, y3k, z3k       f      y3
        //   x4k, y4k, z4k ]            y4

        //Pk << 1.86, 0, 0,
        //      1.01, 0, 1.12,
        //      0, -2.35, 0,
        //      0, 2.35, 0;
        
        //Px et Py coordonnées des points sur l'image relativement au centre du repère
        //xi et yi coordonnées sur l'image
        double x1 = ord_centers.at(0).x;
        double x2 = ord_centers.at(1).x;
        double x3 = ord_centers.at(2).x;
        double x4 = ord_centers.at(3).x;
        double y1 = ord_centers.at(0).y;
        double y2 = ord_centers.at(1).y;
        double y3 = ord_centers.at(2).y;
        double y4 = ord_centers.at(3).y;

        //xc et yc coordonnées sur l'image du centre du repère
        double xc = (x3+x4)/2;
        double yc = (y3+y4)/2;

        //ui et vi coordonnnées sur l'image avec pour origine le centre du repère kite
        double u1 = x1-xc;
        double u2 = x2-xc;
        double u3 = x3-xc;
        double u4 = x4-xc;
        double v1 = yc - y1; //repère opencv : y orienté vers le bas
        double v2 = yc - y2;
        double v3 = yc - y3;
        double v4 = yc - y4;

        VectorXd Px(4);
        Px(0) = u1;
        Px(1) = u2;
        Px(2) = u3;
        Px(3) = u4;
        VectorXd Py(4);
        Py(0) = v1;
        Py(1) = v2;
        Py(2) = v3;
        Py(3) = v4;

        cout << "Px" << Px << "Py" << Py << endl;
        
        //calcul des coefficients de R1 et R2 par moindres carrés
        // min(|| Pk*R1-Px ||^2) <=> t_Pk*Pk*R1 = t_Pk*Px )
        //LeastSquaresConjugateGradient<SparseMatrix<double> > lscg;
        //lscg.compute(Pk);
        VectorXd R1(3);
        VectorXd R2(3);
        R1 = (Pk.transpose() * Pk).ldlt().solve(Pk.transpose() * Px);
        R2 = (Pk.transpose() * Pk).ldlt().solve(Pk.transpose() * Py);

        cout << R1 << endl;
        cout << R2 << endl;
        
        //R1 = lscg.solve(Px);
        //R2 = lscg.solve(Py);

        //normalisation de la matrice de rotation
        //VectorXd Alpha_carre = 0.5 * (R1.transpose()*R1 + R2.transpose()*R2);
        //double alpha_carre = Alpha_carre(0); //( R1[0]*R1[0] + R1[1]*R1[1] + R1[2]*R1[2] + R2[0]*R2[0] + R2[1]*R2[1] + R2[2]*R2[2] )/2;
        //R1 = R1*(1/alpha_carre);
        //R2 = R2*(1/alpha_carre);
        R1 = R1*(1/R1.norm());
        //normalisation de la solution
        R2 = R2*(1/R2.norm());

        //matrice de rotation
        Vector3d R3( sqrt(1 - R1[0]*R1[0] - R2[0]*R2[0]), sqrt(1 - R1[1]*R1[1] - R2[1]*R2[1]), sqrt(1 - R1[2]*R1[2] - R2[2]*R2[2]));
        R3 = R3*(1/R3.norm());
        cout << R1 << endl;
        cout << R2 << endl;
        cout << R3 << endl;

        //conversion en angles d'Euler
        //matrice de rotation
        MatrixXd R(3,3);
        R(0,0) = R1[0];
        R(0,1) = R1[1];
        R(0,2) = R1[2];
        R(1,0) = R2[0];
        R(1,1) = R2[1];
        R(1,2) = R2[2];
        R(2,0) = R3[0];
        R(2,1) = R3[1];
        R(2,2) = R3[2];

        float sx = R(2,1);
        bool singular = sx < 1e-6; // If
        float x, y, z;
        if (!singular)
        {
            x = asin(R(2,1));//x = atan2(R(2,1) , R(2,2));
            y = atan2(-R(2,0),R(2,2));//y = atan2(-R(2,0), sy);
            z = atan2(-R(0,1), R(1,1));
        }
        else
        {
            //x = atan2(-R(1,2), R(1,1));
            //y = atan2(-R(2,0), sy);
            //z = 0;
        }
        Vector3d angles(x,y,z);
        double x_deg = x*180/pi; 
        double y_deg = y*180/pi; 
        double z_deg = z*180/pi; 
        Vector3d angles_degres(x_deg,y_deg,z_deg);
        cout << "angles d'euler : x,y,z precession, nutation, rotation propre" << angles_degres << endl;
        // <<<<< Calcul de l'orientation
        
        // >>>>> Correction Kalman
        if (all_contours.size() == 0) //Si aucun patch n'est détecté
        {
            notFoundCount++;
            if( notFoundCount >= 100 ) //Au delà de 100 frames
            {
                found = false;
            }
        }
        else //Au moins un patch est détecté
        {
            notFoundCount = 0;
            
            //Actualisation des valeurs mesurées 
            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            meas.at<float>(2) = (float)ballsBox[0].width;
            meas.at<float>(3) = (float)ballsBox[0].height;
            
            if (!found) // Première détection
            {
                // >>>> Initialisation
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px
                
                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialisation
                
                //Correction de l'Etat du système 
                kf.statePost = state;

                found = true;
            }
            else
                //Correction des mesures
                kf.correct(meas);
        }
        // <<<<< Correction Kalman
        
        // Final result
        cv::imshow("Tracking", res);

        // User key
        ch = cv::waitKey(1);
    }
    // <<<<< Main loop
    
    cap.release();
    cv::destroyAllWindows();
    std::cout<<"closing now\n";
}
