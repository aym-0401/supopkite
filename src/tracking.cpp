//
// Created by Aymeric HALÉ, Charlotte MILLET & François OLLITRAULT.
//
// We give our explicit authorization to use, distribute and modify this code.
// Some portions of the code are authorized modifications of other works indicated
// by their copyrights.
//

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>

#include "tracking.hpp"

using namespace cv;
using namespace std;

Mat src_gray;
RNG rng(12345);


int tracking(char* const filepath) {
    
	// Variables à changer pour chaque utilisation (ou presque)
	bool show = true; // Si on veut afficher chacune des frames avec le contour et la fenêtre choisie pour calculer le contour suivant
	int resolution = 720; // Entrer la même résolution que le fichier vidéo
	int frame_debut = 0; // Frame à partir de laquelle on veut faire le tracking

	// Variables à modifier si besoin mais en faisant attention
	int multiple_aire = 8; // L'aire de la fenêtre de recherche du contour sera multiple_aire fois plus grande que l'aire de la voile


	// Ne plus modifier à partir d'ici
	bool resizeFenetre = false;// (resolution > 400);
	int width = (resolution/360)*640, height = resolution; // Taille de la fenêtre de la frame 0, correspond à la frame entière
	double widthm = 0, heightm = 0;
	int x0 = width / 2, y0 = height / 2; // Centre de la fenêtre de la frame 0
	time_t tstart, tend;

	Mat src;
	bool bSuccess;
	Size size = { width, height };
	int counter = 0, count_errors = 0;

	Rect ROI = { x0 - width / 2, y0 - height / 2, width, height };
	Rect prev_ROI = ROI; // fenêtre de la frame n-1
	Rect ROI0 = ROI; // frame entière;
	double prev_aire = 0, aire_cible = 0;
	bool neverFound = true;
	tstart = time(0);
	
	//open the video file for reading
	VideoCapture cap(filepath); // La vidéo s'appelle maintenant cap

	// if not success, exit program
	if (cap.isOpened() == false)
	{
		std::cout << "Cannot open the video file" << endl;
		cin.get(); //wait for any key press
		return -1;
	}

	//get the frames rate of the video
	double fps = cap.get(CAP_PROP_FPS);
	std::cout << "Frames per seconds : " << fps << endl;
	Point2i pos_nm2 = { 0, 0 }, pos_nm1 = { 0, 0 };
	Mat positions = Mat::zeros(size, CV_8UC3);
	vector<Point2f> PointsVoile(2);

	while (true)
		{
			// Capture frame-by-frame
			bSuccess = cap.read(src); // read a new frame from video 

			//Breaking the while loop at the end of the video
			if (bSuccess == false)
			{
				std::cout << "Found the end of the video" << endl;
				tend = time(0);
				imwrite("C:\\Users\\charl\\Downloads\\positions.jpg", positions);
				std::cout << "Le calcul se fait en " << float(counter) / (tend - tstart) << " fps." << endl;
				std::cout << "Il y a eu " << count_errors << " erreurs pour " << counter << " frames." << endl;
				return -1;
			}
			if (counter >= frame_debut)
			{
				if (counter == frame_debut)
				{
					// Travail sur la frame 1					
					Mat first_frame = src;

					// On affiche la premiere frame
					const char* answer_window = "answer";
					cv::namedWindow(answer_window);
					cv::imshow(answer_window, first_frame);

					Rect first_ROI = selectROI(answer_window, first_frame, false);
					aire_cible = first_ROI.width * first_ROI.height;
				}

				Point2f pos;
				Mat fenetre = InfoFrame(counter, pos, src, prev_ROI, ROI, prev_aire, aire_cible, neverFound, width, height, multiple_aire, show, resizeFenetre, count_errors, PointsVoile);
				if (pos.x == 0 && (pos.y == 0))
				{
					Point2i pos_temp = { min(max(2 * pos_nm1.x - pos_nm2.x,0),width), min(max(2 * pos_nm1.y - pos_nm2.y,0),height) };
					pos_nm2 = pos_nm1;
					pos_nm1 = pos_temp;
					std::cout << "Sur la frame " << counter << ", on estime la voile a la position (" << pos_temp.x << ", " << pos_temp.y << ") (en pixels)." << endl;
					circle(positions, pos_temp, 2, Scalar(255, 255, 255), -1); // On dessine un cercle correspondant à la position de la voile.
				}
				else
				{
					pos_nm2 = pos_nm1;
					pos_nm1 = pos;
					std::cout << "Sur la frame " << counter << ", la voile se trouve a la position (" << pos.x << ", " << pos.y << ") (en pixels)." << endl;
					unsigned char b = (unsigned char) (min(max(0, counter - 155), 255));
					unsigned char g = (unsigned char) ((counter >=310) * (min(max(0, 565 - counter), 255)));
					unsigned char r = (unsigned char) (min(max(0, 255 - counter), 255));
					circle(positions, pos, 2, Scalar(b, g, r), -1); // On dessine un cercle correspondant à la position de la voile.
				}	

				// Rajouter du code ICI
				// La position de la voile se récupère en : pos.x et pos.y
				// fenetre est la portion de la frame qui contient la voile.
				// pour récupérer les deux points du rectangle pour l'homographie : PointsVoile[0] est le point inférieur gauche, PointsVoile[1] est le point inférieur droit (dans la frame)
				// pour recuperer leur position dans la fenetre il faut faire : PointsVoile[0 ou 1].x - ROI.x et PointsVoile[0 ou 1].y - ROI.y

				//wait for for 10 ms until any key is pressed.
				//If the 'Esc' key is pressed, break the while loop.
				//If the any other key is pressed, continue the loop
				//If any key is not pressed withing 10 ms, continue the loop
				if (waitKey(10) == 27)
				{
					std::cout << "Esc key is pressed by user. Stopping the video" << endl;
					tend = time(0);
					std::cout << "Le calcul se fait en " << float(counter) / (tend - tstart) << " fps." << endl;
					std::cout << "Il y a eu " << count_errors << " erreurs pour " << counter << " frames." << endl;
					break;
				}
			}

			counter = counter + 1;
		}
	
	return 0;
}

Mat addMatrices(Mat A, Mat B)
// Additionne les matrices A et B
{
	Size size = A.size();
	int rows = size.height;
	int cols = size.width;
	Mat C(rows, cols, CV_8UC3);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			C.at<Vec3b>(i,j) = A.at<Vec3b>(i, j) + B.at<Vec3b>(i, j);
		}
	}
	return C;
}

int indexContour(std::vector<std::vector<Point>> newContours, double aire_src, double aire_fen)
// Renvoie l'indice du contour (du vecteur newContours) ayant la plus grande aire
// Ce contour doit aussi être suffisamment de "petite" aire afin de ne pas être le contour de la fenêtre ou de la frame
{
	int index = 0;
	Moments M = moments(newContours[0]);
	double aire_max = 0;
	bool initialise = false;
	for (int i = 0; i < newContours.size(); i++)
	{
		M = moments(newContours[i]);
		double aire = M.m00;
		if ((!initialise) && (aire < 0.3 * aire_src) && (aire < 0.3 * aire_fen))
		{
			aire_max = aire;
			initialise = true;
			index = i;
		}
		else if ((aire > aire_max) && (aire < 0.3 * aire_src) && (aire < 0.3 * aire_fen))
		{
			index = i;
			aire_max = aire;
		}
	}
	return index;
}

Mat padding(Mat drawing, int x, int y, Size size)
// Redimensionne une matrice en y insérant des zéros de partout ou il y a des cases vides
// On choisit où se positionne les valeurs non nulles dans la matrice finale
{
	Mat recons = Mat::zeros(size, CV_8UC3);
	int width = drawing.size().width;
	int height = drawing.size().height;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			recons.at<Vec3b>(y + i, x + j) = drawing.at<Vec3b>(i, j);
		}
	}
	return recons;
}

Mat contoursConvex(Mat src, vector<vector<Point>> &hull, vector<vector<Point>> &allContours, int counter, bool show)
// Recherche tous les contours présents dans l'image src, puis calcule leur enveloppe convexe
// Cela permet de fermer tous les contours et de détecter la voile même si son contour est divisé en deux à la base
// La matrice de sortie est une matrice vide, de la taille de la matrice d'entrée.
{
	Mat gray, blur_image, threshold_output;
	cvtColor(src, gray, COLOR_BGR2GRAY); // convert to grayscale
	blur(gray, blur_image, Size(3, 3)); // apply blur to grayscaled image
	adaptiveThreshold(blur_image, threshold_output, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);

	vector< vector<Point> > contours; // list of contour points
	vector<Vec4i> hierarchy;
	// find contours
	findContours(threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	allContours = contours;
	// create hull array for convex hull points
	hull.resize(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
	}
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	return drawing;
}

int indexClosestArea(double prev_aire, vector<vector<Point>> hull)
// Renvoie l'indice du contour (du vecteur newContours) ayant l'aire la plus proche de l'aire de la voile à la frame précédente.
{
	int index = 0;
	int size = hull.size();
	Moments M = moments(hull[0]);
	double aire = M.m00;
	double diff = abs(prev_aire - aire);
	for (int i = 1; i < size; i++)
	{
		M = moments(hull[i]);
		aire = M.m00;
		if (diff > abs(prev_aire - aire))
		{
			diff = abs(prev_aire - aire);
			index = i;
		}
	}
	return index;
}

void CheckParameters(int& x0, int& y0, int& width, int& height, Size size)
// On vérifie que les paramètres de notre fenêtrage sont valides :
// - width et height pas plus grand que ceux de la frame
// - (x0,y0) est le centre de la fenêtre, il doit se trouver dans la frame
// - si ces 4 paramètres impliquent qu'un bord de la fenêtre ne se trouve pas dans la frame, on les modifie en conséquence
{
	width = min(width, size.width);
	height = min(height, size.height);

	int maxx = x0 + floor(width / 2);
	int minx = x0 - floor(width / 2);
	int maxy = y0 + floor(height / 2);
	int miny = y0 - floor(height / 2);

	if (minx < 0)
	{
		width = maxx;
		x0 = floor(maxx / 2);
	}
	if (miny < 0)
	{
		height = maxy;
		y0 = floor(maxy / 2);
	}
	if (maxx >= size.width)
	{
		width = size.width - minx;
		x0 = minx + floor(width / 2);
	}
	if (maxy >= size.height)
	{
		height = size.height - miny;
		y0 = miny + floor(height / 2);
	}

	maxx = x0 + floor(width / 2);
	minx = x0 - floor(width / 2);
	maxy = y0 + floor(height / 2);
	miny = y0 - floor(height / 2);
}

vector<int> SetWidthAndHeight(double& prev_aire, double aire, Rect prev_ROI, int multiple_aire)
// Choisit la largeur et la hauteur de la fenêtre à applique en fonction de l'aire de la voile
{
	int height, width;
 	if ((prev_aire != 0) && (aire < 0.75 * prev_aire))
	{
		height = double(float(prev_ROI.height) * 0.8);
		width = double(float(prev_ROI.width) * 0.8);
	}
	else
	{
		height = (double)sqrt(multiple_aire * aire);
		width = height;
	}
	prev_aire = aire;
	return { width, height };
}

Mat InfoFrame(int counter, Point2f& pos, Mat src, Rect& prev_ROI, Rect ROI, double& prev_aire, double aire_cible, bool &neverFound, int width, int height, int multiple_aire, bool show, bool resizeFenetre, int &count_errors, vector<Point2f> &PointsVoile)
// Correspond au main() du tracking de contour
// counter			Numero de la frame
// pos				Position de la voile, elle n'est pas renseignée au début de la fonction mais sera modifiée à la fin.
// prev_ROI			Rectangle définissant la fenêtre de la frame précédente.
// ROI				Rectangle définissant la fenêtre de la frame actuelle, il n'est pas renseigné au début de la fonction mais
//					sera modifié à la fin.
// prev_aire		Aire de la voile à la frame précédente. A la fin de la fonction, cette variable sera l'aire de la voile à
//					la frame actuelle.
// aire_cible		Aire de la voile donc aire recherchée.
// neverFound		Booléen qui permet de savoir si la voile a déjà été trouvée dans une frame précédente ou non.
// width			Largeur de la frame
// height			Hauteur de la frame
// multiple_aire	Coefficient : l'aire de la fenêtre de recherche de contour est égale à : multiple_aire * Aire de la voile.
// show				Booléen qui dit si l'on doit afficher les frames avec le contour de la voile ou non.
// resize			Booléen qui dit si l'on doit redimensionner la fenêtre d'affichage afin que les images tiennent sur l'écran.
// count_errors		Nombre d'erreurs.
// PointsVoile		Correspond aux deux points extrêmes de la voile. On suppose que ce sont les deux points de la longueur inférieure du rectangle.
{
	Size size = src.size();
	double aire_src = double(size.width) * double(size.height); // Aire de la frame
	double aire_fen = double(prev_ROI.width) * double(prev_ROI.height); // Aire de la fenêtre de la frame précédente
	Mat fenetre = src(prev_ROI); // On définit la fenêtre de recherche
	std::vector<std::vector<Point>> newContours, allContours, newContours2;
	vector<Vec4i>  hierarchy_newContours;
	int index = 0;
	Mat recons, answer;
	int x0, y0;

	Mat drawing = contoursConvex(fenetre, newContours, allContours, counter, show); // On calcule les contours fermés présents dans la fenêtre.

	Moments M;
	newFunction(newContours, prev_aire, aire_src, aire_fen, index, M);
	x0 = int(M.m10 / M.m00) + prev_ROI.x; // Position x de la voile sur la frame
	y0 = int(M.m01 / M.m00) + prev_ROI.y; // Position y de la voile sur la frame
	double aire = M.m00;

	if ((neverFound) && ((aire > aire_cible) || (aire < 0.3 * aire_cible)))
	{
		std::cout << "Voile non detectee, on passe a la frame suivante." << endl;
		count_errors++;
	}
	else if ((prev_aire != 0) && ((aire < 0.6 * prev_aire) || (aire > 1.7 * prev_aire)))
	{
		std::cout << "On ne detecte pas le bon contour." << endl;
		count_errors++;
	}
	else // Si l'aire du contour correspond à la voile,
	{
		neverFound = false;
		vector<int> WandH = SetWidthAndHeight(prev_aire, aire, prev_ROI, multiple_aire); // On définit la largeur et la hauteur de notre nouvelle fenêtre.
		width = WandH[0];
		height = WandH[1];
		pos.x = x0; // On définit la position de la voile.
		pos.y = y0; // On définit la position de la voile.
		CheckParameters(x0, y0, width, height, size); // On vérifie que notre fenêtre est valide.
		ROI = { x0 - width / 2, y0 - height / 2, width, height }; // On définit notre nouvelle fenêtre.
	}


	if (!neverFound)
	{
		if (show)
		{
			drawContours(drawing, newContours, index, Scalar(0, 0, 255), 2, LINE_8, hierarchy_newContours, 0); // On dessine le contour dans la matrice drawing.
		}
		RotatedRect MAC = minAreaRect(newContours[index]);
		Point2f vertices[4];
		MAC.points(vertices);
		float ymax = vertices[0].y;
		int indexmax = 0;
		for (int i = 0; i < 4; i++)
		{
			if (show)
			{
				line(drawing, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 255), 4);
			}
			if (vertices[i].y > ymax)
			{
				ymax = vertices[i].y;
				indexmax = i;
			}
		}

		vector<Point2f> vertices_pad(4);
		Point2f point;
		for (int i = 0; i < 4; i++)
		{
			vertices_pad[i].x = vertices[i].x + prev_ROI.x;
			vertices_pad[i].y = vertices[i].y + prev_ROI.y;
		}
		if (dist(vertices_pad[indexmax], vertices_pad[(indexmax + 1) % 4]) < dist(vertices_pad[indexmax], vertices_pad[(indexmax + 3) % 4]))
		{
			point = vertices_pad[(indexmax + 3) % 4];
		}
		else
		{
			point = vertices_pad[(indexmax + 1) % 4];
		}

		if (point.x < vertices_pad[indexmax].x)
		{
			PointsVoile[0] = point;
			PointsVoile[1] = vertices_pad[indexmax];
		}
		else
		{
			PointsVoile[0] = vertices_pad[indexmax];
			PointsVoile[1] = point;
		}
		Mat drawing1 = drawing;
		if (show)
		{
			recons = padding(drawing, prev_ROI.x, prev_ROI.y, size); // On redimensionne l'image de dessin à la taille de la frame.
			circle(recons, pos, 4, Scalar(0, 255, 0), -1); // On dessine un cercle correspondant à la position de la voile.
			rectangle(recons, ROI, Scalar(0, 255, 0), 1, 8, 0); // On dessine notre fenêtre.
			//circle(recons, PointsVoile[0], 4, Scalar(255, 255, 255), -1);
			//circle(recons, PointsVoile[1], 4, Scalar(255, 255, 255), -1);
			answer = addMatrices(src, recons); // On superpose frame et dessin.

			if (resizeFenetre)
			{
				resize(answer, answer, Size(answer.cols / 2, answer.rows / 2));
			}
			const char* answer_window = "answer";
			cv::namedWindow(answer_window);
			cv::imshow(answer_window, answer); // On affiche le tout.
		}
	}
	prev_ROI = ROI;
	return fenetre;
}

void newFunction(std::vector<std::vector<Point>> newContours, double prev_aire, double aire_src, double aire_fen, int &index, Moments &M)
{
	int numberOfContours = newContours.size();
	if (numberOfContours == 0) // Si on ne trouve pas de contours,
	{
		std::cout << "No contours" << endl;

		// Puis on récupère l'indice du contour correspondant à la voile.
		if (prev_aire == 0)
		{
			index = indexContour(newContours, aire_src, aire_fen);
		}
		else
		{
			index = indexClosestArea(prev_aire, newContours);
		}
	}
	else // Si on a trouvé au moins un contour
	{
		index = indexContour(newContours, aire_src, aire_fen); // On récupère l'indice du contour correspondant à la voile.
	}

	M = moments(newContours[index]); // Calcul des moments du contour trouvé
}

float dist(Point2f A, Point2f B)
{
	return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}
