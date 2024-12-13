// splitting_objects.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include "funcs.h"

using namespace std;
using namespace cv;

int main(int argc, char** args)
{
    Mat im = imread(args[1]);
    vector<vector<Ellipse>> ellipses = main_func(im, 3, 55);

	//Mat img_contours = draw_points_on_picture(im.clone(), contours);
	Mat image_copy = draw_ellipses(im.clone(), ellipses);
	//make_data(cps[0]);
	//Mat cps_img = draw_points_on_picture(im, cps);
	namedWindow("out", WINDOW_NORMAL);
	//namedWindow("cps", WINDOW_NORMAL);
	//namedWindow("cntrs", WINDOW_NORMAL);
	//imshow("cntrs", img_contours);
	//imshow("cps", cps_img);
	imshow("out", image_copy);
	waitKey(0);
    //imshow("f", im);
    //waitKey(0);
}

