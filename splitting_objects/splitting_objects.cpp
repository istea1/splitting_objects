// splitting_objects.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//
#include <iostream>
#include "funcs.h"
//#include "boost/filesystem.hpp"
using namespace std;
using namespace cv;


void testing() {
	vector<string> filenames = {
		"res_143_656.png", "res_N009.jpg", "res_img_test2.jpg", "res_img_test3.jpg", "res_img_test5.png", "res_B669.jpg"
		, "res_C319.jpg", "res_camera_1_2023-12-12_18-22-44_029.png", "res_camera_2_2024-02-01_13-36-59_725.png", "res_image_test.png", "res_J647.jpg"
	};
	Mat packs = Mat(3, 3, CV_16F);
	packs.at<double>(0, 0) = 3;
	packs.at<double>(0, 1) = 0.009;
	packs.at<double>(0, 2) = 10;
	packs.at<double>(1, 0) = 3.5;
	packs.at<double>(1, 1) = 0.009;
	packs.at<double>(1, 2) = 10;
	packs.at<double>(2, 0) = 4;
	packs.at<double>(2, 1) = 0.035;
	packs.at<double>(2, 2) = 15;
	for (auto filename : filenames) {
		string fullname = "test_data/" + filename;
		for (int i = 0; i < 3; i++) {
			double dTh = packs.at<double>(i, 0);
			double disTh = packs.at<double>(i, 1);
			double dminTh = packs.at<double>(i, 2);
			Mat im = imread(fullname);
			int j = 1;
			vector<vector<Ellipse>> ellipses = main_func(im, 3, j * 10, dTh, disTh, dminTh, filename);
			Mat image_copy = draw_ellipses(im.clone(), ellipses);
			imwrite("test_res/" + filename.substr(0, filename.size() - 4) + "_res_" + to_string(i) + "_" + to_string(j) + ".png", image_copy);
		}
	}
}

void testing_2() {
	vector<string> filenames = {
		"res_143_656.png", "res_N009.jpg", "res_img_test2.jpg", "res_img_test3.jpg", "res_img_test5.png", "res_B669.jpg"
		, "res_C319.jpg", "res_camera_1_2023-12-12_18-22-44_029.png", "res_camera_2_2024-02-01_13-36-59_725.png", "res_image_test.png", "res_J647.jpg"
	};
	Mat packs = Mat(3, 3, CV_16F);
	packs.at<double>(0, 0) = 3.5;
	packs.at<double>(0, 1) = 0.009;
	packs.at<double>(0, 2) = 10;
	packs.at<double>(1, 0) = 3.5;
	packs.at<double>(1, 1) = 0.009;
	packs.at<double>(1, 2) = 10;
	packs.at<double>(2, 0) = 4;
	packs.at<double>(2, 1) = 0.035;
	packs.at<double>(2, 2) = 15;
	for (auto filename : filenames) {
		string fullname = "test_data/" + filename;
		for (int i = 0; i < 1; i++) {
			double dTh = 3.5;
			double disTh = 0.009;
			double dminTh = 10;
			Mat im = imread(fullname);
			int j = 5;
			cout << dTh << " " << disTh << " " << dminTh << "\n";
			vector<vector<Ellipse>> ellipses = main_func(im, 3, j * 10, dTh, disTh, dminTh, filename);
			Mat image_copy = draw_ellipses(im.clone(), ellipses);
			//imwrite("test_res/" + filename.substr(0, filename.size() - 4) + "/" + filename.substr(0, filename.size() - 4) + ".png", im);
			imwrite("test_res/" + filename.substr(0, filename.size() - 4) + "/res" + filename.substr(0, filename.size() - 4) + to_string(i) + "_" + to_string(j) + ".png", image_copy);
		}
	}
}

int main(int argc, char** args)
{
	//camera_2_2024-02-01_13-36-59_725.png
	//N009.jpg
	//143_656.png
	//C319.jpg
	//camera_1_2023-12-12_18-22-44_029.tif.png
	//testing();
	double dTh = 3.5, dminTh = 10, disTh = 0.02; 
	//testing_2();
	string filename = "N009_1mminid.jpg";
    Mat im = imread(filename);
	//cout << im.cols << " " << im.rows << "\n";
    vector<vector<Ellipse>> ellipses = main_func(im, 3, 40, dTh, disTh, dminTh, filename);

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

