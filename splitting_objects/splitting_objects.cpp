// splitting_objects.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include "funcs.h"

using namespace std;
using namespace cv;

int main()
{
    Mat im, im_gray;
    im = imread("image_test.png");
    cvtColor(im, im_gray, COLOR_BGR2GRAY);
    Mat thresh;
    threshold(im_gray, thresh, 40, 255, THRESH_BINARY);
    vector<vector<Point>> contours;
    vector<vector<Point>> cpec;
    cpec.push_back(vector<Point>());
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    Mat image_copy = Mat(im.rows, im.cols, CV_32F);
    cpec[0] = cpe(contours[0]);
    for (int i = 0; i < cpec.size(); i++) {
        for (int j = 0; j < cpec[i].size(); j++) {
            image_copy.at<float>(cpec[i][j].y, cpec[i][j].x) = 1;
        }
    }
    imshow("a.png", image_copy);
    waitKey(0);
}

