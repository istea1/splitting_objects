// splitting_objects.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include "funcs.h"

using namespace std;
using namespace cv;

int main()
{
    Mat im, im_gray;
    im = imread("img_test2.jpg");
    cvtColor(im, im_gray, COLOR_BGR2GRAY);
    Mat thresh;
    threshold(im_gray, thresh, 25, 255, THRESH_BINARY);
    Mat image_copy = draw_points_on_picture(im, 1, 25);
    
    namedWindow("out", WINDOW_NORMAL);
    imshow("thresh", thresh);
    imwrite("out.png", image_copy);
    waitKey(0);
}

