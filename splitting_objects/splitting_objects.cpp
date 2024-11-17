// splitting_objects.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include "funcs.h"

using namespace std;
using namespace cv;

int main()
{
    Mat im = imread("img_test5.png");
    main_func(im, 3, 40);
    //imshow("f", im);
    //waitKey(0);
}

