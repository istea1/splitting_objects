#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>
#include "data_manage.h"
using namespace std;
using namespace cv;

class Ellipse {
public:
	Ellipse(vector<Point> segment);
	vector<Point2f> contour = {};
	vector<Point> self_segment;
	Mat coefficents = Mat(6, 1, CV_16F);
	double Eratio = 0;
	double minorAxisL = 0;
	double majorAxisL = 0;
	double square = 0;
	double deviation_of_segment = 100;
	double r_circle = 0;
	Point center_circle;
	Point2f center = Point2f(0, 0);
	Point2f map1, map2, mip1, mip2;

private:
	void calculate_coefs();
	void make_self_contour();
	void calc_self_deviation();
};

Point findCircleCenter(const Point& A, const Point& B, const Point& C, double R);