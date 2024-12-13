#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

class Ellipse {
public:
	Ellipse(vector<Point> segment);
	vector<Point2f> contour;
	vector<Point> self_segment;
	Mat coefficents;
	double Eratio;
	double minorAxisL;
	double majorAxisL;
	Point2f center;
private:
	void calculate_coefs();
	void make_self_contour();
};