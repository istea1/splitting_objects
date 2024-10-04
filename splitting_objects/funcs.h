#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

double angleBetween(const Point& a, const Point& b);

vector<Point> cpe(vector<Point> c);

bool classify_point_by_straight(Point p, Point p1, Point p2);

vector<Point> findCornerPoints(const std::vector<Point>& contour, double threshold1, double threshold2);

vector<vector<Point>> find_all_concave_points(Mat im, double approx_thresh, int binary_thresh);

Mat draw_points_on_picture(Mat im, double approx_thresh, int binary_thresh);