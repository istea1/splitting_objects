#define _USE_MATH_DEFINES
#include <opencv2/opencv.hpp>
#include <numeric>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

vector<vector<Point>> find_contours(Mat im, int binary_thresh);

double angleBetween(const Point& a, const Point& b);

vector<Point> cpe(vector<Point> c);

bool classify_point_by_straight(Point p, Point p1, Point p2);

vector<Point> find_corner_points(const std::vector<Point>& contour, double threshold1, double threshold2);

vector<vector<Point>> find_all_concave_points(Mat im, double approx_thresh, int binary_thresh);

vector<vector<Point>> make_segments_of_contour(vector<Point> c, vector<Point> cps);

vector<Point> ellipse_fitting(RotatedRect ellipse);

void ellipses_fitting(vector<vector<Point>>& segments, vector<RotatedRect>& Ellipse_Rects,
	vector<vector<Point>>& ellipse_contours);

bool is_ellipse_for_combine(RotatedRect EllipseRect, vector<Point> ellipse_contour, vector<Point> segment);

Mat find_ellipse_coefficients(const vector<Point>& points);

double calculateEratio(Mat coefficents);

double calculate_minorAxis(Mat coefficents);

double calculate_majorAxis(Mat coefficents);

double find_dis_segment_to_ellipse(vector<Point> segment, Mat coefficents);

bool is_ellipse_for_combine(RotatedRect EllipseRect, vector<Point> ellipse_contour, vector<Point> segment);

void combine_ellipses(vector<RotatedRect>& Ellipse_Rects, vector<vector<Point>>& ellipse_contours, vector<vector<Point>>& segments);

void refine_ellipses(vector<RotatedRect>& Ellipse_Rects, vector<vector<Point>>& ellipse_contours,
	vector<vector<Point>>& segments);

void main_func(Mat im, float approx_thresh, int binary_thresh);

Mat draw_points_on_picture(Mat im, vector<vector<Point>> cps);

Mat draw_ellipses(Mat im, vector<vector<vector<Point>>> ellipses);