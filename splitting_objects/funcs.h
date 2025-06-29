#include "Ellipse.h"

void check_is_files_identic(string name1, string name2);

bool is_ellipse_for_combine(Ellipse ellipse, double disTh);

vector<vector<Point>> find_contours(Mat im, int binary_thresh, int method);

double angleBetween(const Point& a, const Point& b);

vector<Point> cpe(vector<Point> c);

bool classify_point_by_straight(Point p, Point p1, Point p2);

vector<Point> find_corner_points(vector<Point> contour, double threshold1, double threshold2);

vector<vector<Point>> find_all_concave_points(Mat im, double approx_thresh, int binary_thresh);

vector<vector<Point>> make_segments_of_contour(vector<Point> c, vector<Point> cps, pair<int, int> image_size);

void ellipses_selection(const vector<vector<Point>>& segments,
	vector<Ellipse>& for_refine,
	vector<Ellipse>& for_combine, double disTh, Mat image);

vector <vector<vector<Point>>> contour_processing_return_segments(Mat im, int binary_thresh, int method_for_cpe, int method_for_segments, float approx_thresh);

double find_dis_segment_to_ellipse(vector<Point> segment, Mat coefficents, Ellipse ellipse);

void combine_ellipses(vector<Ellipse>& ellipses, double dminTh, Mat image);

void refine_ellipses(vector<vector<Point>> &refine_segments, vector<Ellipse>& ellipses);

vector<vector<Ellipse>> main_func(Mat im, float approx_thresh, int binary_thresh, double dTh, double disTh, double dminTh, string imgname);

void draw_points_on_picture(Mat &im, vector<vector<Point>> cps);

Mat draw_ellipses(Mat im, vector<vector<Ellipse>> ellipses);

Mat prepare_im(Mat im_binary);
