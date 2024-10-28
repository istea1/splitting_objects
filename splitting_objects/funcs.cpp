#include "funcs.h"

using namespace std;
using namespace cv;
//функция по расчету угла между двумя векторами
double angleBetween(const Point& a, const Point& b) {
	double dotProduct = a.x * b.x + a.y * b.y;
	double magnitudeA = std::sqrt(a.x * a.x + a.y * a.y);
	double magnitudeB = std::sqrt(b.x * b.x + b.y * b.y);
	return std::acos(dotProduct / (magnitudeA * magnitudeB));
}
//функция, которая проверяет слева или справа p1 от прямой p-p2
bool classify_point_by_straight(Point p, Point p1, Point p2) {
	Point a = p2 - p1;
	Point b = p - p1;
	double sa = a.x * b.y - b.x * a.y;
	if (sa > 0.0) {
		return false;
	}
	else if (sa < 0.0) {
		return true;
	}
	else {
		return false;
	}
}
//функция, которая находит угловые точки(и выпуклые и вогнутые)
vector<Point> find_corner_points(const std::vector<Point>& contour, double threshold1, double threshold2) {
	vector<Point> corners;

	for (int i = 0; i < contour.size(); ++i) {
		Point v1 = { contour[i].x - contour[i - 1].x, contour[i].y - contour[i - 1].y };
		Point v2 = { contour[i + 1].x - contour[i].x, contour[i + 1].y - contour[i].y };

		double magnitudeV1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
		double magnitudeV2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);

		if (magnitudeV1 > 0 && magnitudeV2 > 0) {
			double angle = angleBetween(v1, v2);
			if (angle > threshold1 and angle < threshold2) {
				corners.push_back(contour[i]);
			}
		}
	}

	return corners;
}
//функция, которая выбирает из предыдущей вогнутые точки
vector<Point> cpe(vector<Point> c) {
	vector<Point> cpc;
	//double eps = 0.001;
	double a1ths = 0.6, a2ths = 2.8;
	int ki = 1;
	int j = 0;
	vector<Point> mcps = find_corner_points(c, a1ths, a2ths);
	if (c.size() == 1) {
		return vector<Point>();
	}
	for (int i = 0; i < c.size(); i++) {
		int pre = i, now = (i + 1) % c.size(), next = (i + 2) % c.size();		
		if (c[now] == mcps[j]) {
			if (classify_point_by_straight(c[pre], c[now], c[next]) == false) {
				cpc.push_back(c[now]);
			}
			j += 1;

		}
	}
	return cpc;
}
//функция, находящая вогнутые точки из картинки(в чем и заключается задача)
vector<vector<Point>> find_all_concave_points(Mat im, double approx_thresh, int binary_thresh) {
	Mat im_gray;
	cvtColor(im, im_gray, COLOR_BGR2GRAY);
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresh, contours, hierarchy, RETR_TREE, 3);
	for (int i = 0; i < contours.size(); i++) {
		vector<Point> now_contour;
		approxPolyDP(contours[i], now_contour, approx_thresh, true);

		contours[i] = cpe(now_contour);

	}


	return contours;
}



//функция по рисованию вогнутых точек на картинке зеленым цветом
Mat draw_points_on_picture(Mat im, vector<vector<Point>> cps) {
	for (int i = 0; i < cps.size(); i++) {
		for (int j = 0; j < cps[i].size(); j++) {
			im.at<Vec3b>(cps[i][j].y, cps[i][j].x) = Vec3b(0, 255, 0);
		}
	}
	return im;
}