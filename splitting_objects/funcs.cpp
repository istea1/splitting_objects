#include "funcs.h"

using namespace std;
using namespace cv;

void make_data(vector<Point> points) {
	ofstream file;
	file.open("file.txt");
	for (int i = 0; i < points.size(); i++) {
		file << points[i].x << " " << points[i].y << "\n";
	}
}

vector<vector<Point>> find_contours(Mat im, int binary_thresh, int method) {
	Mat im_gray(im.rows, im.cols, CV_8U);
	for (int i = 0; i < im.rows; i++) {
		for (int j = 0; j < im.cols; j++) {
			int n = (int(im.at<Vec3b>(i, j)[0])
				+ int(im.at<Vec3b>(i, j)[1]) + int(im.at<Vec3b>(i, j)[2])) / 3;
			im_gray.at<__int8>(i, j) = n;
		}
	}
	//cvtColor(im, im_gray, COLOR_BGR2GRAY);
	//cout << im_gray.at<int>(1917, 1916) << "\n";
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	thresh = prepare_im(thresh);
	findContours(thresh, contours, hierarchy, RETR_TREE, method);
	return contours;
}

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
		if (j == mcps.size()) {
			break;
		}
		if (c[now] == mcps[j]) {
			if (classify_point_by_straight(c[pre], c[now], c[next]) == false) {
				cpc.push_back(c[now]);
			}
			j += 1;
		}

	}

	return cpc;
}

vector<vector<Point>> make_segments_of_contour(vector<Point> c, vector<Point> cps) {
	vector<vector<Point>> segments(1);
	if (cps.empty()) {
		return segments;
	}
	int j = 0;
	int paste_i = 0;
	for (int i = 0; i < c.size(); i++) {

		if (cps[j] == c[i]) {
			if (j == cps.size() - 1) {
				paste_i = 0;
			}
			else {
				segments.push_back(vector<Point>());
				paste_i += 1;
				j += 1;
			}
		}
		segments[paste_i].push_back(c[i]);

	}

	return segments;
}

void ellipses_selection(vector<vector<Point>>& segments, 
	vector<vector<Point>> &for_refine, 
	vector<Ellipse>& for_combine) {
	for (int i = 0; i < segments.size(); i++) {
		if (segments[i].size() > 5) {
			Ellipse ellipse = Ellipse(segments[i]);
			if (is_ellipse_for_combine(ellipse)) for_combine.push_back(ellipse);
			else for_refine.push_back(segments[i]);
		}
	}
}

double find_dis_segment_to_ellipse(vector<Point> segment, Mat coefficents) {
	double dis = 0;
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	for (Point p: segment) {
		dis += abs(A * p.x * p.x + B * p.x * p.y + C * p.y * p.y + D * p.x + E * p.y + F);
	}
	dis = dis / segment.size();
	return dis;
}

bool is_ellipse_for_combine(Ellipse ellipse) {
	double disTh = 0.035;
	double eTh = 0.333;
	return (find_dis_segment_to_ellipse(ellipse.self_segment, ellipse.coefficents) <= disTh and ellipse.Eratio >= eTh);
	
}

void combine_ellipses(vector<Ellipse>& ellipses) {
	double dminTh = 10;
	double eps = 0.1;
	for(int i = 0; i < ellipses.size(); i++) {
		for (int j = i + 1; j < ellipses.size(); j++) {

			vector<Point> Nl;
			vector<Point> Li = ellipses[i].self_segment;
			vector<Point> Lj = ellipses[j].self_segment;
			vector<Point2f> Ei = ellipses[i].contour;
			vector<Point2f> Ej = ellipses[j].contour;

			Nl = Li;
			for (int k = 0; k < Lj.size(); k++) {
				Nl.push_back(Lj[k]);
			}

			if (Li.size() >= 5 and Lj.size() >= 5) {
				Ellipse new_ellipse = Ellipse(Nl);

				Ellipse ellipsei = ellipses[i];
				Ellipse ellipsej = ellipses[j];

				vector<Point2f> Eij = new_ellipse.contour;
				double dist1 = sqrt(
					(new_ellipse.center.x - ellipsei.center.x) * (new_ellipse.center.x - ellipsei.center.x) +
					(new_ellipse.center.y - ellipsei.center.y) * (new_ellipse.center.y - ellipsei.center.y));
				double dist2 = sqrt(
					(new_ellipse.center.x - ellipsej.center.x) * (new_ellipse.center.x - ellipsej.center.x) +
					(new_ellipse.center.y - ellipsej.center.y) * (new_ellipse.center.y - ellipsej.center.y));
				double dist3 = sqrt(
					(ellipsei.center.x - ellipsej.center.x) * (ellipsei.center.x - ellipsej.center.x) +
					(ellipsei.center.y - ellipsej.center.y) * (ellipsei.center.y - ellipsej.center.y));
				
				bool case1 = ((dist1 > dminTh) and (dist2 > dminTh) and (dist3 > 2.5 * dminTh));
				Mat Ei_coefs = ellipsei.coefficents;
				Mat Ej_coefs = ellipsej.coefficents;
				double minA1 = ellipsei.minorAxisL;
				double minA2 = ellipsej.minorAxisL;
				double maxA1 = ellipsei.majorAxisL;
				double maxA2 = ellipsej.majorAxisL;
				bool case2 = (
					(minA1 < dminTh) and
					(minA2 < dminTh) and
					(abs(minA1 - minA2) < 0.05 * dminTh) and
					(abs(maxA1 - maxA2) < dminTh));
				bool case3 = 
					abs(find_dis_segment_to_ellipse(Nl, new_ellipse.coefficents) - 
					0.5 * (find_dis_segment_to_ellipse(Li, Ei_coefs) + find_dis_segment_to_ellipse(Lj, Ej_coefs)))
					<= eps;
				if (case1 and (not case2 or not case3)) {
					continue;
				}
				else if (case2 or case3) {
					ellipses[i] = new_ellipse;
					ellipses.erase(ellipses.begin() + j);
					i = 0;
					break;
				}
				cout << "a\n";

			}
		}
	}
}

void refine_ellipses(vector<vector<Point>> refine_segments, vector<Ellipse>& ellipses) {
	double disThRe = 0.114;
	for (int i = 0; i < refine_segments.size(); i++) {
		double mn_dist = INFINITY;
		vector<Ellipse> best_ellipse;
		int index = 0;
		for (int j = 0; j < ellipses.size(); j++) {
			vector<Point> new_segment = ellipses[j].self_segment;
			for (Point p : refine_segments[i]) {
				new_segment.push_back(p);
			}
			Ellipse new_ellipse = Ellipse(new_segment);
			double dist = find_dis_segment_to_ellipse(new_segment, new_ellipse.coefficents);
			if (dist < mn_dist) {
				mn_dist = dist;
				best_ellipse.push_back(new_ellipse);
				index = j;
			}
		}
		if (mn_dist < disThRe) {
			ellipses[index] = best_ellipse[best_ellipse.size() - 1];
		}
		else {
			ellipses.push_back(Ellipse(refine_segments[i]));
		}
	}
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
		vector<Point> c1 = contours[i];
		approxPolyDP(contours[i], now_contour, approx_thresh, true);
		if (i == 1) {
			contours = vector<vector<Point>>(1);
			contours[0] = make_segments_of_contour(c1, cpe(now_contour))[4];
		}
	}


	return contours;
}

Mat draw_ellipses(Mat im, vector<vector<Ellipse>> ellipses) {
	cout << im.rows << " " << im.cols << "\n";
	for (int i = 0; i < ellipses.size(); i++) {
		for (int j = 0; j < ellipses[i].size(); j++) {
			for (Point2f p:ellipses[i][j].contour) {
				int y = p.y;
				int x = p.x;
				//cout << x << " " << y << "\n";
				if (y > 0 and x > 0 and y < im.rows and x < im.cols) {
					im.at<Vec3b>(y, x) = Vec3b(0, 255, 0);
				}
			}
		}
	}
	return im;
}

//функция по рисованию вогнутых точек на картинке зеленым цветом
void draw_points_on_picture(Mat &im, vector<vector<Point>> cps) {
	for (int i = 0; i < cps.size(); i++) {
		for (int j = 0; j < cps[i].size(); j++) {
			im.at<Vec3b>(cps[i][j].y, cps[i][j].x) = Vec3b(0, 255, 0);
		}
	}
}

vector<vector<Ellipse>> main_func(Mat im, float approx_thresh, int binary_thresh) {
	clock_t start = clock();
	Mat im_gray;
	cvtColor(im, im_gray, COLOR_BGR2GRAY);
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	//thresh = prepare_im(thresh);
	/*namedWindow("f", WINDOW_NORMAL);
	imshow("f", thresh);
	waitKey(0);*/
	vector<vector<Point>> contours = find_contours(im, binary_thresh, 3);
	vector<vector<Point>> contours_start = find_contours(im, binary_thresh, 1);
	Mat im_c = im.clone();
	draw_points_on_picture(im_c, contours_start);
	//namedWindow("r", WINDOW_NORMAL);
	//imshow("r", im_c);
	//waitKey(0);
	//vector<vector<Point>> cps = find_all_concave_points(im, approx_thresh, binary_thresh);
	vector<vector<Ellipse>> all_ellipses;
	vector<vector<vector<Point>>> segments;
	for (int i = 0; i < contours.size(); i++) {
		vector<Point> now_contour = contours[i];
		vector<Point> c1 = contours_start[i];
		approxPolyDP(contours[i], now_contour, approx_thresh, true);
		vector<vector<Point>> segments = make_segments_of_contour(c1, cpe(now_contour));
		vector<Ellipse> for_combine;
		vector<vector<Point>> for_refine;
		vector<Ellipse> ellipses;
		ellipses_selection(segments, for_refine, for_combine);
		combine_ellipses(for_combine);

		refine_ellipses(for_refine, for_combine);

		all_ellipses.push_back(for_combine);
		
	}
	clock_t end = clock();
	cout << (double)(end - start) / CLOCKS_PER_SEC << "seconds\n";
	return all_ellipses;
}

Mat prepare_im(Mat im_binary) {
	int w = 0, b = 0;
	Mat im_inverted = Mat(im_binary.rows, im_binary.cols, CV_8UC1);

	for (int i = 0; i < im_binary.rows; i++) {
		for (int j = 0; j < im_binary.cols; j++) {
			if (im_binary.at<__int8>(i, j) == 0) b += 1;
			else w += 1;
			im_inverted.at<__int8>(i, j) = 255 - im_binary.at<__int8>(i, j);
		}
	}
	if (w > b) return im_inverted;
	return im_binary;
}

