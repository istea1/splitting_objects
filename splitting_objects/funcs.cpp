#include "funcs.h"

using namespace std;
using namespace cv;

vector<vector<Point>> find_contours(Mat im, int binary_thresh) {
	Mat im_gray;
	cvtColor(im, im_gray, COLOR_BGR2GRAY);
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresh, contours, hierarchy, RETR_TREE, 3);
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

vector<Point> ellipse_fitting(RotatedRect ellipse) {
	std::vector<cv::Point> integerPoints;

	// Получаем параметры эллипса
	cv::Point2f center = ellipse.center;
	cv::Size2f size = ellipse.size;
	float angle = ellipse.angle * CV_PI / 180; // Угол в радианах

	float a = size.width / 2; // Полуось по X
	float b = size.height / 2; // Полуось по Y
	int numPoints = 100;
	// Генерируем точки вдоль эллипса
	for (int i = 0; i < numPoints; ++i) {
		// Параметр t от 0 до 2π
		float t = (float)i / numPoints * 2 * CV_PI;

		// Параметрическое уравнение эллипса
		float x = a * cos(t);
		float y = b * sin(t);

		// Применяем вращение к точкам
		float rotatedX = center.x + (x * cos(angle) - y * sin(angle));
		float rotatedY = center.y + (x * sin(angle) + y * cos(angle));

		// Преобразуем в целые координаты
		integerPoints.push_back(cv::Point(static_cast<int>(std::round(rotatedX)), static_cast<int>(std::round(rotatedY))));
	}

	return integerPoints;
}

void ellipses_fitting(vector<vector<Point>>& segments, vector<RotatedRect>& Ellipse_Rects,
	vector<vector<Point>>& ellipse_contours) {
	for (int i = 0; i < segments.size(); i++) {
		if (segments[i].size() >= 5) {
			RotatedRect n_e = fitEllipse(segments[i]);
			Ellipse_Rects[i] = n_e;
			ellipse_contours[i] = ellipse_fitting(n_e);
		}
		else {
			Ellipse_Rects[i] = RotatedRect();
			ellipse_contours[i] = vector<Point>();
		}
	}
}

Mat find_ellipse_coefficients(const vector<Point>& points) {
	// Создаем матрицу M и вектор b
	cv::Mat M(points.size(), 6, CV_64F);
	cv::Mat b(points.size(), 1, CV_64F);

	for (size_t i = 0; i < points.size(); ++i) {
		double x = points[i].x;
		double y = points[i].y;

		M.at<double>(i, 0) = x * x;  // A
		M.at<double>(i, 1) = x * y;  // B
		M.at<double>(i, 2) = y * y;  // C
		M.at<double>(i, 3) = x;      // D
		M.at<double>(i, 4) = y;      // E
		M.at<double>(i, 5) = 1;      // F
	}

	// Решаем систему уравнений M * [A, B, C, D, E, F] = 0
	cv::Mat AT = M.t();
	cv::Mat ATA = AT * M;
	cv::Mat ATb = -AT * b;  // Поскольку мы ищем уравнение вида Ax=0, нам нужен -AT*b

	// Находим псевдообратную матрицу
	cv::Mat coefficients;
	cv::solve(ATA, ATb, coefficients, cv::DECOMP_SVD);
	return coefficients;
}

double find_dis_segment_to_ellipse(vector<Point> segment, Mat coefficents) {
	double dis = 0;
	for (int i = 0; i < segment.size(); i++) {
		int x = segment[i].x;
		int y = segment[i].y;
		double A = coefficents.at<double>(0);
		double B = coefficents.at<double>(1);
		double C = coefficents.at<double>(2);
		double D = coefficents.at<double>(3);
		double E = coefficents.at<double>(4);
		double F = coefficents.at<double>(5);
		dis += abs(1 - (A * x * x + B * x * y + C * y * y + D * x + E * y + F));
	}
	return dis / segment.size();
}

double calculateEratio(Mat coefficents) {
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	double determinant = B * B - 4 * A * C;

	if (determinant <= 0) {
		return 1.1;
	}

	// Параметры
	double a = std::sqrt(2 * (A * E * E + C * D * D - B * D * E) / (determinant * (A + C)));
	double b = std::sqrt(2 * (A * E * E + C * D * D - B * D * E) / (determinant * (A + C) + 2 * sqrt(determinant)));

	// Длины больших и малых осей
	double majorAxis = 2 * std::max(a, b);
	double minorAxis = 2 * std::min(a, b);
	return minorAxis / majorAxis;
}

double calculate_minorAxis(Mat coefficents) {
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	double determinant = B * B - 4 * A * C;

	if (determinant <= 0) {
		throw std::invalid_argument("Уравнение не описывает действительный эллипс.");
	}

	// Параметры
	double a = std::sqrt(2 * (A * E * E + C * D * D - B * D * E) / (determinant * (A + C)));
	double b = std::sqrt(2 * (A * E * E + C * D * D - B * D * E) / (determinant * (A + C) + 2 * sqrt(determinant)));

	// Длины больших и малых осей
	double minorAxis = 2 * std::min(a, b);
	return minorAxis;
}

double calculate_majorAxis(Mat coefficents) {
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	double determinant = B * B - 4 * A * C;

	if (determinant <= 0) {
		throw std::invalid_argument("Уравнение не описывает действительный эллипс.");
	}

	// Параметры
	double a = std::sqrt(2 * (A * E * E + C * D * D - B * D * E) / (determinant * (A + C)));
	double b = std::sqrt(2 * (A * E * E + C * D * D - B * D * E) / (determinant * (A + C) + 2 * sqrt(determinant)));

	// Длины больших и малых осей
	double majorAxis = 2 * std::max(a, b);
	return majorAxis;
}

bool is_ellipse_for_combine(RotatedRect EllipseRect, vector<Point> ellipse_contour, vector<Point> segment) {
	double disTh = 3;
	double eTh = 0.333;
	if (ellipse_contour.size() >= 5) {
		Mat ellipse_coefficents = find_ellipse_coefficients(ellipse_contour);

		return (find_dis_segment_to_ellipse(segment, ellipse_coefficents) < disTh and calculateEratio(ellipse_coefficents) > eTh);
	}
	else {
		return false;
	}
	
}

void combine_ellipses(vector<RotatedRect>& Ellipse_Rects, vector<vector<Point>>& ellipse_contours,
	vector<vector<Point>>& segments) {
	int i = 0;
	double dminTh = 10;
	double eps = 1;
	while (i < Ellipse_Rects.size()) {
		int j = i + 1;
		while (j < Ellipse_Rects.size()) {
			vector<Point> Nl;
			vector<Point> Li = segments[i];
			vector<Point> Lj = segments[j];
			vector<Point> Ei = ellipse_contours[i];
			vector<Point> Ej = ellipse_contours[j];
			Nl = Li;
			vector<Point>::iterator iter_paste;
			iter_paste = Lj.begin();
			Nl.insert(iter_paste, Lj.begin(), Lj.end());
			if (Li.size() >= 5 and Lj.size() >= 5) {
				RotatedRect ellipse = fitEllipse(Nl);
				RotatedRect ellipsei = Ellipse_Rects[i];
				RotatedRect ellipsej = Ellipse_Rects[j];

				vector<Point> Eij = ellipse_fitting(ellipse);
				double dist1 = sqrt(
					(ellipse.center.x - ellipsei.center.x) * (ellipse.center.x - ellipsei.center.x) +
					(ellipse.center.y - ellipsei.center.y) * (ellipse.center.y - ellipsei.center.y));
				double dist2 = sqrt(
					(ellipse.center.x - ellipsej.center.x) * (ellipse.center.x - ellipsej.center.x) +
					(ellipse.center.y - ellipsej.center.y) * (ellipse.center.y - ellipsej.center.y));
				double dist3 = sqrt(
					(ellipsei.center.x - ellipsej.center.x) * (ellipsei.center.x - ellipsej.center.x) +
					(ellipsei.center.y - ellipsej.center.y) * (ellipsei.center.y - ellipsej.center.y));
				bool case1 = ((dist1 > dminTh) and (dist2 > dminTh) and (dist3 > 2.5 * dminTh));
				Mat Ei_coefs = find_ellipse_coefficients(Ei);
				Mat Ej_coefs = find_ellipse_coefficients(Ej);
				double minA1 = calculate_minorAxis(Ei_coefs);
				double minA2 = calculate_minorAxis(Ej_coefs);
				double maxA1 = calculate_majorAxis(Ei_coefs);
				double maxA2 = calculate_majorAxis(Ej_coefs);
				bool case2 = (
					(minA1 < dminTh) and
					(minA2 < dminTh) and
					(abs(minA1 - minA2) < 0.05 * dminTh) and
					(abs(maxA1 - maxA2) < dminTh));
				bool case3 = 
					abs(find_dis_segment_to_ellipse(Nl, find_ellipse_coefficients(Eij)) - 
					0.5 * (find_dis_segment_to_ellipse(Li, Ei_coefs) + find_dis_segment_to_ellipse(Lj, Ej_coefs)))
					<= eps;
				if (case1 and not(case2 or case3)) {
					continue;
				}
				else if (case2 or case3) {
					Ellipse_Rects[i] = ellipse;
					ellipse_contours[i] = Eij;
					segments[i] = Nl;
					Ellipse_Rects.erase(Ellipse_Rects.begin() + j);
					ellipse_contours.erase(ellipse_contours.begin() + j);
					segments.erase(segments.begin() + j);
					i = 0;
					break;
				}
			}
		}
	}
}

void refine_ellipses(vector<RotatedRect>& Ellipse_Rects, vector<vector<Point>>& ellipse_contours,
	vector<vector<Point>>& segments) {

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

		vector<RotatedRect> Ellipse_Rects;
	}


	return contours;
}

Mat draw_ellipses(Mat im, vector<vector<vector<Point>>> ellipses) {
	for (int i = 0; i < ellipses.size(); i++) {
		for (int j = 0; j < ellipses[i].size(); j++) {
			for (int k = 0; k < ellipses[i][j].size(); k++) {
				im.at<Vec3b>(ellipses[i][j][k].y, ellipses[i][j][k].x) = Vec3b(0, 255, 0);
			}
		}
	}
	return im;
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

void main_func(Mat im, float approx_thresh, int binary_thresh) {
	vector<vector<Point>> contours = find_contours(im, binary_thresh);
	vector<vector<Point>> cps = find_all_concave_points(im, approx_thresh, binary_thresh);
	vector<vector<vector<Point>>> ellipses;
	vector<vector<vector<Point>>> segments;
	for (int i = 0; i < contours.size(); i++) {
		vector<Point> now_contour = contours[i];
		approxPolyDP(contours[i], now_contour, approx_thresh, true);
		vector<vector<Point>> segments = make_segments_of_contour(contours[i], cpe(now_contour));
		vector<RotatedRect> Ellipse_Rects(segments.size());
		vector<vector<Point>> ellipse_contours(segments.size());
		vector<vector<Point>> segments_combine;
		vector<RotatedRect> Ellipse_Rects_combine;
		vector<vector<Point>> ellipse_contours_combine;
		vector<vector<Point>> segments_refine;
		vector<RotatedRect> Ellipse_Rects_refine;
		vector<vector<Point>> ellipse_contours_refine;
		ellipses_fitting(segments, Ellipse_Rects, ellipse_contours);
		for (int j = 0; j < segments.size(); j++) {
			if (is_ellipse_for_combine(Ellipse_Rects[j], ellipse_contours[j], segments[j])) {
				Ellipse_Rects_combine.push_back(Ellipse_Rects[j]);
				segments_combine.push_back(segments[j]);
				ellipse_contours_combine.push_back(ellipse_contours[j]);
			}
			else {
				Ellipse_Rects_refine.push_back(Ellipse_Rects[j]);
				segments_refine.push_back(segments[j]);
				ellipse_contours_refine.push_back(ellipse_contours[j]);

			}


		}
		cout << "a\n";

		combine_ellipses(Ellipse_Rects_combine, ellipse_contours_combine, segments_combine);
		refine_ellipses(Ellipse_Rects_refine, ellipse_contours_refine, segments_refine);
		ellipse_contours = ellipse_contours_combine;
		for (int j = 0; j < ellipse_contours_refine.size(); j++) {
			ellipse_contours.push_back(ellipse_contours_refine[j]);
		}
		ellipses.push_back(ellipse_contours);

	}

	//Mat img_contours = draw_points_on_picture(im.clone(), contours);
	Mat image_copy = draw_ellipses(im.clone(), ellipses);
	//Mat cps_img = draw_points_on_picture(im, cps);
	namedWindow("out", WINDOW_NORMAL);
	//namedWindow("cps", WINDOW_NORMAL);
	//namedWindow("cntrs", WINDOW_NORMAL);
	//imshow("cntrs", img_contours);
	//imshow("cps", cps_img);
	imshow("out", image_copy);
	waitKey(0);
}