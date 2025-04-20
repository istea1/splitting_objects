#include "funcs.h"

using namespace cv;
using namespace std;


vector <vector<vector<Point>>> contour_processing_return_segments(Mat im, int binary_thresh, int method_1, int method_2, float approx_thresh) {
	Mat im_gray(im.rows, im.cols, CV_8U);
	cvtColor(im, im_gray, COLOR_BGR2GRAY);
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	vector<vector<Point>> contours_start;
	vector<Vec4i> hierarchy;
	vector<Vec4i> hierarchy_start;
	findContours(thresh, contours, hierarchy, RETR_TREE, method_1);
	findContours(thresh, contours_start, hierarchy_start, RETR_TREE, method_2);
	vector<vector<vector<Point>>> segments(contours.size());
	int i = 0;
	auto its = segments.begin();
	for (auto it = contours.begin(); it != contours.end(); it++) {
		vector<Point> now_contour = contours[i];
		vector<Point> c1 = contours_start[i];
		approxPolyDP(contours[i], now_contour, approx_thresh, true);
		vector<vector<Point>> i_segments = make_segments_of_contour(c1, cpe(now_contour));
		int ifat = hierarchy[i][3];
		if (ifat != -1) {
			contours[ifat].insert(contours[ifat].end(), contours[i].begin(), contours[i].end());
			segments[ifat].insert(segments[ifat].end(), i_segments.begin(), i_segments.end());
			segments.erase(its);
			contours.erase(it);
			its--;
			it--;
			i--;
		}
		i++;
		its++;
	}
	return segments;
}


vector<vector<Point>> find_contours(Mat im, int binary_thresh, int method) {
	Mat im_gray(im.rows, im.cols, CV_8U);
	cvtColor(im, im_gray, COLOR_BGR2GRAY);
	//cvtColor(im, im_gray, COLOR_BGR2GRAY);
	//cout << im_gray.at<int>(1917, 1916) << "\n";
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//thresh = prepare_im(thresh);
	findContours(thresh, contours, hierarchy, RETR_TREE, method);
	int i = 0;
	for (auto it = contours.begin(); it != contours.end(); it++) {
		int ifat = hierarchy[i][3];
		if (ifat != -1) {
			contours[ifat].insert(contours[ifat].end(), contours[i].begin(), contours[i].end());
			contours.erase(it);
			it--;
			i--;
		}
		i++;
	}
	for (auto c : contours) {
		namedWindow("cnt", WINDOW_NORMAL);
		Mat im2 = im.clone();
		vector<vector<Point>> contoursnow(1);
		contoursnow[0] = c;
		draw_points_on_picture(im2, contoursnow);
		imshow("cnt", im2);
		waitKey(0);
	}
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
	double a1ths = 0.628, a2ths = 2.83;
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

void ellipses_selection(const vector<vector<Point>>& segments, 
	vector<vector<Point>> &for_refine, 
	vector<Ellipse>& for_combine, double disTh, Mat image) {
	for (int i = 0; i < segments.size(); i++) {
		if (segments[i].size() > 5) {
			Ellipse ellipse = Ellipse(segments[i]);
			vector<vector<Ellipse>> ime(1);
			vector<vector<Point>> imc(1, ellipse.self_segment);
			ime[0].push_back(ellipse);
			namedWindow("select", WINDOW_NORMAL);
			imshow("select", draw_ellipses(image.clone(), ime));
			namedWindow("selectc", WINDOW_NORMAL);
			draw_points_on_picture(image, imc);
			imshow("selectc", image);
			waitKey(0);
			if (is_ellipse_for_combine(ellipse, disTh)) {
				for_combine.push_back(ellipse);
				//cout << find_dis_segment_to_ellipse(ellipse.self_segment, ellipse.coefficents) << "\n";
			}
			else { 
				for_refine.push_back(segments[i]);
				cout << find_dis_segment_to_ellipse(ellipse.self_segment, ellipse.coefficents) << "\n";

			}
		}
	}
}

double find_dis_segment_to_ellipse(vector<Point> segment, Mat coefficents) {
	double dis = 0;
	//coefficents = coefficents / norm(coefficents);
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	//cout << A << " " << B << " " << C << " " << D << " " << E << " " << F << "\n";
	for (Point p: segment) {
		dis += abs(A * p.x * p.x + B * p.x * p.y + C * p.y * p.y + D * p.x + E * p.y + F);
	}
	dis = dis / segment.size();
	return dis;
}

bool is_ellipse_for_combine(Ellipse ellipse, double disTh) {
	double eTh = 0.333;
	return (find_dis_segment_to_ellipse(ellipse.self_segment, ellipse.coefficents) <= disTh and ellipse.Eratio >= eTh);
	
}

void combine_ellipses(vector<Ellipse>& ellipses, double dminTh, Mat image) {
	double eps = 0.1;
	for (int i = 0; i < ellipses.size(); i++) {
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
						find_dis_segment_to_ellipse(Li, Ei_coefs))
					<= eps and abs(find_dis_segment_to_ellipse(Nl, new_ellipse.coefficents) -
						find_dis_segment_to_ellipse(Lj, Ej_coefs))
					<= eps;
				if (case1 and not case3 or not case2) {
					cout << "bad\n" << case1 << " " << case2 << " " << case3 << "\n";
					continue;
				}
				else if (case2 or case3) {
					cout << case3 << "norm\n";
					cout << find_dis_segment_to_ellipse(Nl, new_ellipse.coefficents) << "\n";
					vector<vector<Ellipse>> start_ellipses(1);
					vector<vector<Ellipse>> end_ellipse(1);
					start_ellipses[0].push_back(ellipsei);
					start_ellipses[0].push_back(ellipsej);
					end_ellipse[0].push_back(new_ellipse);
					Mat im_s = draw_ellipses(image.clone(), start_ellipses);
					Mat im_d = draw_ellipses(image, end_ellipse);
					/*namedWindow("s", WINDOW_NORMAL);
					namedWindow("d", WINDOW_NORMAL);
					imshow("s", im_s);
					imshow("d", im_d);
					waitKey(0);*/
					ellipses[i] = new_ellipse;
					ellipses.erase(ellipses.begin() + j);
					i = 0;
					break;
				}
				//cout << "a\n";
			}
		}
	}
}

void refine_ellipses(vector<vector<Point>> &refine_segments, vector<Ellipse>& ellipses) {
	double disThRe = 0.0054;
	//std::cout << refine_segments.size() << "\n";
	//std::cout << ellipses.size() << "\n";
	for (int i = 0; i < refine_segments.size(); i++) {
		double mn_dist = INFINITY;
		Ellipse best_ellipse = Ellipse(vector<Point>());
		int index = 0;
		for (int j = 0; j < ellipses.size(); j++) {
			vector<Point> new_segment = ellipses[j].self_segment;
			for (Point p : refine_segments[i]) {
				new_segment.push_back(p);
			}
			Ellipse new_ellipse = Ellipse(new_segment);
			double dist = find_dis_segment_to_ellipse(new_segment, new_ellipse.coefficents);
			if (dist < mn_dist) {
				//cout << dist << " dst" << "\n";
				mn_dist = dist;
				best_ellipse = new_ellipse;
				index = j;
			}
		}
		if (mn_dist < disThRe) {
			ellipses[index] = best_ellipse;
			refine_segments.erase(refine_segments.begin() + i);
			i -= 1;
			cout << mn_dist << "\n";
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
					im.at<Vec3b>(y, x) = Vec3b(255, 0, 0);
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
			im.at<Vec3b>(cps[i][j].y, cps[i][j].x) = Vec3b(255, 0, 0);
		}
	}
}

vector<vector<Ellipse>> main_func(Mat im, float approx_thresh, int binary_thresh, double dTh, double disTh, double dminTh, string imgname) {
	clock_t start = clock();
	Mat im_gray;
	cvtColor(im, im_gray, COLOR_BGR2GRAY);
	Mat thresh;
	threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);
	vector<vector<Point>> contours = find_contours(im, binary_thresh, 3);
	vector<vector<Point>> contours_start = find_contours(im, binary_thresh, 1);
	vector<vector<Point>> cps;
	vector<vector<Ellipse>> all_ellipses;
	vector<vector<vector<Point>>> segments;
	vector<vector<Ellipse>> pre_ellipses;
	for (int i = 0; i < contours.size(); i++) {
		vector<Point> now_contour = contours[i];
		vector<Point> c1 = contours_start[i];
		approxPolyDP(contours[i], now_contour, approx_thresh, true);
		vector<vector<Point>> segments = make_segments_of_contour(c1, cpe(now_contour));
		cps.push_back(cpe(now_contour));
		vector<Ellipse> for_combine;
		vector<vector<Point>> for_refine;
		vector<Ellipse> for_refine_ellipses;
		for (auto s : segments) {
			for_refine_ellipses.push_back(Ellipse(s));
		}
		vector<Ellipse> ellipses;
		ellipses_selection(segments, for_refine, for_combine, disTh, im.clone());
		pre_ellipses.push_back(for_refine_ellipses);
		//combine_ellipses(for_combine, dminTh, im);
		/*refine_ellipses(for_refine, for_combine);
		vector<Ellipse> af_ref;
		for (auto s : for_refine) {
			af_ref.push_back(Ellipse(s));
		}*/
		all_ellipses.push_back(for_combine);
		//all_ellipses.push_back(af_ref);
		Mat imcopytime = im.clone();
		vector<vector<Point>> now_all_contour;
		now_all_contour.push_back(c1);
		draw_points_on_picture(imcopytime, cps);
		/*namedWindow("try", WINDOW_NORMAL);
		imshow("try", imcopytime);
		waitKey(0);*/
	}
	clock_t end = clock();
	cout << (double)(end - start) / CLOCKS_PER_SEC << "seconds\n";
	Mat impre = draw_ellipses(im.clone(), pre_ellipses);
	Mat imcps = im.clone();
	draw_points_on_picture(imcps, cps);
	//imshow("pre", impre);
	//imwrite("test_res/" + imgname.substr(0, imgname.size() - 4) + "/pre_" + to_string(binary_thresh) + imgname, impre);
	//imshow("cps", imcps);
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

