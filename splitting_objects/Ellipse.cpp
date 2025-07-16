#include "Ellipse.h"

using namespace std;
using namespace cv;

void print_mat(Mat mat) {
	for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			cout << mat.at<double>(i, j) << " ";
		}
		cout << "\n";
	}
}

void print_eigmat(Eigen::MatrixXcd mat) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cout << mat(i, j) << " ";
		}
		cout << "\n";
	}
}

Ellipse::Ellipse(vector<Point> segment) {
	self_segment = segment;
	if (segment.size() >= 5) {
		make_data(segment);
		try{
			calculate_coefs();
			make_self_contour();
			coefficents = coefficents;
			calc_self_deviation();
		}
		catch (...) {
			//vector<Point> print_segment = read_data();
			//cout << "trouble\n";
		}
	}
}
void Ellipse::calculate_coefs() {
	Mat XY = Mat(self_segment.size(), 2, CV_64F);
	int i = 0;
	double cx = 0, cy = 0;
	for (Point p : self_segment) {
		cx += p.x;
		cy += p.y;
		XY.at<double>(i, 0) = p.x;
		XY.at<double>(i, 1) = p.y;
		i += 1;
	}
	cx /= self_segment.size();
	cy /= self_segment.size();
	//cout << cx << " "<< cy << "\n";
	// Calculate the centroid of the data set

	// Construct matrices D1 and D2
	cv::Mat D1(XY.rows, 3, CV_64F);
	cv::Mat D2(XY.rows, 3, CV_64F);
	for (int i = 0; i < XY.rows; ++i) {
		double x = XY.at<double>(i, 0);
		double y = XY.at<double>(i, 1);
		D1.at<double>(i, 0) = (x - cx) * (x - cx);
		D1.at<double>(i, 1) = (x - cx) * (y - cy);
		D1.at<double>(i, 2) = (y - cy) * (y - cy);
		D2.at<double>(i, 0) = x - cx;
		D2.at<double>(i, 1) = y - cy;
		D2.at<double>(i, 2) = 1.0;
	}
	// 3. Âû÷èñëåíèå S1, S2 è S3
	Mat S1 = D1.t() * D1;  // D1' * D1
	Mat S2 = D1.t() * D2;  // D1' * D2
	Mat S3 = D2.t() * D2;  // D2' * D2
	// 4. Âû÷èñëåíèå T
	Mat T = -S3.inv() * S2.t();  // -inv(S3) * S2'

	// 5. Ñîçäàíèå ìàòðèöû M
	Mat M = S1 + S2 * T;
	// 6. Ïîäãîòîâêà M â òðåáóåìîì ôîðìàòå
	Eigen::MatrixXd E_M(3, M.cols);
	Mat M_final = Mat(3, M.cols, CV_64F);
	for (int i = 0; i < M.cols; i++) {
		E_M(0, i) = M.at<double>(2, i) / 2;
		E_M(1, i) = -M.at<double>(1, i);
		E_M(2, i) = M.at<double>(0, i) / 2;
		M_final.at<double>(0, i) = M.at<double>(2, i) / 2;
		M_final.at<double>(1, i) = M.at<double>(1, i) * (-1);
		M_final.at<double>(2, i) = M.at<double>(0, i) / 2;
	}

	// 7. Âû÷èñëåíèå ñîáñòâåííûõ âåêòîðîâ è çíà÷åíèé
	Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(E_M);
	Mat evec, eval;
	Eigen::MatrixXcd eigenvectors = eigenSolver.eigenvectors();
	//cv::eigenNonSymmetric(M_final, eval, evec);
	evec = Mat(eigenvectors.rows(), eigenvectors.cols(), CV_64F);
	//print_eigmat(eigenvectors);

	for (int j = 0; j < eigenvectors.cols(); j++) {
		for (int i = 0; i < eigenvectors.rows(); i++) {
			if (j == 0) {
				evec.at<double>(i, j) = -eigenvectors(i, eigenvectors.cols() - 1).real();
			}
			else {
				if (j - 1 == 0) {
					evec.at<double>(i, j) = eigenvectors(i, j - 1).real();
				}
				else {
					evec.at<double>(i, j) = -eigenvectors(i, j - 1).real();
				}
			}
		}
	}
	//print_mat(evec);

	// 8. Ïðîöåññ èçâëå÷åíèÿ íóæíûõ ñîáñòâåííûõ âåêòîðîâ
	Mat A1;
	Mat cond = 4 * evec.row(0).mul(evec.row(2)) - evec.row(1).mul(evec.row(1));
	//print_mat(cond);
	for (int i = 0; i < cond.cols; i++) {
		if (cond.at<double>(0, i) > 0) {
			A1.push_back(evec.col(i));
		}
	}
	//print_mat(A1);
	// 9. Ñîçäàíèå A
	Mat A = A1;
	Mat TA1 = T * A1;

	for (int i = 0; i < A1.rows; i++) {
		A.push_back(TA1.row(i));
	}
	//print_mat(A);
	/*for (int i = 0; i < A.rows; i++) {
		cout << A.at<double>(i, 0) << "\n";
	}*/
	double A3 = A.at<double>(3, 0) - 2 * A.at<double>(0, 0) * cx - A.at<double>(1, 0) * cy;
	double A4 = A.at<double>(4, 0) - 2 * A.at<double>(2, 0) * cy - A.at<double>(1, 0) * cx;
	double A5 = A.at<double>(5, 0) + A.at<double>(0, 0) * cx * cx + A.at<double>(2, 0) * cy * cy + A.at<double>(1, 0) * cx * cy - A.at<double>(3) * cx - A.at<double>(4) * cy;
	A.at<double>(3, 0) = A3;
	A.at<double>(4, 0) = A4;
	A.at<double>(5, 0) = A5;
	// Normalize A
	//A = A / norm(A);
	coefficents = A;
}
void Ellipse::make_self_contour() {
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	double e = 4 * A * C - B * B;

	double x0 = (B * E - 2 * C * D) / e;
	double y0 = (B * D - 2 * A * E) / e;
	center = Point2f(x0, y0);
	double F0 = -2 * (A * x0 * x0 + B * x0 * y0 + C * y0 * y0 + D * x0 + E * y0 + F);
	double g = sqrt((A - C) * (A - C) + B * B);
	double a = F0 / (A + C + g);
	double b = F0 / (A + C - g);
	a = sqrt(a);
	b = sqrt(b);
	minorAxisL = 2 * min(a, b);
	majorAxisL = 2 * max(a, b);
	Eratio = minorAxisL / majorAxisL;
	double t = 0.5 * atan2(B, A - C);
	double ct = cos(t); double st = sin(t);
	if (a > b) {
		map1 = Point2f( x0 + a * ct, y0 + a * st );
		map2 = Point2f( x0 - a * ct, y0 - a * st );
		mip1 = Point2f( x0 - b * st, y0 + b * ct );
		mip2 = Point2f( x0 + b * st, y0 - b * ct );
	}
	else {
		mip1 = Point2f( x0 + a * ct, y0 + a * st );
		mip2 = Point2f( x0 - a * ct, y0 - a * st );
		map1 = Point2f( x0 - b * st, y0 + b * ct );
		map2 = Point2f( x0 + b * st, y0 - b * ct );
	}
	int num = 50;
	double step = 0.2 * M_PI / num;
	for (double i = 0; i <= num; i += step) {
		double x;
		double y;
		double cp = cos(i); double sp = sin(i);
		// ïðèìåíÿåì ïîâîðîò
		x = x0 + a * ct * cp - b * st * sp;
		y = y0 + a * st * cp + b * ct * sp;

		// ñìåùàåì â öåíòð
		contour.push_back(Point2f(x, y));
	}
	square = M_PI * minorAxisL * majorAxisL / 4;
}
void Ellipse::calc_self_deviation() {
	double dis = 0;
	double disexp = 0;
	double A = coefficents.at<double>(0);
	double B = coefficents.at<double>(1);
	double C = coefficents.at<double>(2);
	double D = coefficents.at<double>(3);
	double E = coefficents.at<double>(4);
	double F = coefficents.at<double>(5);
	double a, b, c, p, S, r;
	c = sqrt(pow(map1.x - map2.x, 2) + pow(map1.y - map2.y, 2));
	a = sqrt(pow(mip1.x - map1.x, 2) + pow(mip1.y - map1.y, 2));
	b = sqrt(pow(mip1.x - map2.x, 2) + pow(mip1.y - map2.y, 2));
	p = (a + b + c) / 2;
	S = sqrt(p * (p - a) * (p - b) * (p - c));
	r = a * b * c / 4 / S;
	cout << "radius : " << r << "\n";
	for (Point p : self_segment) {
		int x = p.x, y = p.y;
		double min_dist = INFINITY;
		for (Point2f pc : contour) {
			int xc = pc.x, yc = pc.y;
			double now_dist = sqrt(pow(y - yc, 2) + pow(x - xc, 2));
			if (now_dist < min_dist) min_dist = now_dist;
		}
		dis += min_dist;
		double d1 = sqrt(pow(mip1.x - x, 2) + pow(mip1.y - y, 2)), d2 = sqrt(pow(mip2.x - x, 2) + pow(mip2.y - y, 2));
		/*if (d1 >= d2) {
			Point timepoint = mip1;
			mip1 = mip2;
			mip2 = timepoint;
		}*/
		Point2f nowcenter = findCircleCenter(map1, map2, mip1, r);
		double now_distexp = fabs(sqrt(pow(nowcenter.x - x, 2) + pow(nowcenter.y - y, 2)) - r);
		cout << nowcenter << " ";
		disexp += now_distexp;
		center_circle = nowcenter;
	}
	cout << "\n";
	dis = dis / self_segment.size();
	disexp = disexp / self_segment.size();
	deviation_of_segment = dis;
	cout << "|n||| " << dis << " dis segment to ellipse\n";
	cout << "dis experemental " << disexp << " |||\n";
	r_circle = r;
}

Point findCircleCenter(const Point& A, const Point& B, const Point& C, double R) {
	// Вычисляем коэффициенты системы уравнений
	double A1 = 2 * (B.x - A.x);
	double B1 = 2 * (B.y - A.y);
	double C1 = B.x * B.x + B.y * B.y - A.x * A.x - A.y * A.y;

	double A2 = 2 * (C.x - A.x);
	double B2 = 2 * (C.y - A.y);
	double C2 = C.x * C.x + C.y * C.y - A.x * A.x - A.y * A.y;

	// Решаем систему методом Крамера
	double det = A1 * B2 - A2 * B1;

	double x0 = (C1 * B2 - C2 * B1) / det;
	double y0 = (A1 * C2 - A2 * C1) / det;

	return Point(x0, y0);
}