#include "Ellipse.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

using namespace std;
using namespace cv;

namespace {
    constexpr double kEps = 1e-12;
    constexpr int kDistanceSearchIterations = 36;

    inline double sqr(double value) {
        return value * value;
    }

    inline bool is_valid_positive(double value) {
        return std::isfinite(value) && value > kEps;
    }

    template <typename T>
    inline const T& clamp_value(const T& value, const T& low, const T& high) {
        return value < low ? low : (value > high ? high : value);
    }

    double point_to_axis_aligned_ellipse_distance_sq(double x, double y, double a, double b) {
        x = std::abs(x);
        y = std::abs(y);

        if (!is_valid_positive(a) || !is_valid_positive(b)) {
            return std::numeric_limits<double>::infinity();
        }

        if (a < b) {
            std::swap(a, b);
            std::swap(x, y);
        }

        auto distance_sq = [x, y, a, b](double t) {
            const double ct = std::cos(t);
            const double st = std::sin(t);
            const double dx = a * ct - x;
            const double dy = b * st - y;
            return dx * dx + dy * dy;
            };

        double left = 0.0;
        double right = CV_PI * 0.5;

        for (int iter = 0; iter < kDistanceSearchIterations; ++iter) {
            const double third = (right - left) / 3.0;
            const double m1 = left + third;
            const double m2 = right - third;
            if (distance_sq(m1) <= distance_sq(m2)) {
                right = m2;
            }
            else {
                left = m1;
            }
        }

        return distance_sq((left + right) * 0.5);
    }

    void reset_invalid_ellipse(Ellipse& ellipse) {
        ellipse.contour.clear();
        ellipse.coefficents = Mat::zeros(6, 1, CV_64F);
        ellipse.Eratio = 0.0;
        ellipse.minorAxisL = 0.0;
        ellipse.majorAxisL = 0.0;
        ellipse.square = 0.0;
        ellipse.deviation_of_segment = 100.0;
        ellipse.center = Point2f(0.f, 0.f);
        ellipse.map1 = Point2f(0.f, 0.f);
        ellipse.map2 = Point2f(0.f, 0.f);
        ellipse.mip1 = Point2f(0.f, 0.f);
        ellipse.mip2 = Point2f(0.f, 0.f);
    }
} // namespace

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
    self_segment = std::move(segment);
    coefficents = Mat::zeros(6, 1, CV_64F);

    if (self_segment.size() < 5) {
        return;
    }

    try {
        calculate_coefs();
        make_self_contour();
        if (is_valid_positive(square)) {
            coefficents = coefficents / square;
        }
        calc_self_deviation();
    }
    catch (...) {
        reset_invalid_ellipse(*this);
    }
}

void Ellipse::calculate_coefs() {
    const int n = static_cast<int>(self_segment.size());
    if (n < 5) {
        throw std::runtime_error("Not enough points for ellipse fitting");
    }

    double cx = 0.0;
    double cy = 0.0;
    for (const Point& p : self_segment) {
        cx += p.x;
        cy += p.y;
    }
    cx /= n;
    cy /= n;

    Mat D1(n, 3, CV_64F);
    Mat D2(n, 3, CV_64F);

    for (int i = 0; i < n; ++i) {
        const double x = self_segment[i].x - cx;
        const double y = self_segment[i].y - cy;

        D1.at<double>(i, 0) = x * x;
        D1.at<double>(i, 1) = x * y;
        D1.at<double>(i, 2) = y * y;
        D2.at<double>(i, 0) = x;
        D2.at<double>(i, 1) = y;
        D2.at<double>(i, 2) = 1.0;
    }

    const Mat S1 = D1.t() * D1;
    const Mat S2 = D1.t() * D2;
    const Mat S3 = D2.t() * D2;

    Mat T;
    if (!solve(S3, -S2.t(), T, DECOMP_SVD)) {
        throw std::runtime_error("Failed to solve ellipse fitting system");
    }

    const Mat M = S1 + S2 * T;

    Eigen::Matrix3d E_M;
    for (int i = 0; i < 3; ++i) {
        E_M(0, i) = M.at<double>(2, i) * 0.5;
        E_M(1, i) = -M.at<double>(1, i);
        E_M(2, i) = M.at<double>(0, i) * 0.5;
    }

    Eigen::EigenSolver<Eigen::Matrix3d> eigenSolver(E_M);
    const Eigen::MatrixXcd eigenvectors = eigenSolver.eigenvectors();

    Mat evec(3, 3, CV_64F);
    for (int j = 0; j < eigenvectors.cols(); ++j) {
        for (int i = 0; i < eigenvectors.rows(); ++i) {
            if (j == 0) {
                evec.at<double>(i, j) = -eigenvectors(i, eigenvectors.cols() - 1).real();
            }
            else if (j == 1) {
                evec.at<double>(i, j) = eigenvectors(i, 0).real();
            }
            else {
                evec.at<double>(i, j) = -eigenvectors(i, j - 1).real();
            }
        }
    }

    int valid_col = -1;
    for (int i = 0; i < evec.cols; ++i) {
        const double a = evec.at<double>(0, i);
        const double b = evec.at<double>(1, i);
        const double c = evec.at<double>(2, i);
        if (4.0 * a * c - b * b > 0.0) {
            valid_col = i;
            break;
        }
    }

    if (valid_col < 0) {
        throw std::runtime_error("No valid ellipse eigenvector found");
    }

    const Mat A1 = evec.col(valid_col).clone();
    const Mat TA1 = T * A1;

    Mat A(6, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        A.at<double>(i, 0) = A1.at<double>(i, 0);
        A.at<double>(i + 3, 0) = TA1.at<double>(i, 0);
    }

    const double A0 = A.at<double>(0, 0);
    const double B0 = A.at<double>(1, 0);
    const double C0 = A.at<double>(2, 0);
    const double D0 = A.at<double>(3, 0);
    const double E0 = A.at<double>(4, 0);
    const double F0 = A.at<double>(5, 0);

    A.at<double>(3, 0) = D0 - 2.0 * A0 * cx - B0 * cy;
    A.at<double>(4, 0) = E0 - 2.0 * C0 * cy - B0 * cx;
    A.at<double>(5, 0) = F0 + A0 * cx * cx + C0 * cy * cy + B0 * cx * cy - D0 * cx - E0 * cy;

    coefficents = A;
}

void Ellipse::make_self_contour() {
    contour.clear();

    const double A = coefficents.at<double>(0, 0);
    const double B = coefficents.at<double>(1, 0);
    const double C = coefficents.at<double>(2, 0);
    const double D = coefficents.at<double>(3, 0);
    const double E = coefficents.at<double>(4, 0);
    const double F = coefficents.at<double>(5, 0);

    const double e = 4.0 * A * C - B * B;
    if (std::abs(e) <= kEps) {
        throw std::runtime_error("Degenerate conic");
    }

    const double x0 = (B * E - 2.0 * C * D) / e;
    const double y0 = (B * D - 2.0 * A * E) / e;
    center = Point2f(static_cast<float>(x0), static_cast<float>(y0));

    const double F0 = -2.0 * (A * x0 * x0 + B * x0 * y0 + C * y0 * y0 + D * x0 + E * y0 + F);
    const double g = std::sqrt((A - C) * (A - C) + B * B);
    const double denom1 = A + C + g;
    const double denom2 = A + C - g;

    if (std::abs(denom1) <= kEps || std::abs(denom2) <= kEps) {
        throw std::runtime_error("Invalid ellipse axes denominator");
    }

    const double a_sq = F0 / denom1;
    const double b_sq = F0 / denom2;
    if (a_sq <= kEps || b_sq <= kEps) {
        throw std::runtime_error("Invalid ellipse axes");
    }

    const double a = std::sqrt(a_sq);
    const double b = std::sqrt(b_sq);

    minorAxisL = 2.0 * std::min(a, b);
    majorAxisL = 2.0 * std::max(a, b);
    if (!is_valid_positive(minorAxisL) || !is_valid_positive(majorAxisL)) {
        throw std::runtime_error("Non-positive ellipse axes");
    }

    Eratio = minorAxisL / majorAxisL;

    const double t = 0.5 * std::atan2(B, A - C);
    const double ct = std::cos(t);
    const double st = std::sin(t);

    if (a > b) {
        map1 = Point2f(static_cast<float>(x0 + a * ct), static_cast<float>(y0 + a * st));
        map2 = Point2f(static_cast<float>(x0 - a * ct), static_cast<float>(y0 - a * st));
        mip1 = Point2f(static_cast<float>(x0 - b * st), static_cast<float>(y0 + b * ct));
        mip2 = Point2f(static_cast<float>(x0 + b * st), static_cast<float>(y0 - b * ct));
    }
    else {
        mip1 = Point2f(static_cast<float>(x0 + a * ct), static_cast<float>(y0 + a * st));
        mip2 = Point2f(static_cast<float>(x0 - a * ct), static_cast<float>(y0 - a * st));
        map1 = Point2f(static_cast<float>(x0 - b * st), static_cast<float>(y0 + b * ct));
        map2 = Point2f(static_cast<float>(x0 + b * st), static_cast<float>(y0 - b * ct));
    }

    const int num = clamp_value(static_cast<int>(std::ceil(CV_PI * std::max(a, b))), 72, 360);
    contour.reserve(num);
    const double step = 2.0 * CV_PI / static_cast<double>(num);

    for (int i = 0; i < num; ++i) {
        const double angle = i * step;
        const double cp = std::cos(angle);
        const double sp = std::sin(angle);
        const double x = x0 + a * ct * cp - b * st * sp;
        const double y = y0 + a * st * cp + b * ct * sp;
        contour.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }

    square = CV_PI * minorAxisL * majorAxisL * 0.25;
    if (!is_valid_positive(square)) {
        throw std::runtime_error("Invalid ellipse area");
    }
}

void Ellipse::calc_self_deviation() {
    if (self_segment.empty()) {
        deviation_of_segment = 100.0;
        return;
    }

    const double semiMajor = majorAxisL * 0.5;
    const double semiMinor = minorAxisL * 0.5;
    if (!is_valid_positive(semiMajor) || !is_valid_positive(semiMinor)) {
        deviation_of_segment = 100.0;
        return;
    }

    const Point2f majorVector = map1 - center;
    const Point2f minorVector = mip1 - center;

    const double majorNorm = std::hypot(static_cast<double>(majorVector.x), static_cast<double>(majorVector.y));
    const double minorNorm = std::hypot(static_cast<double>(minorVector.x), static_cast<double>(minorVector.y));
    if (!is_valid_positive(majorNorm) || !is_valid_positive(minorNorm)) {
        deviation_of_segment = 100.0;
        return;
    }

    const double ux = majorVector.x / majorNorm;
    const double uy = majorVector.y / majorNorm;
    const double vx = minorVector.x / minorNorm;
    const double vy = minorVector.y / minorNorm;

    double dis = 0.0;
    for (const Point& p : self_segment) {
        const double dx = p.x - center.x;
        const double dy = p.y - center.y;
        const double localX = std::abs(dx * ux + dy * uy);
        const double localY = std::abs(dx * vx + dy * vy);
        dis += std::sqrt(point_to_axis_aligned_ellipse_distance_sq(localX, localY, semiMajor, semiMinor));
    }

    deviation_of_segment = dis / static_cast<double>(self_segment.size());
}

Point findCircleCenter(const Point& A, const Point& B, const Point& C, double R) {
    (void)R;

    const double A1 = 2.0 * (B.x - A.x);
    const double B1 = 2.0 * (B.y - A.y);
    const double C1 = B.x * B.x + B.y * B.y - A.x * A.x - A.y * A.y;

    const double A2 = 2.0 * (C.x - A.x);
    const double B2 = 2.0 * (C.y - A.y);
    const double C2 = C.x * C.x + C.y * C.y - A.x * A.x - A.y * A.y;

    const double det = A1 * B2 - A2 * B1;
    if (std::abs(det) <= kEps) {
        return Point(0, 0);
    }

    const double x0 = (C1 * B2 - C2 * B1) / det;
    const double y0 = (A1 * C2 - A2 * C1) / det;

    return Point(cvRound(x0), cvRound(y0));
}
