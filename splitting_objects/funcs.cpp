#include "funcs.h"

#include <algorithm>
#include <iterator>
#include <limits>

using namespace cv;
using namespace std;

namespace {
    inline Mat to_gray(const Mat& image) {
        if (image.channels() == 1) {
            return image.clone();
        }
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        return gray;
    }

    inline bool is_inner_point(const Point& p, const pair<int, int>& image_size) {
        return p.x != 0 && p.x != image_size.first - 1 && p.y != 0 && p.y != image_size.second - 1;
    }

    inline double point_distance(const Point2f& a, const Point2f& b) {
        return std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
    }

    template <typename T>
    inline const T& clamp_value(const T& value, const T& low, const T& high) {
        return value < low ? low : (value > high ? high : value);
    }

    void refine_ellipses_impl(vector<Ellipse>& refine_segments, vector<Ellipse>& ellipses) {
        const double disThRe = 1.0;
        if (refine_segments.empty() || ellipses.empty()) {
            return;
        }

        for (size_t i = 0; i < refine_segments.size(); ++i) {
            double mn_dist = std::numeric_limits<double>::infinity();
            Ellipse best_ellipse(vector<Point>{});
            size_t best_index = 0;

            for (size_t j = 0; j < ellipses.size(); ++j) {
                vector<Point> new_segment;
                new_segment.reserve(ellipses[j].self_segment.size() + refine_segments[i].self_segment.size());
                new_segment.insert(new_segment.end(), ellipses[j].self_segment.begin(), ellipses[j].self_segment.end());
                new_segment.insert(new_segment.end(), refine_segments[i].self_segment.begin(), refine_segments[i].self_segment.end());

                Ellipse new_ellipse(new_segment);
                if (new_ellipse.majorAxisL <= 0.0) {
                    continue;
                }

                if (new_ellipse.deviation_of_segment < mn_dist) {
                    mn_dist = new_ellipse.deviation_of_segment;
                    best_ellipse = std::move(new_ellipse);
                    best_index = j;
                }
            }

            if (mn_dist < disThRe) {
                ellipses[best_index] = std::move(best_ellipse);
            }
        }
    }
} // namespace

void check_is_files_identic(string name1, string name2) {
    ifstream file1(name1), file2(name2);
    string line1, line2;
    while (getline(file1, line1)) {
        getline(file2, line2);
        if (line1 != line2) {
            cout << line1 << "###" << line2 << "###err\n";
        }
    }
}

void write_contours_to_file(vector<vector<Point>> contours, string filename) {
    ofstream file(filename);
    for (const auto& contour : contours) {
        for (const Point& p : contour) {
            file << p << "\n";
        }
    }
}

void write_segments_to_file(vector<vector<vector<Point>>> segments, string filename) {
    ofstream file(filename);
    for (const auto& contour : segments) {
        for (const auto& segment : contour) {
            for (const auto& p : segment) {
                file << p << "\n";
            }
        }
    }
}

vector<vector<vector<Point>>> contour_processing_return_segments(Mat im, int binary_thresh, int method_for_cpe, int method_for_segments, float approx_thresh) {
    (void)method_for_cpe;

    const pair<int, int> image_size = { im.cols, im.rows };
    const Mat im_gray = to_gray(im);

    Mat thresh;
    threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, method_for_segments);

    vector<vector<vector<Point>>> segments(contours.size());
    vector<char> keep(contours.size(), 1);

    for (size_t i = 0; i < contours.size(); ++i) {
        vector<Point> approx_contour;
        approxPolyDP(contours[i], approx_contour, approx_thresh, true);
        segments[i] = make_segments_of_contour(contours[i], cpe(approx_contour), image_size);
    }

    for (size_t i = 0; i < contours.size(); ++i) {
        const int parent = hierarchy[i][3];
        if (parent != -1) {
            auto& dst = segments[parent];
            auto& src = segments[i];
            dst.insert(dst.end(), std::make_move_iterator(src.begin()), std::make_move_iterator(src.end()));
            keep[i] = 0;
        }
    }

    vector<vector<vector<Point>>> result;
    result.reserve(contours.size());
    for (size_t i = 0; i < segments.size(); ++i) {
        if (keep[i]) {
            result.push_back(std::move(segments[i]));
        }
    }

    return result;
}

vector<vector<Point>> find_contours(Mat im, int binary_thresh, int method) {
    const Mat im_gray = to_gray(im);

    Mat thresh;
    threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, method);

    vector<vector<Point>> merged(contours.size());
    vector<char> keep(contours.size(), 1);
    for (size_t i = 0; i < contours.size(); ++i) {
        merged[i] = contours[i];
    }

    for (size_t i = 0; i < contours.size(); ++i) {
        const int parent = hierarchy[i][3];
        if (parent != -1) {
            merged[parent].insert(merged[parent].end(), contours[i].begin(), contours[i].end());
            keep[i] = 0;
        }
    }

    vector<vector<Point>> result;
    result.reserve(contours.size());
    for (size_t i = 0; i < merged.size(); ++i) {
        if (keep[i]) {
            result.push_back(std::move(merged[i]));
        }
    }

    return result;
}

// функция по расчету угла между двумя векторами
double angleBetween(const Point& a, const Point& b) {
    const double magnitudeA = std::sqrt(static_cast<double>(a.x) * a.x + static_cast<double>(a.y) * a.y);
    const double magnitudeB = std::sqrt(static_cast<double>(b.x) * b.x + static_cast<double>(b.y) * b.y);
    if (magnitudeA <= 1e-12 || magnitudeB <= 1e-12) {
        return 0.0;
    }

    double cosValue = (static_cast<double>(a.x) * b.x + static_cast<double>(a.y) * b.y) / (magnitudeA * magnitudeB);
    cosValue = clamp_value(cosValue, -1.0, 1.0);
    return std::acos(cosValue);
}

// функция, которая проверяет слева или справа p1 от прямой p-p2
bool classify_point_by_straight(Point p, Point p1, Point p2) {
    const Point a = p2 - p1;
    const Point b = p - p1;
    const double sa = static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    if (sa > 0.0) {
        return false;
    }
    if (sa < 0.0) {
        return true;
    }
    return false;
}

// функция, которая находит угловые точки (и выпуклые и вогнутые)
vector<Point> find_corner_points(vector<Point> contour, double threshold1, double threshold2) {
    vector<Point> corners;
    if (contour.size() < 3) {
        return corners;
    }

    corners.reserve(contour.size() / 4 + 1);
    for (size_t i = 0; i < contour.size(); ++i) {
        const size_t ib = (i + contour.size() - 1) % contour.size();
        const size_t inow = i;
        const size_t ia = (i + 1) % contour.size();

        const Point v1 = contour[inow] - contour[ib];
        const Point v2 = contour[ia] - contour[inow];

        const double magnitudeV1 = std::sqrt(static_cast<double>(v1.x) * v1.x + static_cast<double>(v1.y) * v1.y);
        const double magnitudeV2 = std::sqrt(static_cast<double>(v2.x) * v2.x + static_cast<double>(v2.y) * v2.y);

        if (magnitudeV1 > 0.0 && magnitudeV2 > 0.0) {
            const double angle = angleBetween(v1, v2);
            if (angle > threshold1 && angle < threshold2) {
                corners.push_back(contour[inow]);
            }
        }
    }

    return corners;
}

// функция, которая выбирает из предыдущей вогнутые точки
vector<Point> cpe(vector<Point> c) {
    vector<Point> cpc;
    if (c.size() < 3) {
        return cpc;
    }

    const double a1ths = 0.628;
    const double a2ths = 2.83;
    const vector<Point> mcps = find_corner_points(c, a1ths, a2ths);
    if (mcps.empty()) {
        return cpc;
    }

    size_t j = 0;
    for (size_t i = 0; i < c.size() && j < mcps.size(); ++i) {
        const size_t pre = (i + c.size() - 1) % c.size();
        const size_t now = i;
        const size_t next = (i + 1) % c.size();

        if (c[now] == mcps[j]) {
            if (!classify_point_by_straight(c[pre], c[now], c[next])) {
                cpc.push_back(c[now]);
            }
            ++j;
        }
    }

    return cpc;
}

vector<vector<Point>> make_segments_of_contour(vector<Point> c, vector<Point> cps, pair<int, int> image_size) {
    vector<vector<Point>> segments(1);

    if (c.empty()) {
        return segments;
    }

    if (cps.empty()) {
        segments[0].reserve(c.size());
        for (const Point& p : c) {
            if (is_inner_point(p, image_size)) {
                segments[0].push_back(p);
            }
        }
        return segments;
    }

    size_t j = 0;
    int paste_i = 0;
    bool is_at_edge_now = false;

    for (size_t i = 0; i < c.size(); ++i) {
        if (j < cps.size() && cps[j] == c[i]) {
            if (j == cps.size() - 1) {
                paste_i = 0;
            }
            else {
                segments.emplace_back();
                ++paste_i;
                ++j;
            }
        }

        if (is_inner_point(c[i], image_size)) {
            is_at_edge_now = false;
            segments[paste_i].push_back(c[i]);
        }
        else if (!is_at_edge_now) {
            is_at_edge_now = true;
            segments.emplace_back();
            ++paste_i;
        }
    }

    return segments;
}

void ellipses_selection(const vector<vector<Point>>& segments,
    vector<Ellipse>& for_refine,
    vector<Ellipse>& for_combine, double disTh, Mat image) {
    (void)image;

    for_refine.reserve(for_refine.size() + segments.size());
    for_combine.reserve(for_combine.size() + segments.size());

    for (const auto& segment : segments) {
        if (segment.size() <= 15) {
            continue;
        }

        Ellipse ellipse(segment);
        if (ellipse.majorAxisL <= 0.0) {
            continue;
        }
        Mat image_copy = draw_ellipses(image.clone(), { {ellipse} });
        cout << ellipse.deviation_of_segment << ": deviation_this\n";
        if (ellipse.deviation_of_segment > 3) {
            //make_data(cps[0]);
        //Mat cps_img = draw_points_on_picture(im, cps);
            namedWindow("out", WINDOW_NORMAL);
            //namedWindow("cps", WINDOW_NORMAL);
            //namedWindow("cntrs", WINDOW_NORMAL);
            //imshow("cntrs", img_contours);
            //imshow("cps", cps_img);
            imshow("out", image_copy);
            waitKey(0);

        }
        
        if (is_ellipse_for_combine(ellipse, disTh)) {
            for_combine.push_back(std::move(ellipse));
        }
        else {
            for_refine.push_back(std::move(ellipse));
        }
    }
}

bool is_ellipse_for_combine(Ellipse ellipse, double disTh) {
    const double eTh = 0.0;
    return ellipse.deviation_of_segment <= disTh * 100 && ellipse.Eratio >= eTh;
}

void combine_ellipses(vector<Ellipse>& ellipses, double dminTh, Mat image) {
    (void)image;

    if (ellipses.size() < 2) {
        return;
    }

    const double kHugePenalty = 1e9;

    // Базовые ограничения, но мягкие
    const double kMinERatio = 0.06;
    const double kMaxCenterShift = 4.5 * dminTh;
    const double kMaxOldCenters = 5.5 * dminTh;
    const double kMaxDeviationAbs = 2.0;
    const double kMaxDeviationRel = 0.95;
    const double kMaxAxisGrowth = 3.2;
    const double kMaxGap = 5.5 * dminTh;
    const double kMaxAngleDeg = 70.0;              // угол между осями старых эллипсов

    auto length = [](const Point2f& v) -> double {
        return std::sqrt(static_cast<double>(v.x) * v.x + static_cast<double>(v.y) * v.y);
        };

    auto ellipse_angle_deg = [&](const Ellipse& e) -> double {
        Point2f v = e.map1 - e.center;
        if (length(v) < 1e-9) {
            return 0.0;
        }
        double ang = std::atan2(static_cast<double>(v.y), static_cast<double>(v.x)) * 180.0 / CV_PI;
        if (ang < 0.0) ang += 180.0;
        return ang;
        };

    auto angle_diff_deg = [&](double a1, double a2) -> double {
        double d = std::fabs(a1 - a2);
        if (d > 90.0) d = 180.0 - d;
        return d;
        };

    auto min_segment_distance = [&](const vector<Point>& s1, const vector<Point>& s2) -> double {
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < s1.size(); ++i) {
            for (size_t j = 0; j < s2.size(); ++j) {
                const double dx = static_cast<double>(s1[i].x - s2[j].x);
                const double dy = static_cast<double>(s1[i].y - s2[j].y);
                const double d = std::sqrt(dx * dx + dy * dy);
                if (d < best) {
                    best = d;
                }
            }
        }
        return best;
        };

    auto allowed_deviation = [&](double base_dev) -> double {
        return std::max(base_dev + kMaxDeviationAbs, base_dev * (1.0 + kMaxDeviationRel));
        };

    auto score_merge = [&](const Ellipse& e1, const Ellipse& e2, const Ellipse& merged) -> double {
        if (merged.majorAxisL <= 0.0 || merged.minorAxisL <= 0.0) {
            return -kHugePenalty;
        }
        if (!std::isfinite(merged.deviation_of_segment) || !std::isfinite(merged.Eratio)) {
            return -kHugePenalty;
        }
        if (merged.Eratio < kMinERatio) {
            return -kHugePenalty;
        }

        const double old_centers_dist = point_distance(e1.center, e2.center);
        const double shift1 = point_distance(merged.center, e1.center);
        const double shift2 = point_distance(merged.center, e2.center);
        const double gap = min_segment_distance(e1.self_segment, e2.self_segment);

        const double ang1 = ellipse_angle_deg(e1);
        const double ang2 = ellipse_angle_deg(e2);
        const double angm = ellipse_angle_deg(merged);

        const double old_angle_diff = angle_diff_deg(ang1, ang2);
        const double merged_to_1 = angle_diff_deg(angm, ang1);
        const double merged_to_2 = angle_diff_deg(angm, ang2);

        const double max_old_major = std::max(e1.majorAxisL, e2.majorAxisL);
        const double max_old_minor = std::max(e1.minorAxisL, e2.minorAxisL);

        const double old_best = std::min(e1.deviation_of_segment, e2.deviation_of_segment);
        const double old_avg = 0.5 * (e1.deviation_of_segment + e2.deviation_of_segment);
        const double dev_limit = allowed_deviation(old_best);

        // Оставляем только реально аварийные отбрасывания
        if (gap > 7.0 * dminTh) {
            return -kHugePenalty;
        }
        if (merged.majorAxisL > 3.8 * max_old_major) {
            return -kHugePenalty;
        }
        if (merged.deviation_of_segment > dev_limit + 2.0) {
            return -kHugePenalty;
        }

        double score = 0.0;

        // Главный критерий: merged должен быть не сильно хуже старых
        score += 6.0 * (old_avg - merged.deviation_of_segment);

        // Близость сегментов
        score += 2.4 * (1.0 - std::min(gap / std::max(1.0, 5.0 * dminTh), 1.0));

        // Похожесть направлений
        score += 1.4 * (1.0 - std::min(old_angle_diff / 70.0, 1.0));
        score += 0.9 * (1.0 - std::min((merged_to_1 + merged_to_2) / 120.0, 1.0));

        // Штрафы вместо жёстких запретов
        score -= 0.18 * old_centers_dist / std::max(1.0, 4.5 * dminTh);
        score -= 0.22 * (shift1 + shift2) / std::max(1.0, 4.5 * dminTh);

        score -= 0.30 * std::max(0.0, (merged.majorAxisL - max_old_major) / std::max(1.0, max_old_major));
        score -= 0.18 * std::max(0.0, (merged.minorAxisL - max_old_minor) / std::max(1.0, max_old_minor));

        const double major_diff_norm =
            std::fabs(e1.majorAxisL - e2.majorAxisL) / std::max(1.0, max_old_major);
        const double minor_diff_norm =
            std::fabs(e1.minorAxisL - e2.minorAxisL) / std::max(1.0, max_old_minor);

        score -= 0.15 * major_diff_norm;
        score -= 0.10 * minor_diff_norm;

        // Небольшой бонус, если merged реально улучшает фит
        if (merged.deviation_of_segment < old_best) {
            score += 0.8;
        }

        return score;
        };

    bool merged_any = true;
    while (merged_any) {
        merged_any = false;

        double best_score = 0.0;
        size_t best_i = 0;
        size_t best_j = 0;
        Ellipse best_merged(vector<Point>{});

        for (size_t i = 0; i < ellipses.size(); ++i) {
            if (ellipses[i].self_segment.size() < 5 || ellipses[i].majorAxisL <= 0.0) {
                continue;
            }

            for (size_t j = i + 1; j < ellipses.size(); ++j) {
                if (ellipses[j].self_segment.size() < 5 || ellipses[j].majorAxisL <= 0.0) {
                    continue;
                }

                vector<Point> merged_segment;
                merged_segment.reserve(ellipses[i].self_segment.size() + ellipses[j].self_segment.size());
                merged_segment.insert(merged_segment.end(),
                    ellipses[i].self_segment.begin(), ellipses[i].self_segment.end());
                merged_segment.insert(merged_segment.end(),
                    ellipses[j].self_segment.begin(), ellipses[j].self_segment.end());

                Ellipse merged_ellipse(merged_segment);
                const double score = score_merge(ellipses[i], ellipses[j], merged_ellipse);

                if (score > best_score) {
                    best_score = score;
                    best_i = i;
                    best_j = j;
                    best_merged = std::move(merged_ellipse);
                }
            }
        }

        // Порог merge: только реально полезные объединения
        if (best_score > 0.01) {
            ellipses[best_i] = std::move(best_merged);
            ellipses.erase(ellipses.begin() + static_cast<vector<Ellipse>::difference_type>(best_j));
            merged_any = true;
        }
    }
}


void refine_ellipses(vector<Ellipse>& refine_segments, vector<Ellipse>& ellipses, Mat image) {
    (void)image;
    refine_ellipses_impl(refine_segments, ellipses);
}

void refine_ellipses(vector<vector<Point>>& refine_segments, vector<Ellipse>& ellipses) {
    vector<Ellipse> prepared;
    prepared.reserve(refine_segments.size());
    for (const auto& segment : refine_segments) {
        if (segment.size() >= 5) {
            prepared.emplace_back(segment);
        }
    }
    refine_ellipses_impl(prepared, ellipses);
}

// функция, находящая все вогнутые точки из картинки
vector<vector<Point>> find_all_concave_points(Mat im, double approx_thresh, int binary_thresh) {
    const Mat im_gray = to_gray(im);

    Mat thresh;
    threshold(im_gray, thresh, binary_thresh, 255, THRESH_BINARY);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_KCOS);

    vector<vector<Point>> result;
    result.reserve(contours.size());

    for (const auto& contour : contours) {
        vector<Point> approx_contour;
        approxPolyDP(contour, approx_contour, approx_thresh, true);
        vector<Point> concave_points = cpe(approx_contour);
        if (!concave_points.empty()) {
            result.push_back(std::move(concave_points));
        }
    }

    return result;
}

Mat draw_ellipses(Mat im, vector<vector<Ellipse>> ellipses) {
    for (const auto& ellipse_group : ellipses) {
        for (const auto& ellipse : ellipse_group) {
            if (ellipse.contour.size() < 2) {
                continue;
            }

            vector<Point> polyline;
            polyline.reserve(ellipse.contour.size());
            for (const Point2f& p : ellipse.contour) {
                polyline.emplace_back(cvRound(p.x), cvRound(p.y));
            }
            polylines(im, polyline, true, Scalar(0, 0, 255), 3, LINE_8);
        }
    }

    return im;
}

// функция по рисованию вогнутых точек на картинке красным цветом
void draw_points_on_picture(Mat& im, vector<vector<Point>> cps) {
    for (const auto& contour_points : cps) {
        for (const Point& p : contour_points) {
            if (p.y >= 0 && p.x >= 0 && p.y < im.rows && p.x < im.cols) {
                im.at<Vec3b>(p.y, p.x) = Vec3b(0, 0, 255);
            }
        }
    }
}

vector<vector<Ellipse>> main_func(Mat im, float approx_thresh, int binary_thresh, double dTh, double disTh, double dminTh, string imgname) {
    (void)dTh;
    (void)imgname;

    const clock_t start = clock();
    vector<vector<Ellipse>> all_ellipses;

    vector<vector<vector<Point>>> segments = contour_processing_return_segments(im, binary_thresh, CHAIN_APPROX_TC89_KCOS, CHAIN_APPROX_NONE, approx_thresh);
    all_ellipses.reserve(segments.size());

    for (const auto& contour_segments : segments) {
        vector<Ellipse> for_combine;
        vector<Ellipse> for_refine;
        ellipses_selection(contour_segments, for_refine, for_combine, disTh, im);
        combine_ellipses(for_combine, dminTh, im);
        all_ellipses.push_back(std::move(for_combine));
    }

    const clock_t end = clock();
    cout << static_cast<double>(end - start) / CLOCKS_PER_SEC << "seconds\n";
    return all_ellipses;
}

Mat prepare_im(Mat im_binary) {
    const int white = countNonZero(im_binary);
    const int black = static_cast<int>(im_binary.total()) - white;
    if (white > black) {
        Mat inverted;
        bitwise_not(im_binary, inverted);
        return inverted;
    }
    return im_binary.clone();
}
