#include "funcs.h"

using namespace std;
using namespace cv;

vector<Point> cpe(vector<Point> c) {
	vector<Point> cpc;
	double eps = 0.001;
	for (int i = 0; i < c.size(); i++) {
		if (i == 0) {
			double a1, a2;
			if ((c[c.size() - 1].x - c[i].x) == 0) {
				a1 = M_PI / 2;
			}
			else {
				a1 = atan(((c[c.size() - 1].y - c[i].y) / (c[c.size() - 1].x - c[i].x)));
			}
			if ((c[i + 1].x - c[i].x) == 0) {
				a2 = M_PI / 2;
			}
			else {
				a2 = atan(((c[i + 1].y - c[i].y) / (c[i + 1].x - c[i].x)));
			}
			double mcp;
			if (abs(a1 - a2) < M_PI) {
				mcp = abs(a1 - a2);
			}
			else {
				mcp = M_PI - abs(a1 - a2);
			}

			if (a1 < mcp < a2) {
				Point e, q;
				e.x = c[i + 1].x - c[c.size() - 1].x;
				e.y = c[i + 1].y - c[c.size() - 1].y;
				q.x = c[i].x - c[i + 1].x;
				q.y = c[i].y - c[i + 1].y;
				double r = e.x * q.y - e.y * q.x;
				if (abs(r) >= eps) {
					cpc.push_back(c[i]);
				}
			}

		}
		else if (i == c.size() - 1) {

			double a1, a2;
			if ((c[i - 1].x - c[i].x) == 0) {
				a1 = M_PI / 2;
			}
			else {
				a1 = atan(((c[i - 1].y - c[i].y) / (c[i - 1].x - c[i].x)));
			}
			if ((c[0].x - c[i].x) == 0) {
				a2 = M_PI / 2;
			}
			else {
				a2 = atan(((c[0].y - c[i].y) / (c[0].x - c[i].x)));
			}
			double mcp;
			if (abs(a1 - a2) < M_PI) {
				mcp = abs(a1 - a2);
			}
			else {
				mcp = M_PI - abs(a1 - a2);
			}
			if (a1 < mcp < a2) {
				Point e, q;
				e.x = c[0].x - c[i - 1].x;
				e.y = c[0].y - c[i - 1].y;
				q.x = c[i].x - c[0].x;
				q.y = c[i].y - c[0].y;
				double r = e.x * q.y - e.y * q.x;
				if (abs(r) >= eps) {
					cpc.push_back(c[i]);
				}
			}
		}
		else {
			
			double a1, a2; 
			if ((c[i - 1].x - c[i].x) == 0) {
				a1 = M_PI / 2;
			}
			else {
				a1 = atan(((c[i - 1].y - c[i].y) / (c[i - 1].x - c[i].x)));
			}
			if ((c[i + 1].x - c[i].x) == 0) {
				a2 = M_PI / 2;
			}
			else {
				a2 = atan(((c[i + 1].y - c[i].y) / (c[i + 1].x - c[i].x)));
			}
			double mcp;
			if (abs(a1 - a2) < M_PI) {
				mcp = abs(a1 - a2);
			}
			else {
				mcp = M_PI - abs(a1 - a2);
			}
			if (a1 < mcp < a2) {

				Point e, q;
				e.x = c[i + 1].x - c[i - 1].x;
				e.y = c[i + 1].y - c[i - 1].y;
				q.x = c[i].x - c[i + 1].x;
				q.y = c[i].y - c[i + 1].y;
				double r = e.x*q.y - e.y*q.x;
				if (abs(r) >= eps) {
					cpc.push_back(c[i]);
				}
			}
		}
	}
	return cpc;
}