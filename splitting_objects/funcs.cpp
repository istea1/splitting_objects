#include "funcs.h"

using namespace std;
using namespace cv;

vector<Point> cpe(vector<Point> c) {
	vector<Point> cpc;
	double eps = 10;
	for (int i = 0; i < c.size(); i++) {
		int pre = i - 1, next = (i + 1) % (c.size() - 1);
		if (pre == -1) {
			pre = c.size() - 1;
		}
		double a1, a2; 
		if ((c[pre].x - c[i].x) == 0) {
			a1 = M_PI / 2;
		}
		else {
			a1 = atan(((c[pre].y - c[i].y) / (c[pre].x - c[i].x)));
		}
		if ((c[next].x - c[i].x) == 0) {
			a2 = M_PI / 2;
		}
		else {
			a2 = atan(((c[next].y - c[i].y) / (c[next].x - c[i].x)));
		}
		double mcp;
		if (abs(a1 - a2) < M_PI) {
			mcp = abs(a1 - a2);
		}
		else {
			mcp = M_PI - abs(a1 - a2);
		}
		if (a1 < mcp < a2 or a1 > mcp > a2) {

			Point e, q;
			e.x = c[next].x - c[pre].x;
			e.y = c[next].y - c[pre].y;
			q.x = c[i].x - c[next].x;
			q.y = c[i].y - c[next].y;
			double r = e.x*q.y - e.y*q.x;
			if (abs(r) >= eps) {
				cpc.push_back(c[i]);
			}
		}
	}
	return cpc;
}