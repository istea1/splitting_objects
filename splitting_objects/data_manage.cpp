#include "data_manage.h"

void make_data(vector<Point> points) {
	ofstream file;
	file.open("file.txt");
	for (int i = 0; i < points.size(); i++) {
		file << points[i].x << " " << points[i].y << "\n";
	}
	file.close();
}

vector<Point> read_data() {
	ifstream file;
	file.open("file.txt");
	string s;
	vector<Point> v;
	while (getline(file, s)) {
		stringstream stream(s);
		int x, y;
		stream >> x >> y;
		v.push_back(Point(x, y));
	}
	return v;
}
