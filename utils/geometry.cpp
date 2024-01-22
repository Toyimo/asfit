#include <cmath>
#include <limits>
#include <numeric>

#include "geometry.h"

using namespace asfit;

using Vector = Point;

inline double sign(const double& x){return x < 0 ? -1.0 : 1.0;}

Point Point::operator+(const Point& other) const { return Point(x + other.x, y + other.y); }
Point Point::operator-(const Point& other) const { return Point(x - other.x, y - other.y); }
Point Point::operator*(const double& factor) const { return Point(x * factor, y * factor); }
Point Point::operator/(const double& factor) const { return Point(x / factor, y / factor); }

double Point::dot(const Point& other) const { return x * other.x + y * other.y; }
double Point::cross(const Point& other) const { return x * other.y - y * other.x; }
double Point::norm() const { return std::sqrt(x * x + y * y); }

Point Point::normalize() const
{
    double length = this->norm();
    return Point(x / length, y / length);
}

double Point::convex(const Point& front, const Point& back) const
{
    Vector front_p = *this - front;
    Vector p_back = back - *this;
    double sign_tag = sign(front_p.cross(p_back));
    double cos_value = front_p.dot(p_back) / (front_p.norm() * p_back.norm());
    if(cos_value > 1.0) return 0.0f;
    if(cos_value < -1.0) return M_PI;
    return sign_tag * std::acos(cos_value);
}

double Point::get_length_to_pt(const Point& other) const
{
    return std::sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y));
}

double Point::get_length_to_line(const Point& p0, const Point& p1) const
{
    Vector a = *this - p0;
    Vector b = p1 - p0;
    double lb = b.norm();
    if(fabs(lb) == std::numeric_limits<double>::epsilon()){
        return get_length_to_pt(p0);
    } else return std::fabs(a.cross(b) / lb);
}

double Point::get_length_to_segment(const Point& p0, const Point& p1, double& ds) const
{
    // temp vector varible
    Vector a = *this - p0;
    Vector c = p1 - *this;
    Vector b = p1 - p0;
    double dot_ab = a.dot(b);
    double dot_cb = c.dot(b);
    double length = b.norm();
    // p0 and p1 is the same point
    if(fabs(length) == std::numeric_limits<double>::epsilon()){
        ds = 0.0;
        return get_length_to_pt(p0);
    }
    // check wether project p is on the segement
    if(dot_ab >= 0 && dot_cb >= 0){
        ds = dot_ab / length;
        return std::fabs(a.cross(b) / length);
    }
    else if(dot_ab < 0){
        ds = dot_ab / length;
        return get_length_to_pt(p0);
    }
    else{
        ds = length - dot_cb / length;
        return get_length_to_pt(p1);
    }
}

void Polyline::douglas_peuker(std::vector<Point>& out, const double& epsilon)
{
    return douglas_peuker(data, 0, data.size() - 1, epsilon, out);
}

void Polyline::douglas_peuker(
    const std::vector<Point>& points, 
    int start_index, int end_index, double epsilon, 
    std::vector<Point>& out)
{
if (end_index <= start_index) { return; }

    double dmax = 0;
    int index = 0;

    const Point& start = points[start_index];
    const Point& end = points[end_index];

    for (int i = start_index + 1; i < end_index; i++) {
        double d = points[i].get_length_to_line(start, end);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    if (dmax > epsilon) {
        std::vector<Point> recResults1;
        std::vector<Point> recResults2;
        douglas_peuker(points, start_index, index, epsilon, recResults1);
        douglas_peuker(points, index, end_index, epsilon, recResults2);

        out.insert(out.end(), recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
    } else {
        out.push_back(start);
        out.push_back(end);
    }
}