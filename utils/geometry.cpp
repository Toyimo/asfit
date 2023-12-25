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
    return sign_tag * std::acos(front_p.dot(p_back) / (front_p.norm() * p_back.norm()));
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