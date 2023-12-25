// @Description: Geometric Tools
// @Time       : 2023/12/20 10:52
// @Author     : tongjx

#pragma once

#include <string>
#include <vector>
#include <unordered_map>

namespace asfit
{
    /* 2D Point class.*/
    class Point
    {
    public:
        Point(){}
        Point(const double& X, const double& Y): x(X), y(Y){}

    public:
        Point operator+(const Point& other) const;
        Point operator-(const Point& other) const;
        Point operator*(const double& factor) const;
        Point operator/(const double& factor) const;

    public:
        // angle of front->P and P->back, + if anticlockwise, - otherwise
        virtual double dot(const Point& other) const;
        virtual double cross(const Point& other) const;
        virtual double norm() const;
        virtual Point normalize() const;
        virtual double convex(const Point& front, const Point& back) const;

    public:
        virtual double get_length_to_pt(const Point& other) const;
        virtual double get_length_to_line(const Point& p0, const Point& p1) const;
        // s is the distance to p0 on the line p0-p1
        virtual double get_length_to_segment(const Point& p0, const Point& p1, double& ds) const;

    public:
        double x = 0.0;
        double y = 0.0;
        std::unordered_map<std::string, double> attributes; // attach attributes for points
    };

    /* 3D Point class.*/
    class Point3D: public Point{
    public:
        Point3D(){}
        Point3D(const double& X, const double& Y, const double& Z): Point(X, Y), z(Z){}
    public:
        double z = 0.0;
    };

    /* Polyline with 2D Point. */
    class Polyline
    {
    public:
        std::vector<Point> data;
    };
}