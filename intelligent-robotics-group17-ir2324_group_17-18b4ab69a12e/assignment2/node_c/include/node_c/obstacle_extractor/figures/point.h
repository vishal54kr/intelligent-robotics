#pragma once 

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


class Point
{
public:
    Point(cv::Point2d point = cv::Point2d(0, 0));
    Point(const Point &point);

    cv::Point2d getPoint() const;
    void setPoint(cv::Point2d point);
    double length() const;
    double lengthSquared() const;
    double angle() const;
    double angleDeg() const;
    double dot(const Point &point) const;
    double cross(const Point &point) const;
    Point normalized();
    Point reflected(const Point &normal) const;
    Point perpendicular();

    Point operator-();
    Point operator+();
    Point &operator=(const Point &point);
    Point &operator+=(const Point &point);
    Point &operator-=(const Point &point);

    friend Point operator+(const Point &point1, const Point &point2)
    {
        return Point(cv::Point2d(point1.point.x + point2.point.x, point1.point.y + point2.point.y));
    }

    friend Point operator-(const Point &point1, const Point &point2)
    {
        return Point(cv::Point2d(point1.point.x - point2.point.x, point1.point.y - point2.point.y));
    }

    friend Point operator*(const double factor, const Point &point)
    {
        return Point(cv::Point2d(factor * point.point.x, factor * point.point.y));
    }

    friend Point operator*(const Point &point, const double factor)
    {
        return Point(cv::Point2d(factor * point.point.x, factor * point.point.y));
    }

    friend Point operator/(const Point &point, const double factor)
    {
        return (factor != 0.0) ? Point(cv::Point2d(point.point.x / factor, point.point.y / factor)) : Point();
    }

    friend bool operator==(const Point &point1, const Point &point2)
    {
        return (point1.point.x == point2.point.x) && (point1.point.y == point2.point.y);
    }

    friend bool operator!=(const Point &point1, const Point &point2)
    {
        return !(point1 == point2);
    }

    friend bool operator<(const Point &point1, const Point &point2)
    {
        return point1.lengthSquared() < point2.lengthSquared();
    }

    friend bool operator<=(const Point &point1, const Point &point2)
    {
        return point1.lengthSquared() <= point2.lengthSquared();
    }

    friend bool operator>(const Point &point1, const Point &point2)
    {
        return point1.lengthSquared() > point2.lengthSquared();
    }

    friend bool operator>=(const Point &point1, const Point &point2)
    {
        return point1.lengthSquared() >= point2.lengthSquared();
    }

    friend bool operator!(const Point &point1)
    {
        return (point1.point.x == 0.0 && point1.point.y == 0.0);
    }

    friend std::ostream &operator<<(std::ostream &out, const Point &point)
    {
        out << point.point;
        return out;
    }

private:
    cv::Point2d point;
};
