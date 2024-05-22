#include <node_c/obstacle_extractor/figures/point.h>

Point::Point(cv::Point2d point)
{
    this->point = point;
}

double Point::length() const
{
    return std::sqrt(lengthSquared());
} // length

double Point::lengthSquared() const
{
    return std::pow(point.x, 2) + std::pow(point.y, 2);
} // lengthSquared

double Point::angle() const
{
    return atan2(point.y, point.x);
} // angle

double Point::angleDeg() const
{
    return 180 * angle() / M_PI;
} // angleDeg

double Point::dot(const Point &point) const
{
    return this->point.dot(point.point);
} // dot

double Point::cross(const Point &point) const
{
    return this->point.cross(point.point);
} // cross

Point Point::normalized()
{
    return (length() > 0.0) ? *this / length() : *this;
} // normalized

Point Point::reflected(const Point &normal) const
{
    return *this - 2.0 * normal * (normal.dot(*this));
} // reflected

Point Point::perpendicular()
{
    return Point(cv::Point2d(-point.y, point.x));
} // perpendicular

Point Point::operator-()
{
    return Point(cv::Point2d(-point.x, -point.y));
} // operator-

Point Point::operator+()
{
    return Point(cv::Point2d(point.x, point.y));
} // operator+

Point &Point::operator=(const Point &point)
{
    if (this != &point)
    {
        this->point.x = point.point.x;
        this->point.y = point.point.y;
    }
    return *this;
} // operator=

Point &Point::operator+=(const Point &point)
{
    this->point.x += point.point.x;
    this->point.y += point.point.y;
    return *this;
} // operator+=

Point &Point::operator-=(const Point &point)
{
    this->point.x -= point.point.x;
    this->point.y -= point.point.y;
    return *this;
} // operator-=

cv::Point2d Point::getPoint() const
{
    return point;
} // getPoint

void Point::setPoint(cv::Point2d point)
{
    this->point = point;
} // setPoint

Point::Point(const Point &point) : point(point.getPoint())
{}