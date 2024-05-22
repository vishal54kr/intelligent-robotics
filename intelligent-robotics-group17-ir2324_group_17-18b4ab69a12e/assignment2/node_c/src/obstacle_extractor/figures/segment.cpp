#include <node_c/obstacle_extractor/figures/segment.h>

Segment::Segment(const Point &point1, const Point &point2)
{
    // Swap if not counter-clockwise
    if (point1.cross(point2) > 0.0)
    {
        firstPoint = point1;
        lastPoint = point2;
    }
    else
    {
        firstPoint = point2;
        lastPoint = point1;
    }
}

void Segment::setFirstPoint(Point point)
{
    firstPoint = point;
} // setFirstPoint

void Segment::setLastPoint(Point point)
{
    lastPoint = point;
} // setLastPoint

double Segment::length() const
{
    return (lastPoint - firstPoint).length();
} // length

double Segment::lengthSquared() const
{
    return (lastPoint - firstPoint).lengthSquared();
} // lengthSquared

Point Segment::normal() const
{
    return (lastPoint - firstPoint).perpendicular().normalized();
} // normal

Point Segment::projection(const Point &point) const
{
    Point a = lastPoint - firstPoint;
    Point b = point - firstPoint;
    return firstPoint + a.dot(b) * a / a.lengthSquared();
} // projection

Point Segment::trueProjection(const Point &point) const
{
    Point a = lastPoint - firstPoint;
    Point b = point - firstPoint;
    Point c = point - lastPoint;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0) // Lays before the first
        return (firstPoint);
    else if (t > 1.0) // Lays after the last
        return (lastPoint);
    else
        return firstPoint + a.dot(b) * a / a.lengthSquared();
} // trueProjection

double Segment::distanceTo(const Point &point) const
{
    return (point - projection(point)).length();
} // distanceTo

double Segment::trueDistanceTo(const Point &p) const
{
    Point a = lastPoint - firstPoint;
    Point b = p - firstPoint;
    Point c = p - lastPoint;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0) // Lays before the first
        return b.length();
    else if (t > 1.0) // Lays after the last
        return c.length();

    Point projection = firstPoint + t * a;
    return (p - projection).length();
} // trueDistanceTo

Point Segment::getFirstPoint() const
{
    return firstPoint;
} // getFirstPoint

Point Segment::getLastPoint() const
{
    return lastPoint;
} // getLastPoint

Segment Segment::fitSegment(const PointSet &pointSet)
{
    // Compute Number of Points
    int N = pointSet.getNumberOfPoints();

    // Make sure N>=2
    assert(N >= 2);

    // Build X matrix (N,2),  Parameter vector [A ; B] (2,1), [-C] (N,1) Parameter vector
    arma::mat input = arma::mat(N, 2).zeros(); // [x_i, y_i]
    arma::vec output = arma::vec(N).ones();    // [-C]
    arma::vec params = arma::vec(2).zeros();   // [A ; B]

    // Populate X Matrix
    PointIterator point = pointSet.getBegin();
    for (int i = 0; i < N; ++i)
    {
        input(i, 0) = point->getPoint().x;
        input(i, 1) = point->getPoint().y;
        std::advance(point, 1);
    }

    // Find A and B coefficients from linear regression (assuming C = -1.0)
    params = arma::pinv(input) * output;

    double A = params(0);
    double B = params(1);
    double C = -1.0;

    // Find end points
    Point p1 = *pointSet.getBegin();
    Point p2 = *pointSet.getEnd();

    // Build Segments with end Points
    Segment segment(p1, p2);
    segment.pointSets.push_back(pointSet);

    // Compute norm squared of the parameter vector
    double D = (A * A + B * B);

    // Project end points on the line
    if (D > 0.0)
    {
        Point projectedP1, projectedP2;

        projectedP1.setPoint(cv::Point2d(
            (B * B * p1.getPoint().x - A * B * p1.getPoint().y - A * C) / D,
            (-A * B * p1.getPoint().x + A * A * p1.getPoint().y - B * C) / D));

        projectedP2.setPoint(cv::Point2d(
            (B * B * p2.getPoint().x - A * B * p2.getPoint().y - A * C) / D,
            (-A * B * p2.getPoint().x + A * A * p2.getPoint().y - B * C) / D));

        segment.setFirstPoint(projectedP1);
        segment.setLastPoint(projectedP2);
    }

    return segment;
} // fitSegment

Segment Segment::fitSegment(const std::vector<PointSet> &pointSets)
{
    // Compute Number of Points
    int N = 0;
    for (PointSet ps : pointSets)
        N += ps.getNumberOfPoints();

    // make sure N >=2
    assert(N >= 2);

    // Build X matrix (N,2),  Parameter vector [A ; B] (2,1), [-C] (N,1) Parameter vector
    arma::mat input = arma::mat(N, 2).zeros(); // [x_i, y_i]
    arma::vec output = arma::vec(N).ones();    // [-C]
    arma::vec params = arma::vec(2).zeros();   // [A ; B]

    // Populate X Matrix
    int n = 0;
    for (PointSet ps : pointSets)
    {
        PointIterator point = ps.getBegin();
        for (int i = 0; i < ps.getNumberOfPoints(); i++)
        {
            input(i + n, 0) = point->getPoint().x;
            input(i + n, 1) = point->getPoint().y;
            std::advance(point, 1);
        }
        n += ps.getNumberOfPoints();
    }

    // Find A and B coefficients from linear regression (assuming C = -1.0)
    params = arma::pinv(input) * output;

    double A = params(0);
    double B = params(1);
    double C = -1.0;

    // Find end points
    Point p1 = *pointSets.front().getBegin();
    Point p2 = *pointSets.back().getEnd();

    // Build Segments with end Points
    Segment segment(p1, p2);
    segment.pointSets = pointSets;

    // Compute norm squared of the parameter vector
    double D = (A * A + B * B);

    // Project end points on the line
    if (D > 0.0)
    {
        Point projectedP1, projectedP2;

        projectedP1.setPoint(cv::Point2d(
            (B * B * p1.getPoint().x - A * B * p1.getPoint().y - A * C) / D,
            (-A * B * p1.getPoint().x + A * A * p1.getPoint().y - B * C) / D));

        projectedP2.setPoint(cv::Point2d(
            (B * B * p2.getPoint().x - A * B * p2.getPoint().y - A * C) / D,
            (-A * B * p2.getPoint().x + A * A * p2.getPoint().y - B * C) / D));

        segment.setFirstPoint(projectedP1);
        segment.setLastPoint(projectedP2);
    }

    return segment;
} // fitSegment

std::vector<PointSet> Segment::getPointSets() const
{
    return pointSets;
}
