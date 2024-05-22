#include <node_c/obstacle_extractor/figures/circle.h>

Circle::Circle(const Point &point, double radius) : center(point), radius(radius)
{}

Circle::Circle(const Segment &segment)
{
    // Compute radius as in the paper : sqrt(3)/3 * length of the segment
    radius = 0.5773502 * segment.length();

    // Compute the center as the formula in the paper 0.5 * (p_1n + p_2n - r_n * n_n)
    center = (segment.getFirstPoint() + segment.getLastPoint() - radius * segment.normal()) / 2.0;
}

double Circle::distanceTo(const Point &point)
{
    return (point - center).length() - radius;
} // distanceTo

std::vector<PointSet> Circle::getPointSets() const
{
    return pointSets;
} // getPointSets

Circle Circle::fitCircle(const std::list<Point> &pointSet)
{
    // Get size of the pointset
    int N = pointSet.size();

    // Make sure the number of points of the pointset is >= 3
    assert(N >= 3);

    // Build input matrix (N,3), Output vector (2,1), Parameter vector (3,1)
    arma::mat input = arma::mat(N, 3).zeros(); // [x_i, y_i, 1]
    arma::vec output = arma::vec(N).zeros();   // [-(x_i^2 + y_i^2)]
    arma::vec params = arma::vec(3).zeros();   // [a_1 ; a_2 ; a_3]

    // Populate input matrix and output vector
    int i = 0;
    for (const Point &point : pointSet)
    {
        input(i, 0) = point.getPoint().x;
        input(i, 1) = point.getPoint().y;
        input(i, 2) = 1.0;
        output(i) = -(pow(point.getPoint().x, 2) + pow(point.getPoint().y, 2));
        i++;
    }

    // Find a_1, a_2 and a_3 coefficients from linear regression
    params = arma::pinv(input) * output;

    // Compute the center of the circle
    Point center(cv::Point2d(-params(0) / 2.0, -params(1) / 2.0));

    // Compute the radius of the cicle
    double radius = std::sqrt((params(0) * params(0) + params(1) * params(1)) / 4.0 - params(2));

    return Circle(center, radius);
} // fitCircle

void Circle::incrementRadius(double radius)
{
    this->radius += radius;
} // incrementRadius

double Circle::getRadius() const
{
    return radius;
} // getRadius

Point Circle::getCenter() const
{
    return center;
}