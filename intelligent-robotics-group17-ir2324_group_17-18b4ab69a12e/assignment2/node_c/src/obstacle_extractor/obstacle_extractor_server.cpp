#include <node_c/obstacle_extractor/obstacle_extractor_server.h>

ObstacleExtractorServer::ObstacleExtractorServer(ros::NodeHandle node)
{
    this->node = node;
    this->server = this->node.advertiseService(
        "/obstacle_extractor", 
        &ObstacleExtractorServer::sendResponse, 
        this
    );
}

bool ObstacleExtractorServer::sendResponse(node_c_msgs::Circles::Request &req, node_c_msgs::Circles::Response &res)
{
    ObstacleExtractor ex(this->computePoints(req.scan_data),req.scan_data.angle_increment,req.max_circle_radius, req.min_circle_radius, req.radius_enlargment);
    std::list<Segment> segments;
    std::list<Circle> circles;
    ex.processPoints(segments, circles);

    for(Circle circle : circles)
    {
        node_c_msgs::Circle c;
        c.x = circle.getCenter().getPoint().x;
        c.y = circle.getCenter().getPoint().y;
        c.radius = circle.getRadius();
        res.cirles.push_back(c);
    }

    return true;
}

std::list<Point> ObstacleExtractorServer::computePoints(sensor_msgs::LaserScan msg)
{
    float angleMin = msg.angle_min + M_PI_2;
    float angleIncrement = msg.angle_increment;
    std::list<Point> points;

    for (int i = 0; i < msg.ranges.size(); i++)
    {
        float x = msg.ranges.at(i) * std::cos(angleMin);
        float y = msg.ranges.at(i) * std::sin(angleMin);

        points.push_back(Point(cv::Point2d(x, y)));
        angleMin += angleIncrement;
    }
    return points;
}