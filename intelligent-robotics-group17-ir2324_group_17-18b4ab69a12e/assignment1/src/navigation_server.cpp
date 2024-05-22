/**
 * @author: Marco Calì
 */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment1/MovementAction.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

#include <chrono>
#include <thread>

#include "dbscan.h"
#include "hyperfit.h"
#include "utils.h"

class NavigationServer {
 public:
  /**
   * @brief Constructor for the NavigationServer class.
   *
   * This constructor initializes the NavigationServer object.
   * It sets up the action server, subscribes to the move_base feedback topic,
   * and prepares for communication with the move_base action server.
   *
   * @param name The name of the action server.
   */
  NavigationServer(std::string name)
      : as_(nh_, name,
            boost::bind(&NavigationServer::executeCallback, this, _1), false),
        action_name_(name),
        ac_("move_base", true) {
    // Start the action server (communicates with our client)
    as_.start();

    // Subscribe to move_base feedback topic
    move_base_feedback_sub_ = nh_.subscribe(
        "/move_base/feedback", 10, &NavigationServer::feedbackCallback, this);
  }

  /**
   * @brief Callback function to execute when a goal is received from the
   * client.
   *
   * This function is called when a goal is received from the client.
   * It sends the goal to the move_base action server, waits for the result,
   * and performs obstacle detection using laser scan data after reaching the
   * final pose sent from the client. The execution status is communicated back
   * to the client via action feedback.
   *
   * @param goal The goal pose received from the client.
   */
  void executeCallback(const assignment1::MovementGoalConstPtr &goal) {
    ROS_INFO("Received goal from client. Executing callback...");

    // Create a MoveBaseGoal to send to move_base
    move_base_msgs::MoveBaseGoal move_base_goal = goalToMoveBase(goal);

    // Send the goal to move_base and wait for result
    ac_.sendGoal(move_base_goal);
    ac_.waitForResult();

    // Check if move_base goal was successful
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // Notify the action server that the goal is succeeded
      feedback_.current_status = "The robot stopped at the goal pose";
      as_.publishFeedback(feedback_);

      // Notify the action server that obstacle detection is starting
      feedback_.current_status =
          "The robot started the detection of the obstacles";
      as_.publishFeedback(feedback_);

      // Subscribe to the laser scan topic for obstacle detection
      ROS_INFO("Subscribing to the laser scan topic...");
      laser_scan_sub_ = nh_.subscribe(
          "/scan", 1, &NavigationServer::scanFeedbackCallback, this);

      // Perform obstacle detection (wait for a short period)
      ROS_INFO("Detecting obstacles...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      // Notify the action server about obstacle detection results
      feedback_.current_status =
          "The robot completed the detection of the obstacles, found " +
          std::to_string(result_.obstacles.size()) + " obstacles";
      as_.publishFeedback(feedback_);

      // We only need one reading!
      as_.setSucceeded(result_);
    } else {
      // Notify the action server that the goal is aborted
      ROS_ERROR("The robot failed to reach the goal.");
      as_.setAborted(result_);
    }
  }

 private:
  ros::NodeHandle nh_;
  std::string action_name_;
  assignment1::MovementFeedback feedback_;
  assignment1::MovementResult result_;
  actionlib::SimpleActionServer<assignment1::MovementAction> as_;
  ros::Subscriber move_base_feedback_sub_;
  ros::Subscriber laser_scan_sub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

  /**
   * @brief Callback function for move_base action feedback.
   *
   * This function processes the feedback from the move_base action server.
   * It updates the current_status field in the feedback_ member variable based
   * on the status received from move_base. The updated feedback is then
   * published.
   *
   * @param feedback A pointer to the move_base action feedback message.
   */
  void feedbackCallback(
      const move_base_msgs::MoveBaseActionFeedbackConstPtr &feedback) {
    if (feedback->status.status == 1) {
      feedback_.current_status = "The robot is moving";
    } else if (feedback->status.status == 3) {
      feedback_.current_status = "The robot reached the goal pose";
    } else {
      feedback_.current_status = "Unknown status";
    }
    as_.publishFeedback(feedback_);
  }

  /**
   * @brief Callback function for processing laser scan feedback.
   *
   * This function processes the provided laser scan data, extracts valid data
   * (non-infinite distances), applies the DBSCAN algorithm to find clusters,
   * and extracts obstacles from the clusters using circle fitting. The
   * resulting obstacles are stored in the result_ member variable.
   * Finally, it shuts down the laser scan subscriber after processing as the
   * robot is in steady state when reaching destination pose, thus the
   * measurements are constant.
   *
   * @param scan_reading A pointer to the laser scan message.
   */
  void scanFeedbackCallback(
      const sensor_msgs::LaserScanConstPtr &scan_reading) {
    // Extract valid data from laser scan (non-infinite distances)
    std::vector<vec2f> valid_data = extractValidDataCartesian(
        scan_reading->ranges, scan_reading->angle_increment,
        scan_reading->angle_min);

    // Run the DBSCAN algorithm to find clusters
    auto dbscan = DBSCAN<vec2f, double>();
    /*
    The last two parameters are:
    The maximum distance between two samples for one to be considered as in
    the neighborhood of the other. This is the most important DBSCAN parameter
    to choose appropriately for your data set and distance function.

    The number of samples in a neighborhood for a point to be considered as a
    core point. If it is set to a higher value, DBSCAN will find denser
    clusters, whereas if it is set to a lower value, the found clusters will be
    more sparse.
    **/
    dbscan.Run(&valid_data, 2, 0.4f, 3);
    std::vector<std::vector<uint>> clusters = dbscan.Clusters;

    // Extract obstacles from clusters
    result_.obstacles = extractObstacles(valid_data, clusters);

    // Shutdown the laser scan subscriber (only one measurement is
    // sufficient)
    laser_scan_sub_.shutdown();
  }

  /**
   * @brief Extracts obstacles from clustered valid data using circle fitting.
   *
   * This function takes a vector of valid Cartesian data points and a vector
   * of clusters obtained from the DB clustering algorithm. It applies circle
   * fitting to each cluster and extracts obstacles based on the fitted
   * circle's radius and tolerance, which are known from heuristics.
   *
   * @param valid_data Vector of valid Cartesian coordinates (x, y).
   * @param clusters Vector of clusters obtained from DBSCAN algorithm.
   * @return Vector of geometry_msgs::Point representing the detected
   * obstacles.
   */
  std::vector<geometry_msgs::Point> extractObstacles(
      const std::vector<vec2f> &valid_data,
      const std::vector<std::vector<uint>> &clusters) {
    std::vector<geometry_msgs::Point> obstacles;

    for (auto &cluster : clusters) {
      std::vector<double> X, Y;
      double mean_x = 0.0;
      double mean_y = 0.0;
      double size = 1;

      // Extract cluster points and compute mean
      for (auto &point : cluster) {
        double x = valid_data[point][0];
        double y = valid_data[point][1];

        // Add the x, y coordinates to the cluster
        X.push_back(x);
        Y.push_back(y);

        // Compute the mean of each cluster
        mean_x += (x - mean_x) / size;
        mean_y += (y - mean_y) / size;

        // Increment the size of the cluster
        size++;
      }

      // For each cluster, apply the circle fitting algorithm
      Circle c = CircleFitByHyper(X, Y, cluster.size(), mean_x, mean_y);

      double obstacles_size = 0.18;
      double tolerance = 0.05;

      // Filter circles by size; the obstacles have a radius of about 0.18
      if (std::abs(c.r - obstacles_size) < tolerance) {
        ROS_INFO("Circle has radius %f and center in (%f, %f)", c.r, c.a, c.b);
        geometry_msgs::Point point;
        point.x = c.a;
        point.y = c.b;
        point.z = 0;

        // Add obstacle to the result
        obstacles.push_back(point);
      }
    }

    return obstacles;
  }

  /**
   * @brief Converts a custom MovementGoal to a move_base_msgs::MoveBaseGoal.
   *
   * This function takes a custom MovementGoal message and converts it into a
   * move_base_msgs::MoveBaseGoal, which is compatible with the MoveBase
   * action. It sets the target_pose header frame_id to "map", timestamp to
   * the current ROS time, and copies the pose from the provided MovementGoal.
   *
   * @param goal The custom MovementGoal to be converted.
   * @return A move_base_msgs::MoveBaseGoal representing the converted goal.
   */
  move_base_msgs::MoveBaseGoal goalToMoveBase(
      const assignment1::MovementGoalConstPtr &goal) {
    move_base_msgs::MoveBaseGoal move_base_goal;

    // Set the target_pose header frame_id to "map"
    move_base_goal.target_pose.header.frame_id = "map";

    // Set the timestamp to the current ROS time
    move_base_goal.target_pose.header.stamp = ros::Time::now();

    // Copy the pose from the provided MovementGoal
    move_base_goal.target_pose.pose = goal->pose;

    return move_base_goal;
  }
};

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "navigation_server");

  // Create NavigationServer instance with action name "navigation"
  NavigationServer server("navigation");

  ROS_INFO("Launched Navigation Server");
  ros::spin();

  return 0;
}
