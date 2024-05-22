/**
 * @author Marco Calì
 */

#include <actionlib/client/simple_action_client.h>
#include <assignment1/MovementAction.h>
#include <assignment1/MovementFeedback.h>
#include <assignment1/MovementGoal.h>
#include <assignment1/MovementResult.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

class NavigationClient {
 public:
  /**
   * @brief Constructor for the NavigationClient class.
   *
   * Initializes the NavigationClient with the specified goal pose (x, y,
   * theta).
   *
   * @param x The x-coordinate of the target pose.
   * @param y The y-coordinate of the target pose.
   * @param theta The orientation angle (in degrees) of the target pose.
   */
  NavigationClient(double x, double y, double theta)
      : goal_(createGoal(x, y, theta)), ac_("navigation", true) {}

  /**
   * @brief Execute the navigation logic.
   *
   * This function sends the pre-defined goal to the ActionServer, waits for the
   * result, and displays the obstacles if the goal is successfully achieved.
   */
  void executeNavigation() {
    ROS_INFO("Waiting for the navigation action server to start");
    ac_.waitForServer();
    ROS_INFO("Navigation server started, sending goal pose");

    // Send the goal to the ActionServer
    ac_.sendGoal(goal_, &doneCb, &activeCb, &feedbackCb);

    // Wait for the result
    ac_.waitForResult();

    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // Display the result
      const auto& result = *ac_.getResult();
      displayObstacles(result.obstacles);
    } else {
      ROS_ERROR("The robot failed to reach the goal.");
    }
  }

 private:
  assignment1::MovementGoal goal_;  // The goal pose for navigation.
  actionlib::SimpleActionClient<assignment1::MovementAction> ac_;

  /**
   * @brief Create a MovementGoal with the specified goal pose.
   *
   * @param x The x-coordinate of the target pose.
   * @param y The y-coordinate of the target pose.
   * @param theta The orientation angle (in degrees) of the target pose.
   * @return The created MovementGoal.
   */
  assignment1::MovementGoal createGoal(double x, double y, double theta) {
    assignment1::MovementGoal goal;
    setGoalPose(goal, x, y, theta);
    return goal;
  }

  /**
   * @brief Set the goal pose in the provided MovementGoal.
   *
   * @param goal The MovementGoal to set the pose.
   * @param x The x-coordinate of the target pose.
   * @param y The y-coordinate of the target pose.
   * @param theta The orientation angle (in degrees) of the target pose.
   */
  static void setGoalPose(assignment1::MovementGoal& goal, double x, double y,
                          double theta) {
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    tf::Quaternion quat;
    quat.setRPY(0, 0, theta * M_PI / 180.0);
    tf::quaternionTFToMsg(quat, goal.pose.orientation);
  }

  static void doneCb(const actionlib::SimpleClientGoalState& state,
                     const assignment1::MovementResultConstPtr& result) {
    // Callback executed when the goal is completed
    // Not used
  }

  static void activeCb() {
    // Callback executed when the goal becomes active
    // Not used
  }

  /**
   * @brief Callback executed when feedback is received from the action server.
   *
   * @param feedback The feedback received from the action server.
   */
  static void feedbackCb(
      const assignment1::MovementFeedbackConstPtr& feedback) {
    // Callback executed when feedback is received from the action server
    // Process and display feedback as needed
    ROS_INFO("Feedback: %s", feedback->current_status.c_str());
  }

  /**
   * @brief Display obstacles information received as feedback.
   *
   * @param obstacles The vector of geometry_msgs::Point representing obstacles.
   */
  static void displayObstacles(
      const std::vector<geometry_msgs::Point>& obstacles) {
    for (const auto& obstacle : obstacles) {
      ROS_INFO("Obstacle at (x, y) = (%f, %f)", obstacle.x, obstacle.y);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_client");
  ROS_INFO("Started the navigation client.");

  if (argc != 4) {
    ROS_ERROR("Usage: navigation_client <x> <y> <theta>");
    return 1;
  }

  double x = atof(argv[1]);
  double y = atof(argv[2]);
  double theta = atof(argv[3]);

  // Create a NavigationClient object
  NavigationClient navigationClient(x, y, theta);

  // Execute the navigation logic
  navigationClient.executeNavigation();

  return 0;
}
