#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "utils.h"

class CorridorNavigator {
 public:
  CorridorNavigator() : nh_("~"), rate_(10) {
    // Create a publisher for the /mobile_base_controller/cmd_vel topic
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "/mobile_base_controller/cmd_vel", 10);

    laser_scan_sub_ =
        nh_.subscribe("/scan", 1, &CorridorNavigator::laserScanCallback, this);

    // Set the rate at which to publish the velocity commands
    rate_ = ros::Rate(10);  // 10 Hz
  }

  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_reading) {
    // Extract valid data from laser scan (non-infinite distances)

    valid_data_polar_ = extractValidDataPolar(scan_reading->ranges,
                                              scan_reading->angle_increment,
                                              scan_reading->angle_min);
  }

  double computeAngularVelocity() {
    // React to the distance between walls
    double threshold = 2;

    // Define the angle ranges for averaging
    double left_start_angle = 70.0;
    double left_end_angle = 95.0;
    double right_start_angle = -95.0;
    double right_end_angle = -70.0;

    // Compute the average distances over the specified ranges
    double left_distance = computeAverageDistanceInRange(
        valid_data_polar_, left_start_angle, left_end_angle);
    double right_distance = computeAverageDistanceInRange(
        valid_data_polar_, right_start_angle, right_end_angle);


    // Compute the difference in distances
    double distance_difference = right_distance - left_distance;
    double average_width = right_distance + left_distance;

    double relative_distance = std::abs(distance_difference) / average_width;
    ROS_INFO("Right distance %f, left distance %f -> %f percent", right_distance,
             left_distance, relative_distance);


    // Set a threshold for the difference
    if (relative_distance < 0.20) 
    	return 0.0;

    // Gain
    double K_omega = 0.35;

    // Compute angular velocity based on the distance difference
    return -K_omega * distance_difference;
  }

  void processLaserScan() {
    // Simple control law: Go forward with fixed linear velocity (vx) and react
    // to wall distance

    // Need to understand when we get out of the corridor and enter the main
    // room: in a corridor I expect to see a small average range, while in the
    // main room a larger average
    double average_range = computeWeightedAverageRange(valid_data_polar_);
    ROS_INFO("Average weighted range %f", average_range);

    double range_threshold = 2.33;
    if (average_range < range_threshold) {
      cmd_vel_.linear.x = 0.3;
      cmd_vel_.angular.z = computeAngularVelocity();
      if (cmd_vel_.angular.z != 0) cmd_vel_.linear.x = 0.15;
    } else {
      cmd_vel_.linear.x = 0;
      cmd_vel_.angular.z = 0;
    }
    
    if (average_range > range_threshold)
    	ros::shutdown();
    
    ROS_INFO("Inputs: u = %f, w = %f", cmd_vel_.linear.x, cmd_vel_.angular.z);
  }

  void run() {
    while (ros::ok()) {
      // Process laser scan data and compute control laws
      processLaserScan();

      // Publish the velocity command
      velocity_pub_.publish(cmd_vel_);

      valid_data_polar_.clear();
      valid_data_cartesian_.clear();
      // Sleep to maintain the specified publishing rate
      rate_.sleep();

      // Allow ROS to process callbacks
      ros::spinOnce();
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher velocity_pub_;
  geometry_msgs::Twist cmd_vel_;
  std::vector<Point2D> valid_data_cartesian_;
  std::vector<PolarCoord> valid_data_polar_;
  ros::Subscriber laser_scan_sub_;

  ros::Rate rate_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_corridor");

  CorridorNavigator navigator;
  navigator.run();
  return 0;
}
