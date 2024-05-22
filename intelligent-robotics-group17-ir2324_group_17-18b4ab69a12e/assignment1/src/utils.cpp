#include "utils.h"

#include <cmath>
#include <iostream>

void polarToCartesian(double theta, double rho, Point2D& position) {
  position.x = rho * std::cos(theta);
  position.y = rho * std::sin(theta);
}

double findRhoByTheta(const std::vector<PolarCoord>& polarCoords,
                      double targetTheta, double threshold) {
  for (const auto& polarCoord : polarCoords) {
    // Check if the theta is within the threshold range
    if (std::abs(polarCoord.theta - targetTheta) <= threshold) {
      return polarCoord.rho;  // Return the corresponding rho
    }
  }
  // If the targetTheta is not found, you might want to handle this case
  std::cerr << "Target theta not found in the vector within the threshold."
            << std::endl;
  return -1.0;  // or any other sentinel value
}

double computeAverageDistanceInRange(const std::vector<PolarCoord>& polarCoords,
                                     double startAngle, double endAngle) {
  double sum = 0.0;
  int count = 0;

  // Iterate over the polar coordinates and sum up distances within the
  // specified angle range
  for (const auto& polarCoord : polarCoords) {
    if (polarCoord.theta >= startAngle && polarCoord.theta <= endAngle) {
      sum += polarCoord.rho;
      count++;
    }
  }

  // Compute the average distance
  return (count > 0) ? (sum / count) : 0.0;
}

std::vector<PolarCoord> extractValidDataPolar(
    const std::vector<float>& ranges_data, const double angle_increment,
    const double angle_min_rad) {
  std::vector<PolarCoord> valid_data;
  for (std::size_t i = 0; i < ranges_data.size(); ++i) {
    // Ignore invalid (inf) range values
    if (!std::isinf(ranges_data[i])) {
      // Calculate the corresponding angle in radians
      double angle_rad = i * angle_increment + angle_min_rad;
      double angle_deg = angle_rad * 180.0 / M_PI;
      PolarCoord polar = {ranges_data[i], angle_deg};
      valid_data.push_back(polar);
    }
  }
  return valid_data;
}

std::vector<vec2f> extractValidDataCartesian(
    const std::vector<float>& ranges_data, const double angle_increment,
    const double angle_min_rad) {
  std::vector<vec2f> valid_data;

  for (std::size_t i = 0; i < ranges_data.size(); ++i) {
    // Ignore invalid (inf) range values
    if (!std::isinf(ranges_data[i])) {
      // Calculate the corresponding angle in radians
      double angle_rad = i * angle_increment + angle_min_rad;

      // Convert polar coordinates to Cartesian coordinates
      Point2D point;
      polarToCartesian(angle_rad, ranges_data[i], point);
      vec2f point_2f = {point.x, point.y};
      valid_data.push_back(point_2f);
    }
  }

  return valid_data;
}

double computeWeightedAverageRange(const std::vector<PolarCoord>& polar_data) {
  double weighted_sum = 0.0;
  double total_weight = 0.0;

  for (const auto& pair : polar_data) {
    // Compute weight based on the angle (theta)
    double weight =
        std::abs(pair.theta) + 1.0;  // Adjust the weight function as needed

    // Accumulate the weighted sum and total weight
    weighted_sum += pair.rho * weight;
    total_weight += weight;
  }

  return weighted_sum / total_weight;
}