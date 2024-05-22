#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Twist.h>

#include <vector>

struct Point2D {
  double x;
  double y;
};

struct PolarCoord {
  double rho;
  double theta;
};

// Struct necessary to work with the DBSCAN algorithm
struct vec2f {
  double data[2];
  double operator[](int idx) const { return data[idx]; }
};

/**
 * @brief Converts polar coordinates to Cartesian coordinates.
 *
 * This function takes polar coordinates (theta, rho) and converts them to
 * Cartesian coordinates (x, y) using trigonometric functions. The result is
 * stored in the provided Point2D structure.
 *
 * @param theta The polar angle in radians.
 * @param rho The polar distance (radius).
 * @param position Reference to a Point2D structure to store the Cartesian
 *                 coordinates (x, y).
 */
void polarToCartesian(double theta, double rho, Point2D& position);

/**
 * @brief Find the polar radius (rho) corresponding to a target theta within a
 * specified threshold.
 *
 * This function searches for a polar radius within a vector of PolarCoord based
 * on the target theta and a specified threshold. It iterates through the
 * provided vector and returns the polar radius corresponding to the first
 * occurrence of a theta within the threshold range of the target theta.
 *
 * @param polarCoords A vector of PolarCoord containing polar coordinates (rho,
 * theta).
 * @param targetTheta The target theta for which to find the corresponding polar
 * radius.
 * @param threshold The allowable range (+/-) around the targetTheta within
 * which to consider a match.
 * @return The polar radius (rho) corresponding to the target theta if found;
 * otherwise, returns -1.0.
 *
 * @note If the targetTheta is not found within the specified threshold, an
 * error message is printed to the standard error stream (cerr), and the
 * function returns a sentinel value of -1.0.
 */
double findRhoByTheta(const std::vector<PolarCoord>& polarCoords,
                      double targetTheta, double threshold);

/**
 * @brief Compute the average polar radius within a specified angle range.
 *
 * This function calculates the average polar radius within the specified angle
 * range from a vector of PolarCoord. It iterates through the provided vector,
 * considering only those polar coordinates whose theta values fall within the
 * specified startAngle and endAngle.
 *
 * @param polarCoords A vector of PolarCoord containing polar coordinates (rho,
 * theta).
 * @param startAngle The starting angle of the range (in degrees).
 * @param endAngle The ending angle of the range (in degrees).
 * @return The average polar radius within the specified angle range.
 *         If no valid data points are found, returns 0.0.
 */
double computeAverageDistanceInRange(const std::vector<PolarCoord>& polarCoords,
                                     double startAngle, double endAngle);

/**
 * @brief Extracts valid Cartesian data from a laser scan.
 *
 * This function takes a vector of laser scan ranges and ignores invalid (inf)
 * range values.
 *
 * @param ranges_data Vector of laser scan range data.
 * @return Vector of valid polar coordinates (rho, theta).
 */
std::vector<PolarCoord> extractValidDataPolar(
    const std::vector<float>& ranges_data, const double angle_increment,
    const double angle_min_rad);

/**
 * @brief Extracts valid Cartesian data from a laser scan.
 *
 * This function takes a vector of laser scan ranges and converts them to
 * Cartesian coordinates (x, y). It ignores invalid (inf) range values.
 *
 * @param ranges_data Vector of laser scan range data.
 * @return Vector of valid Cartesian coordinates (x, y).
 */
std::vector<vec2f> extractValidDataCartesian(
    const std::vector<float>& ranges_data, const double angle_increment,
    const double angle_min_rad);

/**
 * @brief Compute the weighted average polar radius based on angle.
 *
 * This function calculates the weighted average polar radius from a vector of
 * PolarCoord. It iterates through the provided vector, assigning weights to
 * each data point based on the absolute value of its theta angle. The weight
 * function used is (|theta| + 1.0).
 *
 * @param polar_data A vector of PolarCoord containing polar coordinates (rho,
 * theta).
 * @return The weighted average polar radius based on angle.
 *         If no valid data points are found, returns 0.0.
 */
double computeWeightedAverageRange(const std::vector<PolarCoord>& polar_data);
#endif  // UTILS_H
