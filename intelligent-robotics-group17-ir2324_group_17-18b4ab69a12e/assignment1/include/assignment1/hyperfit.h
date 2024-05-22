#ifndef HYPERFIT_H
#define HYPERFIT_H

#include <cmath>
#include <vector>

struct Circle {
  double a;  // x-coordinate of center
  double b;  // y-coordinate of center
  double r;  // radius
};

Circle CircleFitByHyper(const std::vector<double>& X,
                        const std::vector<double>& Y, int size, double mean_x,
                        double mean_y);

#endif  // HYPERFIT_H
