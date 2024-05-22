#include "hyperfit.h"
/*
    Circle fit to a given set of data points (in 2D)

    This is an algebraic fit based on the journal article

    A. Al-Sharadqah and N. Chernov, "Error analysis for circle fitting
   algorithms", Electronic Journal of Statistics, Vol. 3, pages 886-911, (2009)

    It is an algebraic circle fit with "hyperaccuracy" (with zero essential
   bias). The term "hyperaccuracy" first appeared in papers by Kenichi Kanatani
   around 2006

    Input:  data     - the class of data (contains the given points):
            size     - the number of data points
            data.X[] - the array of X-coordinates
            data.Y[] - the array of Y-coordinates

    Output: circle   - parameters of the fitting circle:
            circle.a - the X-coordinate of the center of the fitting circle
            circle.b - the Y-coordinate of the center of the fitting circle
            circle.r - the radius of the fitting circle
            circle.s - the root mean square error (the estimate of sigma)
            circle.j - the total number of iterations

    This method combines the Pratt and Taubin fits to eliminate the essential
   bias.

    It works well whether data points are sampled along an entire circle or
   along a small arc.
*/

Circle CircleFitByHyper(const std::vector<double>& X,
                        const std::vector<double>& Y, int size, double mean_x,
                        double mean_y) {
  int i, iter, IterMAX = 99;

  double Xi, Yi, Zi;
  double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
  double A0, A1, A2, A22;
  double Dy, xnew, x, ynew, y;

  Circle circle;

  // Compute x- and y- sample means (already provided as input)

  // Computing moments
  Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.;

  for (i = 0; i < size; i++) {
    Xi = X[i] - mean_x;  // centered x-coordinates
    Yi = Y[i] - mean_y;  // centered y-coordinates
    Zi = Xi * Xi + Yi * Yi;

    Mxy += Xi * Yi;
    Mxx += Xi * Xi;
    Myy += Yi * Yi;
    Mxz += Xi * Zi;
    Myz += Yi * Zi;
    Mzz += Zi * Zi;
  }

  Mxx /= size;
  Myy /= size;
  Mxy /= size;
  Mxz /= size;
  Myz /= size;
  Mzz /= size;

  // Computing the coefficients of the characteristic polynomial
  Mz = Mxx + Myy;
  Cov_xy = Mxx * Myy - Mxy * Mxy;
  Var_z = Mzz - Mz * Mz;

  A2 = 4 * Cov_xy - 3 * Mz * Mz - Mzz;
  A1 = Var_z * Mz + 4 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
  A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) -
       Var_z * Cov_xy;
  A22 = A2 + A2;

  // Finding the root of the characteristic polynomial using Newton's method
  // starting at x=0 (it is guaranteed to converge to the right root)
  for (x = 0., y = A0, iter = 0; iter < IterMAX; iter++) {
    Dy = A1 + x * (A22 + 16. * x * x);
    xnew = x - y / Dy;
    if ((xnew == x) || (!std::isfinite(xnew))) break;
    ynew = A0 + xnew * (A1 + xnew * (A2 + 4 * xnew * xnew));
    if (std::abs(ynew) >= std::abs(y)) break;
    x = xnew;
    y = ynew;
  }

  // Computing parameters of the fitting circle
  double DET = x * x - x * Mz + Cov_xy;
  double Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / DET / 2;
  double Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / DET / 2;

  // Assembling the output
  circle.a = Xcenter + mean_x;
  circle.b = Ycenter + mean_y;
  circle.r = std::sqrt(Xcenter * Xcenter + Ycenter * Ycenter + Mz - x - x);
  return circle;
}
