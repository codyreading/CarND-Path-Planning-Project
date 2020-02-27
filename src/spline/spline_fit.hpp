#ifndef SPLINEFIT_HPP
#define SPLINEFIT_HPP

#include <tuple>
#include <vector>

#include "third-party/spline/src/spline.h"

/* Fit spline based on s */
std::tuple<tk::spline, tk::spline> fitSpline(
    const double car_s,
    const double car_d,
    const std::vector<double> &map_waypoints_s,
    const std::vector<double> &map_waypoints_x,
    const std::vector<double> &map_waypoints_y,
    const float spline_length);

#endif
