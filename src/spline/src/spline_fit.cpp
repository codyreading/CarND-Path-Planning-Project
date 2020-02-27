#include "spline_fit.hpp"
#include "utils.hpp"



/* Fit spline based on s */
std::tuple<tk::spline, tk::spline> fitSpline(
    const double car_s,
    const double car_d,
    const std::vector<double>& map_waypoints_s,
    const std::vector<double>& map_waypoints_x,
    const std::vector<double>& map_waypoints_y,
    const float spline_length)
{
    /* Set Spline parameters */
    int num_spline_pts = 50;
    float interval = spline_length / num_spline_pts;
    std::vector<double> spline_s, spline_x_pts, spline_y_pts;

    /* Add points to fit spline to */
    for (int i = 0; i < num_spline_pts; ++i)
    {
        double s = car_s + i * interval;
        double d = car_d;
        std::vector<double> point = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        spline_s.push_back(s);
        spline_x_pts.push_back(point[0]);
        spline_y_pts.push_back(point[1]);
    }

    tk::spline spline_x;
    tk::spline spline_y;
    spline_x.set_points(spline_s, spline_x_pts);
    spline_y.set_points(spline_s, spline_y_pts);
    return std::make_tuple(spline_x, spline_y);
}