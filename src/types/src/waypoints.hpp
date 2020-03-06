#ifndef Waypoints_HPP
#define Waypoints_HPP
#include "spline.hpp"
#include <iostream>
#include <vector>

class Waypoints
{
public:
    static Waypoints& getInstance();

    // C++ 11 technique for singleton
    Waypoints(Waypoints const&)         = delete;
    void operator=(Waypoints const&)    = delete;

    void addWaypoint(double x, double y, double s, double dx, double dy);
    void buildSplines();

    std::vector<double> toFrenet(double x, double y, double theta);

    std::vector<double> toRealWorldXY(double s, double d);

private:
    // Private constructor
    Waypoints() {}

    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

    tk::spline sp_x_s;
    tk::spline sp_y_s;
    tk::spline sp_dx_s;
    tk::spline sp_dy_s;

    int closestWaypoint(double x, double y);
    int nextWaypoint(double x, double y, double theta);
};

#endif