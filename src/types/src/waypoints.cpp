#include <iostream>
#include <vector>
#include <math.h>
#include "waypoints.hpp"
#include "utils.hpp"
#include "spline.hpp"


Waypoints &Waypoints::getInstance()
{
    // Guaranteed to be instantiated on first use
    static Waypoints instance;
    return instance;
}

void Waypoints::addWaypoint(double x, double y, double s, double dx, double dy)
{
    this->waypoints_x.push_back(x);
    this->waypoints_y.push_back(y);
    this->waypoints_s.push_back(s);
    this->waypoints_dx.push_back(dx);
    this->waypoints_dy.push_back(dy);
}

void Waypoints::buildSplines()
{
    this->sp_x_s.set_points(this->waypoints_s, this->waypoints_x);
    this->sp_y_s.set_points(this->waypoints_s, this->waypoints_y);
    this->sp_dx_s.set_points(this->waypoints_s, this->waypoints_dx);
    this->sp_dy_s.set_points(this->waypoints_s, this->waypoints_dy);
}

int Waypoints::closestWaypoint(double x, double y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < this->waypoints_x.size(); i++)
    {
        double Waypoints_x = this->waypoints_x[i];
        double Waypoints_y = this->waypoints_y[i];
        double dist = distance(x, y, Waypoints_x, Waypoints_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int Waypoints::nextWaypoint(double x, double y, double theta)
{
    int closestWaypoint = this->closestWaypoint(x, y);

    double Waypoints_x = this->waypoints_x[closestWaypoint];
    double Waypoints_y = this->waypoints_y[closestWaypoint];

    double heading = atan2((Waypoints_y - y), (Waypoints_x - x));

    double angle = fabs(theta - heading);
    angle = min(2 * pi() - angle, angle);

    if (angle > pi() / 4)
    {
        closestWaypoint++;
        if (closestWaypoint == this->waypoints_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

std::vector<double> Waypoints::toFrenet(double x, double y, double theta)
{
    int next_wp = this->nextWaypoint(x, y, theta);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = this->waypoints_x.size() - 1;
    }

    double n_x = this->waypoints_x[next_wp] - this->waypoints_x[prev_wp];
    double n_y = this->waypoints_y[next_wp] - this->waypoints_y[prev_wp];
    double x_x = x - this->waypoints_x[prev_wp];
    double x_y = y - this->waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - this->waypoints_x[prev_wp];
    double center_y = 2000 - this->waypoints_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(this->waypoints_x[i], this->waypoints_y[i], this->waypoints_x[i + 1], this->waypoints_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

std::vector<double> Waypoints::toRealWorldXY(double s, double d)
{
    s = fmod(s, MAX_TRACK_S);
    // Use the spline we have created to get a smoother path
    double x = sp_x_s(s) + d * sp_dx_s(s);
    double y = sp_y_s(s) + d * sp_dy_s(s);

    return {x, y};
}