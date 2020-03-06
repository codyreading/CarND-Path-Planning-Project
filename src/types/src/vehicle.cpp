#include <iostream>
#include <math.h>
#include "vehicle.hpp"
#include "utils.hpp"
#include "waypoints.hpp"

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double t)
{
    m_id = id;
    m_x = x;
    m_y = y;
    m_vx = vx;
    m_vy = vy;
    m_s = s;
    m_d = d;
    m_t = t;

    m_theta = getTheta(vx, vy);
    m_isInLane = isWithinLane(m_d, 4.0, 1.5);
    m_lane = calculateLane(m_d, 4.0, 1.5);
}

Vehicle Vehicle::predictNextPosition(double t1)
{
    double newX = m_x + m_vx * t1;
    double newY = m_y + m_vy * t1;
    double theta = atan2(newY - m_y, newX - m_x);
    vector<double> frenet = Waypoints::getInstance().toFrenet(newX, newY, theta);

    return Vehicle(m_id, newX, newY, m_vx, m_vy, frenet[0], frenet[1], t1);
}

Vehicle Vehicle::predictFuturePosition(double t) const
{
    double newX = m_x + m_vx * t;
    double newY = m_y + m_vy * t;
    std::vector<double> frenet = Waypoints::getInstance().toFrenet(newX, newY, m_theta);
    return Vehicle(m_id, newX, newY, m_vx, m_vy, frenet[0], frenet[1], t);
}

double Vehicle::getSpeed() const
{
    return sqrt(m_vx * m_vx + m_vy * m_vy);
}

std::vector<Vehicle> Vehicle::ahead(const std::vector<Vehicle> &others, int lane) const
{
    std::vector<Vehicle> v_ahead;
    for (const Vehicle &v : others)
    {
        if (v.m_lane != lane)
        {
            continue;
        }
        if (v.m_s >= m_s)
        {
            v_ahead.push_back(v);
        }
    }

    return v_ahead;
}

std::vector<Vehicle> Vehicle::behind(const std::vector<Vehicle> &others, int lane) const
{
    std::vector<Vehicle> v_behind;
    for (const Vehicle &v : others)
    {
        if (v.m_lane != lane)
        {
            continue;
        }
        if (v.m_s < m_s)
        {
            v_behind.push_back(v);
        }
    }

    return v_behind;
}

Vehicle::~Vehicle() {}

// Compute angle of travel using https://www.khanacademy.org/science/physics/two-dimensional-motion/two-dimensional-projectile-mot/a/what-are-velocity-components