#include "collision_detector.hpp"
#include <iostream>
#include <vector>
#include "utils.hpp"
#include "waypoints.hpp"

Collision::Collision(const Vehicle &v, const bool willCollide,
                     const double collision_point_x, const double collision_point_y,
                     const double collision_timestep)
    : v(v), willCollide(willCollide),
      collision_point_x(collision_point_x), collision_point_y(collision_point_y),
      collision_timestep(collision_timestep)
{
}


Collision predictCollision(Path &path, const Vehicle &vehicle, double timestep)
{
    Waypoints &waypoints = Waypoints::getInstance();

    // TODO we should store vehicle predicted coordinates somewhere so that it is reused
    for (int i = 0; i < path.size(); ++i)
    {
        double ref_x = path.m_x[i];
        double ref_y = path.m_y[i];

        double v_predcited_x = vehicle.m_x + vehicle.m_vx * timestep * i;
        double v_predcited_y = vehicle.m_y + vehicle.m_vy * timestep * i;

        std::vector<double> frenet = waypoints.toFrenet(v_predcited_x, v_predcited_y, vehicle.m_theta);

        // TODO check lane here
        int ego_lane = calculateLane(path.m_d[i], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
        int v_lane = calculateLane(frenet[1], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);

        // if (ego_lane < 0 || ego_lane >= LANES_COUNT)
        // {
        //     continue;
        // }

        // if (v_lane < 0 || v_lane >= LANES_COUNT)
        // {
        //     continue;
        // }

        double dist = distance(ref_x, ref_y, v_predcited_x, v_predcited_y);

        // TODO Put this into a constant
        if (dist < 15)
        {
            std::cout << ">>>>>>>>>>>>** Collision timestep = " << i << endl;
            return Collision(vehicle, true, ref_x, ref_y, (double)i);
        }
    }
    return Collision(vehicle, false, 0.0, 0.0, 0.0);
}