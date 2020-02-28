#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <tuple>

#include "state.hpp"
#include "path.hpp"
#include "frenet_point.hpp"
#include "waypoints.hpp"

class Planner
{
private:
    int m_lane;
    double m_speed;
    double m_dt;
    double m_path_length;
    double m_ds;
    double m_path_time;
    int m_num_path_points;
    Waypoints m_waypoints;

public:
    Planner(const int lane,
            const double speed,
            const double dt,
            const double path_length,
            const Waypoints& waypoints);
    ~Planner();
    Path planPath(const State& cur_state,
                  const Path& prev_path,
                  const FrenetPoint& prev_point,
                  const std::vector<std::vector<double>>& perception);
};

#endif