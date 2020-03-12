#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <tuple>

#include "path_generator.hpp"
#include "state_machine.hpp"
#include "path.hpp"
#include "frenet_point.hpp"
#include "waypoints.hpp"
#include "vehicle.hpp"

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
    int m_current_timestep;
    int m_lock_timestep;

    Path m_path;
    StateMachine m_state_machine;
    PathGenerator m_path_generator;

    void updateState(State new_state);
    std::vector<Vehicle> predictVehicles(const std::vector<Vehicle>& vehicles);
    Path generateTrajectory(const Vehicle& ego);

public:
    Planner(const int lane,
            const double speed,
            const double dt,
            const double path_length);
    ~Planner();
    Path planPath(const Vehicle& ego,
                  Path& prev_path,
                  const FrenetPoint& prev_point,
                  const std::vector<Vehicle>& vehicles);
};

#endif