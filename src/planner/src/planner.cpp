#include "planner.hpp"
#include "spline.hpp"
#include "spline_fit.hpp"

Planner::Planner(const int lane,
                 const double speed,
                 const double dt,
                 const double path_length,
                 const Waypoints& waypoints)
{
    m_lane = lane;
    m_speed = speed;
    m_dt = dt;
    m_path_length = path_length;
    m_ds = m_speed * m_dt; // Assuming constant velocity
    m_path_time = m_path_length / m_speed; // Assuming constant velocity
    m_num_path_points = (int)(m_path_time / m_dt);
    m_waypoints = waypoints;
}

Planner::~Planner()
{
}

Path Planner::planPath(const State& cur_state,
                       const Path& prev_path,
                       const FrenetPoint& prev_point,
                       const std::vector<std::vector<double>>& perception)

{
    /**
     * TODO: define a path made up of (x,y) points that the car will visit
     *   sequentially every .02 seconds
     */

    /* Fit spline to points */
    tk::spline spline_x, spline_y;
    std::tie(spline_x, spline_y) = fitSpline(cur_state.s, cur_state.d, m_waypoints.s, m_waypoints.x, m_waypoints.y, m_path_length);

    Path plan_path;
    /* Sample from spline */
    for (int i = 0; i < m_num_path_points; ++i)
    {
        double path_s = cur_state.s + i * m_ds;
        plan_path.x.push_back(spline_x(path_s));
        plan_path.y.push_back(spline_y(path_s));
    }
    return plan_path;
}