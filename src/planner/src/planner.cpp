#include "planner.hpp"
#include "utils.hpp"


#include "spline.hpp"
#include "spline_fit.hpp"

Planner::Planner(const int lane,
                 const double speed,
                 const double dt,
                 const double path_length)
{
    m_lane = lane;
    m_speed = speed;
    m_dt = dt;
    m_path_length = path_length;
    m_ds = m_speed * m_dt; // Assuming constant velocity
    m_path_time = m_path_length / m_speed; // Assuming constant velocity
    m_num_path_points = (int)(m_path_time / m_dt);
    m_current_timestep = 0;

    /* Initialize path */
    m_path = Path();

    /* Initialize state machine */
    int start_lane = 1; //calculateLane(this->trajectory.ds[0], DEFAULT_LANE_SPACING, DEFAULT_LANE_INSIDE_OFFSET);
    int finish_lane = start_lane;
    State initial_state = State(LongitudinalState::ACCELERATE, LateralState::STAY_IN_LANE, start_lane, finish_lane);
    m_state_machine = StateMachine(initial_state);

    /* Initialize path generator*/
    double time_interval = 1.7;
    m_path_generator = PathGenerator(time_interval);
}

Planner::~Planner()
{
}

Path Planner::planPath(const Vehicle& ego,
                       Path& prev_path,
                       const FrenetPoint& prev_point,
                       const std::vector<Vehicle>& vehicles)

{
    /**
     * TODO: define a path made up of (x,y) points that the car will visit
     *   sequentially every .02 seconds
     */

    /* Remove points consumed by simulator*/
    int points_consumed = m_path.size() - prev_path.size();
    m_current_timestep += points_consumed;
    if (prev_path.size() > 0)
    {
        m_path.removeFirstPoints(points_consumed);
    }

    /* Prediction */
    std::vector<Vehicle> predict_vehicles = predictVehicles(vehicles);

    /* Get next possible states */
    std::vector<State> next_states = m_state_machine.nextPossibleStates();

    /* Generate paths */
    int from_point = m_current_timestep == 0 ? 0 : 10;
    for (const State &state : next_states)
    {
        std::vector<Path> paths = m_path_generator.generatePaths(state, ego, m_path, from_point, 1);
    }

    // /* Behavior Planning */

    // /* Trajectory Generation */
    // Path plan_path = generateTrajectory(cur_state);

    /* Fit spline to points */
    // tk::spline spline_x, spline_y;
    // std::tie(spline_x, spline_y) = fitSpline(cur_state.s, cur_state.d, m_waypoints.s, m_waypoints.x, m_waypoints.y, m_path_length);

    Path plan_path;
    // plan_path.x = {};
    // plan_path.y = {};
    //  Sample from spline
    // for (int i = 0; i < m_num_path_points; ++i)
    // {
    //     double path_s = cur_state.s + i * m_ds;
    //     plan_path.x.push_back(spline_x(path_s));
    //     plan_path.y.push_back(spline_y(path_s));
    // }
    return plan_path;
}

std::vector<Vehicle> Planner::predictVehicles(const std::vector<Vehicle>& vehicles)
{
    std::vector<Vehicle> predict_vehicles;
    for (auto vehicle : vehicles)
    {
        predict_vehicles.push_back(vehicle.predictFuturePosition(m_dt));
    }
    return predict_vehicles;
}



Path Planner::generateTrajectory(const Vehicle& ego)
{
    Path path;
    return path;
}