#include "planner.hpp"
#include "utils.hpp"
#include "path_validator.hpp"
#include "cost_functions.hpp"

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
    /* std::vector<Vehicle> predict_vehicles = predictVehicles(vehicles); */

    /* Get next possible states */
    std::vector<State> next_states = m_state_machine.nextPossibleStates();

    /* Generate paths */
    int from_point = m_current_timestep == 0 ? 0 : 10;
    for (const State &state : next_states)
    {
        std::vector<Path> paths = m_path_generator.generatePaths(state, ego, m_path, from_point, 1);

        for (const Path &path : paths)
        {
            PathValidationStatus path_status = validate_path(ego, vehicles, state, path, 0);

            if (path_status != PathValidationStatus::VALID)
            {
                cout << "*** IGNORING STATE - PATH VALIDATION STATUS =  for state (" << state.s_state << ", " << state.d_state << ") = "
                     << path_status << endl;
                continue;
            }

            CostFunction lane_center_cost_fn = centerOfLaneDistCostFunction;
            double lane_center_cost = centerOfLaneDistCostFunction(ego, vehicles, path, state, 5.0);

            CostFunction cost_speed_fn = speedCostFunction;
            double cost_speed = cost_speed_fn(ego, vehicles, path, state, 1.0);
            // double cost_speed = 0.0;

            CostFunction avg_speed_lane_diff_fn = averageLaneSpeedDiffCostFunction;
            double avg_speed_lane_diff_cost = avg_speed_lane_diff_fn(ego, vehicles, path, state, 50.0);

            CostFunction dist_cars_cost_fn = distanceToClosestCarAheadCostFunction;
            double cost_dist_cars = dist_cars_cost_fn(ego, vehicles, path, state, 100.0);

            CostFunction change_lane_cost_fn = laneChangeCostFunction;
            double change_lane_cost = change_lane_cost_fn(ego, vehicles, path, state, 5.0);
            // double change_lane_cost = 0.0;

            CostFunction future_dist_to_goal_cost_fn = futureDistanceToGoalCostFunction;
            double future_dist_to_goal_cost = future_dist_to_goal_cost_fn(ego, vehicles, path, state, 1.0);

            CostFunction speed_diff_to_car_ahead_fn = speedDifferenceWithClosestCarAheadCostFunction;
            double speed_diff_to_car_ahead_cost = speed_diff_to_car_ahead_fn(ego, vehicles, path, state, 100.0);

            CostFunction collision_time_cost_fn = collisionTimeCostFunction;
            double collision_time_cost = collision_time_cost_fn(ego, vehicles, path, state, 10000.0);

            CostFunction dist_car_future_lane_cost_fn = distanceToClosestCarAheadFutureLaneCostFunction;
            double dist_car_future_lane_cost = dist_car_future_lane_cost_fn(ego, vehicles, path, state, 1000.0);
            // double dist_car_future_lane_cost = 0.0;

            CostFunction lon_dist_adjacent_car_cost_fn = longitudinalDistanceToClosestAdjacentCarFunction;
            // double lon_dist_adjacent_car_cost = lon_dist_adjacent_car_cost_fn(ego, vehicles, path, state, 1000.0);
            double lon_dist_adjacent_car_cost = 0.0;

            double final_cost = lane_center_cost
                                + cost_speed
                                + avg_speed_lane_diff_cost
                                + cost_dist_cars + change_lane_cost
                                + future_dist_to_goal_cost
                                + speed_diff_to_car_ahead_cost
                                + collision_time_cost + dist_car_future_lane_cost
                                + lon_dist_adjacent_car_cost;

            cout << left << "(" << state.s_state << "," << state.d_state << ")"
                 << ":" << state.current_lane << "->" << state.future_lane << endl;
            cout << left << setw(14) << setfill(' ') << "   Ego Lane: " << ego.lane;
            cout << left << "| " << setw(13) << setfill(' ') << lane_center_cost;
            cout << left << "| " << setw(13) << setfill(' ') << cost_speed;
            cout << left << "| " << setw(13) << setfill(' ') << avg_speed_lane_diff_cost;
            cout << left << "| " << setw(13) << setfill(' ') << cost_dist_cars;
            cout << left << "| " << setw(13) << setfill(' ') << change_lane_cost;
            cout << left << "| " << setw(13) << setfill(' ') << dist_car_future_lane_cost;
            cout << left << "| " << setw(13) << setfill(' ') << lon_dist_adjacent_car_cost;
            cout << left << "| " << setw(13) << setfill(' ') << future_dist_to_goal_cost;
            cout << left << "| " << setw(13) << setfill(' ') << speed_diff_to_car_ahead_cost;
            cout << left << "| " << setw(13) << setfill(' ') << collision_time_cost;
            cout << left << "| " << setw(13) << setfill(' ') << final_cost;
            cout << endl;

            if (final_cost < lowest_cost)
            {
                lowest_cost = final_cost;
                chosen_trajectory = path;
                chosen_state = state;
            }
        }
    }
    cout << "*  - Chosen state: (" << chosen_state.s_state
         << "," << chosen_state.d_state << ")"
         << " lane transition: " << chosen_state.current_lane << " -> " << chosen_state.future_lane
         << endl;

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