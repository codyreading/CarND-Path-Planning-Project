#include <iostream>
#include "path_validator.hpp"
#include "utils.hpp"



PathValidationStatus validate_path(const Vehicle &ego,
                                   std::vector<Vehicle> others,
                                   const State &state,
                                   Path &path,
                                   int from_point)
{
    // Reject change lane trajectories when the speed is below 30 KM/H
    // cout << "*** BEGINING PATH VALIDATION STATUS =  for state (" << state.s_state << ", " << state.d_state << ") " << endl;
    // cout << "*** VELOCITY BEFORE LANE CHANGE " << trajectory.averageSpeed(trajectory.size()) << endl;
    // cout << "*** EGO SPEED " << ego.getSpeed() << endl;
    if (state.d_state != LateralState::STAY_IN_LANE)
    {
        if (path.averageSpeed(path.size()) < MIN_SPEED_FOR_LANE_CHANGE_METERS_PER_SECOND)
        {
            // cout << "VELOCITY TOO LOW FOR LANE CHANGE " << path.averageSpeed(path.size()) << endl;
            return PathValidationStatus::VELOCITY_TOO_LOW_FOR_LANE_CHANGE;
        }
    }

    if (!isLaneValid(state.current_lane) || !isLaneValid(state.future_lane))
    {
        cout << "** PATH current or future lane outside bounds: "
             << state.current_lane << " ---> " << state.current_lane
             << endl;
        return PathValidationStatus::OUTSIDE_OF_LANE;
    }


    double total_s_acc = 0.0;
    double total_d_acc = 0.0;
    double total_s_vel = 0.0;
    double total_d_vel = 0.0;
    double total_s_jerk = 0.0;
    double total_d_jerk = 0.0;
    double prev_s_vel = 0.0;
    double prev_d_vel = 0.0;

    for (int i = from_point + 1; i < path.size(); ++i)
    {
        double last_s = path.m_s[i - 1];
        double last_d = path.m_d[i - 1];

        double cur_s = path.m_s[i];
        double cur_d = path.m_d[i];

        double s_vel = cur_s - last_s;
        double d_vel = cur_d - last_d;

        // cout << "------------- CURRENT S VELOCITY = " << s_vel << endl;
        // cout << "------------- CURRENT D VELOCITY = " << d_vel << endl;

        total_s_vel += s_vel;
        total_d_vel += d_vel;

        double s_acc = i < from_point + 1 ? 0.0 : s_vel - prev_s_vel;
        double d_acc = i < from_point + 1 ? 0.0 : d_vel - prev_d_vel;

        total_s_acc += abs(s_acc);
        total_d_acc += abs(d_acc);

        prev_s_vel = s_vel;
        prev_d_vel = d_vel;

        // TODO Check turning angle should be less than a specified value
    }

    // cout << "------------- TOTAL S_ACC = " << total_s_acc << endl;
    // cout << "------------- TOTAL D_ACC = " << total_d_acc << endl;

    double total_acc = 0;
    double prev_velocity = 0.0;
    double total_velocity = 0.0;
    int segment_size = path.size() - from_point;

    // We are interested in the path for the last second
    for (int i = from_point + 1; i < 50; ++i)
    {
        double last_x = path.m_x[i - 1];
        double last_y = path.m_y[i - 1];

        double cur_x = path.m_x[i];
        double cur_y = path.m_y[i];

        double vel = distance(last_x, last_y, cur_x, cur_y);
        // cout << "**** DISTANCE  = " << vel << endl;
        // cout << "**** VELOCITY  = " << vel / CONTROLLER_UPDATE_RATE_SECONDS << endl;

        if (vel / CONTROLLER_UPDATE_RATE_SECONDS > MAX_LEGAL_SPEED_LIMIT_METERS_PER_SECOND)
        {
            cout << "**** MAXIMUM VELOCITY ABOVE THRESHOLD = " << vel / CONTROLLER_UPDATE_RATE_SECONDS << endl;
            return PathValidationStatus::VELOCITY_ABOVE_THRESHOLD;
        }

        total_velocity += vel;
        double acc = i == from_point ? 0.0 : vel - prev_velocity;
        total_acc += acc;

        prev_velocity = vel;
    }
    double avg_velocity = total_velocity / (segment_size * CONTROLLER_UPDATE_RATE_SECONDS);

    return PathValidationStatus::VALID;
}