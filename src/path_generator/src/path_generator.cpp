#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/QR"
#include "path_generator.hpp"
#include "vehicle.hpp"
#include "path.hpp"
#include "waypoints.hpp"
#include "state_machine.hpp"
#include "utils.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

PathGenerator::PathGenerator()
{
}

PathGenerator::PathGenerator(double time_interval)
{
    m_time_interval = time_interval;
}

std::vector<Path> PathGenerator::generatePaths(
    const State &state,
    const Vehicle &ego,
    Path &current_path,
    int from_point_index,
    int path_count)
{
    double target_s = 0.0;
    double target_s_vel = 0.0;
    double target_s_acc = 0.0;

    double target_d = 0.0;
    double target_d_vel = 0.0;
    double target_d_acc = 0.0;

    double std_s = 2.0;
    double std_d = 0.0;

    double speed_at_index = current_path.m_s_vel[from_point_index];
    double acc_at_index = current_path.m_s_acc[from_point_index];

    switch (state.s_state)
    {
    case LongitudinalState::MAINTAIN_COURSE:
        target_s_vel = speed_at_index;
        break;
    case LongitudinalState::ACCELERATE:
        if (speed_at_index == 0)
        {
            target_s_vel = 8.0;
        }
        else
        {
            double mult = 1.05;

            // Allow high acceleration at lower speeds
            if (speed_at_index < 10)
            {
                mult = 1.1;
            }
            target_s_vel = speed_at_index * mult;
        }
        break;
    case LongitudinalState::DECELERATE:
        target_s_vel = speed_at_index * 0.85;
        break;
    case LongitudinalState::STOP:
        target_s_vel = speed_at_index * 0.6;
    }

    if (target_s_vel > 21)
    {
        // 22m/s ~ 50 MPH
        target_s_vel = 21;
    }

    double s_offset = 0.0;
    double d_at_index = current_path.m_d[from_point_index];
    switch (state.d_state)
    {
    case LateralState::STAY_IN_LANE:
        target_d = getLaneCenterFrenet(state.current_lane);
        break;
    case LateralState::PREPARE_CHANGE_LANE_LEFT:
        target_d = getLaneCenterFrenet(state.current_lane);
        break;
    case LateralState::PREPARE_CHANGE_LANE_RIGHT:
        target_d = getLaneCenterFrenet(state.current_lane);
        break;
    case LateralState::CHANGE_LANE_LEFT:
        target_d = getLaneCenterFrenet(state.future_lane);
        std_d = 1.0;
        break;
    case LateralState::CHANGE_LANE_RIGHT:
        target_d = getLaneCenterFrenet(state.future_lane);
        std_d = 1.0;
        break;
    }

    if (current_path.size() < 5)
    {
        target_d = current_path.m_d[0];
    }

    // cout << "******** START S = " << current_path.ss[from_point_index] << endl;
    target_s = (current_path.m_s[from_point_index] + target_s_vel * m_time_interval);

    return this->generatePaths(current_path, target_s, target_d, target_s_vel,
                               0.0, 0.0, 0.0, std_s, std_d,
                               path_count, from_point_index + 1);
}

std::vector<Path> PathGenerator::generatePaths(
    Path &current_path,
    double target_s,
    double target_d,
    double target_s_speed,
    double target_d_speed,
    double target_s_acc,
    double target_d_acc,
    double std_s,
    double std_d,
    int count,
    int from_point_index)
{
    std::vector<Path> paths;

    Path first_path = current_path.clone(from_point_index);
    int path_size = first_path.size();
    double last_x = 0.0;
    double last_y = 0.0;
    double last_s = 0.0;
    double last_d = 0.0;
    double last_s_vel = 0.0;
    double last_d_vel = 0.0;
    double last_s_acc = 0.0;
    double last_d_acc = 0.0;

    if (path_size > 0)
    {
        last_x = first_path.m_x[path_size - 1];
        last_y = first_path.m_y[path_size - 1];
        last_s = first_path.m_s[path_size - 1];
        last_d = first_path.m_d[path_size - 1];
        last_s_vel = first_path.m_s_vel[path_size - 1];
        last_d_vel = first_path.m_d_vel[path_size - 1];
        last_s_acc = first_path.m_s_acc[path_size - 1];
        last_d_acc = first_path.m_d_acc[path_size - 1];
    }

    // cout << "** LAST S = " << last_s << endl;
    // cout << "** END S = " << target_s << endl;
    std::vector<double> start_s = {last_s, last_s_vel, last_s_acc};
    std::vector<double> end_s = {target_s, target_s_speed, target_s_acc};

    std::vector<double> start_d = {last_d, last_d_vel, last_d_acc};
    std::vector<double> end_d = {target_d, target_d_speed, target_d_acc};

    std::vector<double> coeffs_s = this->JMT(start_s, end_s, m_time_interval);
    std::vector<double> coeffs_d = this->JMT(start_d, end_d, m_time_interval);

    this->appendPath(start_s, end_s, start_d, end_d, first_path, m_time_interval);
    paths.push_back(first_path);

    // Create a normal distribution and a time based seed
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> s_distrib(target_s, std_s);
    std::normal_distribution<double> d_distrib(target_d, std_d);

    for (int i = 1; i < count; ++i)
    {
        Path path = current_path.clone(from_point_index);
        double new_target_s = s_distrib(generator);
        double new_target_d = d_distrib(generator);

        start_s = {last_s, last_s_vel, last_s_acc};
        end_s = {new_target_s, target_s_speed, target_s_acc};

        start_d = {last_d, last_d_vel, last_d_acc};
        end_d = {new_target_d, target_d_speed, target_d_acc};

        coeffs_s = this->JMT(start_s, end_s, m_time_interval);
        coeffs_d = this->JMT(start_d, end_d, m_time_interval);

        this->appendPath(start_s, end_s, start_d, end_d, path, m_time_interval);

        paths.push_back(path);
    }

    return paths;
}

void PathGenerator::appendPath(std::vector<double> start_s, std::vector<double> end_s,
                               std::vector<double> start_d, std::vector<double> end_d,
                               Path &path, double time_interval)
{
    std::vector<double> coeffs_s = this->JMT(start_s, end_s, time_interval);
    std::vector<double> coeffs_d = this->JMT(start_d, end_d, time_interval);

    int total_points = time_interval / CONTROLLER_UPDATE_RATE_SECONDS;
    // cout << "^^^^^^ TOTAL POINTS " << total_points << endl;
    int points_remaining = total_points - path.size();
    Waypoints &waypoints = Waypoints::getInstance();

    double last_x = path.m_x[path.size() - 1];
    double last_y = path.m_y[path.size() - 1];

    double last_s = path.m_s[path.size() - 1];
    double last_d = path.m_d[path.size() - 1];

    for (int i = 0; i < points_remaining; ++i)
    {
        double t = CONTROLLER_UPDATE_RATE_SECONDS * (i + 1);
        double t_2 = pow(t, 2);
        double t_3 = pow(t, 3);
        double t_4 = pow(t, 4);
        double t_5 = pow(t, 5);

        double s_t = start_s[0] + start_s[1] * t + 0.5 * start_s[2] * t_2 + coeffs_s[3] * t_3 + coeffs_s[4] * t_4 + coeffs_s[5] * t_5;
        double s_t_dot = start_s[1] + start_s[2] * t + 3 * coeffs_s[3] * t_2 + 4 * coeffs_s[4] * t_3 + 5 * coeffs_s[5] * t_4;
        double s_t_dot_dot = start_s[2] + 6 * coeffs_s[3] * t + 12 * coeffs_s[4] * t_2 + 20 * coeffs_s[5] * t_3;
        double s_jerk = 6 * coeffs_s[3] + 24 * coeffs_s[4] * t + 60 * coeffs_s[5] * t_2;

        double d_t = start_d[0] + start_d[1] * t + start_d[2] * 0.5 * t_2 + coeffs_d[3] * t_3 + coeffs_d[4] * t_4 + coeffs_d[5] * t_5;
        double d_t_dot = start_d[1] + start_d[2] * t + 3 * coeffs_d[3] * t_2 + 4 * coeffs_d[4] * t_3 + 5 * coeffs_d[5] * t_4;
        double d_t_dot_dot = start_d[2] + 6 * coeffs_d[3] * t + 12 * coeffs_d[4] * t_2 + 20 * coeffs_d[5] * t_3;
        double d_jerk = 6 * coeffs_d[3] + 24 * coeffs_d[4] * t + 60 * coeffs_d[5] * t_2;

        std::vector<double> x_y = waypoints.toRealWorldXY(s_t, d_t);
        double x = x_y[0];
        double y = x_y[1];

        double theta = atan2(y - last_y, x - last_x);
        // TODO fix the theta angle
        path.add(x_y[0], x_y[1],
                 s_t, s_t_dot, s_t_dot_dot, s_jerk,
                 d_t, d_t_dot, d_t_dot_dot, d_jerk,
                 theta);

        double dist = distance(last_x, last_y, x, y);
        last_x = x;
        last_y = y;
    }
}

std::vector<double> PathGenerator::JMT(std::vector<double> start, std::vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    // create T matrix
    MatrixXd t_matrix(3, 3);
    t_matrix << pow(T, 3), pow(T, 4), pow(T, 5),
             3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
             6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

    // create sf diff vector
    VectorXd sf_diff(3);
    sf_diff << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)),
            end[1] - (start[1] + start[2] * T),
            end[2] - start[2];

    VectorXd coeffs = t_matrix.inverse() * sf_diff;

    std::vector<double> final_coeffs = {start[0], start[1], 0.5 * start[2], coeffs[0], coeffs[1], coeffs[2]};
    return final_coeffs;
}

PathGenerator::~PathGenerator() {}