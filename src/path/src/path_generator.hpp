#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H
#include <iostream>
#include <vector>
#include "path.hpp"
#include "vehicle.hpp"
#include "state_machine.hpp"


class PathGenerator
{
public:
    PathGenerator();
    PathGenerator(double time_interval);
    ~PathGenerator();


    // TODO When generating path we need to give the option on whether
    // we want to force a lane centering or simply use the current lane
    // coordinate d

    /**
     * Generates a number of paths given a set of target conditions
     *
     */
    std::vector<Path> generatePaths(Path &current_path,
                                    double target_s, double target_d,
                                    double target_s_speed, double target_d_speed,
                                    double target_s_acc, double target_d_acc,
                                    double std_s, double std_d,
                                    int count, int from_point_index);

    std::vector<Path> generatePaths(const State& state, const Vehicle& ego,
                                    Path &current_path, int from_point_index, int path_count);




private:

    /**
     * @brief Computes the coefficients of the quintic polynomial for a jerk
     * minimized path
     *
     * @param start the start position, velocity and acceleration
     * @param end the desired end position, velocity and acceleration
     * @param T the time period in seconds
     * @return std::vector<double> the 6 coefficients vector of the quintic polynomial that minimizes jerk
     */
    std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);


    /**
     * @brief Appends a path with the start and end targets in Frenet coordinates
     * to the supplied path
     *
     * @param start_s the vector of longitudinal start  position , velocity and acceleration
     * @param end_s the vector of longitudinal end position, velocity and acceleration
     * @param start_d the vector of lateral start position, velocity and acceleration
     * @param end_d the vector of lateral end position, velocity and acceleration
     * @param path the path to append to
     * @param time_interval the time interval in seconds to execute this path
     */
    void appendPath(std::vector<double> start_s, std::vector<double> end_s,
                    std::vector<double> start_d, std::vector<double> end_d,
                    Path &path, double time_interval);

    double m_time_interval;
};


#endif