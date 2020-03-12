#ifndef PATH_HPP
#define PATH_HPP

#include <vector>
#include "json.hpp"

class Path
{
public:
    Path();
    Path(const std::vector<double>& x, const std::vector<double>& y);
    Path(nlohmann::basic_json<> x, nlohmann::basic_json<> y);
    int size();

    /**
    * Removes the first n waypoints in the trajectory
    */
    void removeFirstPoints(int numPoints);

    /**
     * Add a new point to the trajectory
     */
    void add(double x, double y,
             double s, double s_vel, double s_acc, double s_jerk,
             double d, double d_vel, double d_acc, double d_jerk,
             double yaw);

    /**
     * Clones the underlying trajectory from points [0, up_to_index)
     * @param up_to_index
     * @return a new trajectory that contains all points up to the specified index excluded
     */
    Path clone(int up_to_index);


    std::vector<double> m_x; // List of x coordinates to define a path
    std::vector<double> m_y; // List of y coordinates to define a path
    std::vector<double> m_s; // List of s coordinates to define a path
    std::vector<double> m_d; // List of d coordinates to define a path
    std::vector<double> m_s_vel;
    std::vector<double> m_d_vel;
    std::vector<double> m_s_acc;
    std::vector<double> m_d_acc;
    std::vector<double> m_s_jerk;
    std::vector<double> m_d_jerk;
    std::vector<double> m_yaw;
};

#endif