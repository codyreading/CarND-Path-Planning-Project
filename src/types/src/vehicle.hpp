#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include <iostream>
#include <vector>
#include "waypoints.hpp"

using namespace std;

class Vehicle
{
public:
    int m_id;

    double m_x;
    double m_y;

    double m_vx;
    double m_vy;

    double m_s;
    double m_d;

    // Timestep
    double m_t;

    // We can compute those
    int m_lane;
    bool m_isInLane;
    double m_theta;

    /**
     * Constructors
     */
    Vehicle();
    Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double t);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    // Returns a new vehicle at the next timestep
    Vehicle predictNextPosition(double t1);

    /**
     * @brief Predicts the vehicle's future position at time t (in seconds)
     * We assume constant speed and 0 acceleration
     *
     * @param t the timestep into the future in seconds
     * @return Vehicle the vehicle at timestep t in the future
     */
    Vehicle predictFuturePosition(double t) const;

    /**
     * @brief Checks which vehicles are ahead of the current vehicle on the given lane
     *
     * @param others the other vehicles which may be ahead
     * @param lane the lane to check against
     * @return vector<Vehicle>  the vehicles ahead of the current vehicle
     */
    vector<Vehicle> ahead(const vector<Vehicle>& others, int lane) const;

    /**
     * @brief Checks which vehicles are behind of the current vehicle on the given lane
     *
     * @param others the other vehicles which may be behind
     * @param lane the lane to check against
     * @return vector<Vehicle>  the vehicles behind of the current vehicle
     */
    vector<Vehicle> behind(const vector<Vehicle>& others, int lane) const;



    double getSpeed() const;
};
#endif