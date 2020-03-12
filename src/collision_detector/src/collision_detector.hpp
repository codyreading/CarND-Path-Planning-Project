#ifndef COLLISION_DETECTOR_HPP
#define COLLISION_DETECTOR_HPP

#include "vehicle.hpp"
#include "path.hpp"

class Collision
{
public:
    const Vehicle& v;
    const bool willCollide;
    const double collision_point_x;
    const double collision_point_y;
    const double collision_timestep;

    Collision(const Vehicle& v, const bool willCollide,
              const double collision_point_x, const double collision_point_y, const double collision_timestep);
    ~Collision();
};


/**
 * Computes the coordinates (x, y) and time of a potential collision with another vehicle.
 * We assume constant speed and zero acceleration for the passed vehicle
 * @param vehicle the vehicle we want to check whether we will collide with
 * @param timestep the timestep unit (e.g. 0.5s, 1s, etc.) between successive points
 *
 */
Collision predictCollision(const Vehicle &vehicle, double timestep);

#endif
