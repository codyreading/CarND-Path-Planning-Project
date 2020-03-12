#ifndef PATH_VALIDATOR_HPP
#define PATH_VALIDATOR_HPP
#include "vehicle.hpp"
#include "state_machine.hpp"
#include "path.hpp"
#include <vector>

enum PathValidationStatus
{
    VALID,
    OUTSIDE_OF_LANE,
    VELOCITY_TOO_LOW_FOR_LANE_CHANGE,
    COLLISION_VEHICLE_AHEAD,
    COLLISION_VEHICLE_ADJACENT,
    AVERAGE_SPEED_BELOW_THRESHOLD,
    VELOCITY_ABOVE_THRESHOLD,
    TOTAL_ACCELERATION_ABOVE_THRESHOLD
};


PathValidationStatus path_validate(const Vehicle &v, std::vector<Vehicle> others, const State &state, const Path &path, int from_point);

#endif