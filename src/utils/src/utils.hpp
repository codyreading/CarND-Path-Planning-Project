#ifndef UTILS_HPP
#define UTILS_HPP

#include <math.h>
#include <string>
#include <vector>

#include "path.hpp"
#include "frenet_point.hpp"
#include "json.hpp"
#include "vehicle.hpp"

// ~80 KPH (50 MPH)
const double MAX_LEGAL_SPEED_LIMIT_METERS_PER_SECOND = 22.2;

const double MAX_SPEED_METERS_PER_SECOND = 22.0;

// ~30 KPH
const double MIN_SPEED_FOR_LANE_CHANGE_METERS_PER_SECOND = 8.35;

const double CONTROLLER_UPDATE_RATE_SECONDS = 0.02;

const double VEHICLE_DISTANCE_THRESHOLD_METERS = 50.0;

const double LANE_CHANGE_VEHICLE_AHEAD_DISTANCE_THRESHOLD_METERS = 25.0;

const double LANE_CHANGE_VEHICLE_BEHIND_DISTANCE_THRESHOLD_METERS = 15.0;

const double VEHICLE_MIN_SECURITY_DISTANCE_LANE_CHANGE_METERS = 15.0;

const double VEHICLE_COLLISION_THRESHOLD_METERS = 15.0;

const double COLLISION_MAX_TIMESTEP_THRESHOLD = 75.0;

const double MAX_TRACK_S = 6945.554;

const int LANES_COUNT = 3;
const double DEFAULT_LANE_SPACING = 4.0;
const double DEFAULT_LANE_INSIDE_OFFSET = 1.5;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double getTheta(double vx, double vy);

/**
 *
 * @returns the calculated lane index
 */
int calculateLane(double d, double lane_spacing, double lane_inside_offset);

bool isWithinLane(double d, double lane_spacing, double lane_inside_offset);

/**
 * @brief Checks whether the lane id is a valid one
 *
 * @param lane the lane id
 * @return true if the id is within our range of admissible lanes
 * @return false if the id is outside of our admissible lane range
 */
bool isLaneValid(int lane);

/**
 * @brief Get the appropriate lane center value (d_center) in Frenet coordinates
 *
 * @param lane the lane id
 * @return double the lane center in frenet coordinates
 */
double getLaneCenterFrenet(int lane);

double milesPerHourToKmPerHour(double mph);

double KmPerHourToMetersPerSecond(double kmh);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);


/* Type conversions */
FrenetPoint json_to_point(nlohmann::basic_json<> s, nlohmann::basic_json<> d);
std::vector<Vehicle> sensor_fusion_to_vehicles(const std::vector<std::vector<double>>& sensor_fusion);

#endif  // UTILS_HPP