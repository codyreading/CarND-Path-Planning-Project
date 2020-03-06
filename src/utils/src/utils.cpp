#include <math.h>
#include <string>
#include <vector>

#include "utils.hpp"


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
//   else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 *
 * @returns the angle theta in radians, based on the supplied velocity components
 */
double getTheta(double vx, double vy)
{
  return atan2(vy, vx);
}


// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/*
 *
 * @returns the calculated lane index
 */
int calculateLane(double d, double lane_spacing, double lane_inside_offset)
{
  double calibrated_d = d - lane_inside_offset;
  if (calibrated_d < 0.0)
  {
    return -1;
  }
  return (int)floor(calibrated_d / lane_spacing);
}

bool isWithinLane(double d, double lane_spacing, double lane_inside_offset)
{
  double calibrated_d = d - lane_inside_offset;
  if (calibrated_d < 0.0)
  {
    return false;
  }

  int target_lane = (int)floor(d / lane_spacing);
  int calibrated_lane = (int)floor(calibrated_d / lane_spacing);
  return target_lane == calibrated_lane;
}

bool isLaneValid(int lane)
{
  return lane >= 0 && lane < LANES_COUNT;
}

double getLaneCenterFrenet(int lane)
{
  // return 2.0 + 4.0 * lane;
  return 1.8 + 4.0 * lane;
}

double milesPerHourToKmPerHour(double mph)
{
  return mph * 1.609344;
}

double KmPerHourToMetersPerSecond(double kmh)
{
  return (kmh * 1000.0) / 3600.0;
}


Path json_to_path(nlohmann::basic_json<> x, nlohmann::basic_json<> y)
{
  Path path;
  for (int i = 0; i < x.size(); ++i)
  {
    path.x.push_back(x[i]);
    path.y.push_back(y[i]);
  }
  return path;
}

FrenetPoint json_to_point(nlohmann::basic_json<> s, nlohmann::basic_json<> d)
{
  FrenetPoint point;
  point.s = s;
  point.d = d;
  return point;
}

std::vector<Vehicle> sensor_fusion_to_vehicles(const std::vector<std::vector<double>>& sensor_fusion)
{
  std::vector<Vehicle> vehicles;
  for (auto data : sensor_fusion)
  {
    Vehicle vehicle = Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6], 0.0);
    vehicles.push_back(vehicle);
  }
}
