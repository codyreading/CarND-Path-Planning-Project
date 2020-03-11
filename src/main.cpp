#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// #include "third-party/Eigen-3.3/Eigen/Core"
// #include "third-party/Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "path.hpp"
#include "frenet_point.hpp"
#include "planner.hpp"
#include "utils.hpp"


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Waypoints &waypoints = Waypoints::getInstance();

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    waypoints.addWaypoint(x, y, s, d_x, d_y);
  }

  waypoints.buildSplines();

  int lane = 1;
  double speed = 22; // 50 MPH = 22.352 m/s
  double dt = 0.02;
  double path_length = 50; // 50 metres
  Planner planner(lane, speed, dt, path_length);

  h.onMessage([&planner]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  uWS::OpCode opCode) {


    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = nlohmann::json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data nlohmann::json object

          // Main car's localization Data
          double car_yaw_rad = deg2rad(j[1]["yaw"]);
          Vehicle ego = Vehicle(-1, j[1]["x"], j[1]["y"], cos(car_yaw_rad), sin(car_yaw_rad), j[1]["s"], j[1]["d"], 0.0);;

          // Previous path data given to the Planner
          Path prev_path = Path(j[1]["previous_path_x"], j[1]["previous_path_y"]);

          // Previous path's end s and d values
          FrenetPoint prev_point = json_to_point(j[1]["end_path_s"], j[1]["end_path_d"]);

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          std::vector<Vehicle> sur_vehicles = sensor_fusion_to_vehicles(sensor_fusion);

          Path plan_path = planner.planPath(ego, prev_path, prev_point, sur_vehicles);

          nlohmann::json msgJson;
          msgJson["next_x"] = plan_path.m_x;
          msgJson["next_y"] = plan_path.m_y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
  char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}