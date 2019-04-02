#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::tuple;

/* Fit spline based on s */
tuple<tk::spline, tk::spline> fitSpline(
    const double car_s, const double car_d,
    const vector<double> &map_waypoints_s,
    const vector<double> &map_waypoints_x,
    const vector<double> &map_waypoints_y,
    const float spline_length)
{
    /* Set Spline parameters */
    int num_spline_pts = 50;
    float interval = spline_length / num_spline_pts;
    std::vector<double> spline_s, spline_x_pts, spline_y_pts;

    /* Add points to fit spline to */
    for (int i = 0; i < num_spline_pts; ++i)
    {
        double s = car_s + i * interval;
        double d = car_d;
        vector<double> point = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        spline_s.push_back(s);
        spline_x_pts.push_back(point[0]);
        spline_y_pts.push_back(point[1]);
    }

    tk::spline spline_x, spline_y;
    spline_x.set_points(spline_s, spline_x_pts);
    spline_y.set_points(spline_s, spline_y_pts);
    return std::make_tuple(spline_x, spline_y);
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;




  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    /* Initialization */
    int lane = 1;
    double speed = 22; // 50 MPH = 22.352 m/s
    double dt = 0.02;
    double path_length = 50; // 50 metres
    double ds = speed * dt; // Assuming constant velocity
    double path_time = path_length / speed; // Assuming constant velocity
    int num_path_points = (int)(path_time / dt);


    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /* Fit spline to points */
          tk::spline spline_x, spline_y;

          std::tie(spline_x, spline_y) = fitSpline(car_s, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y, path_length);

          /* Sample from spline */
          for (int i = 0; i < num_path_points; ++i)
          {
              double path_s = car_s + i*ds;
              next_x_vals.push_back(spline_x(path_s));
              next_y_vals.push_back(spline_y(path_s));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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