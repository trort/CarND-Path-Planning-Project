#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "car.h"
#include "trajectory_generator.h"
#include "helper_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

vector<vector<double>> generate_path(json j, vector<vector<double>> map_waypoints){
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = j["sensor_fusion"];

  Car ego(j);
  TrajectoryGenerator trajectory_generator(map_waypoints);

  // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02
  // Generate OUTPUT DATA
  int ref_lane = 1;
  vector<double> ref_vel = {MAX_VEL, MAX_VEL, MAX_VEL};

  // sensor fusion part
  //bool too_close = false;
  double prev_size = ego.prev_size;

  for (int check_lane = 0; check_lane < 3; ++ check_lane) {
    for (int i = 0; i < sensor_fusion.size(); ++i) {
      // unpack sensor fusion data
      int ss_id = sensor_fusion[i][0];
      double ss_x = sensor_fusion[i][1];
      double ss_y = sensor_fusion[i][2];
      double ss_vx = sensor_fusion[i][3];
      double ss_vy = sensor_fusion[i][4];
      double ss_s = sensor_fusion[i][5];
      double ss_d = sensor_fusion[i][6];

      if (ss_d < (2 + 4 * check_lane + 2) && ss_d > (2 + 4 * check_lane - 2)) {
        double ss_speed = sqrt(ss_vx * ss_vx + ss_vy * ss_vy);
        double check_car_s = ss_s + prev_size * INTERVAL * ss_speed;

        if ((check_car_s > ego.car_s) && ((check_car_s - ego.car_s) < 40)) {
          //ref_vel = ss_speed * 0.9;
          //too_close = true;
          if (ref_vel[check_lane] > ss_speed) {
            ref_vel[check_lane] = ss_speed;
          }
        }
      }
    }
  }
  int target_lane = 1;
  double target_vel;
  for (int check_lane = 0; check_lane < 3; ++ check_lane) {
    if (ref_vel[check_lane] > ref_vel[target_lane] + 1){
      target_lane = check_lane;
      target_vel = ref_vel[target_lane];
    }
  }


  // starting points as either the current state or the end of previous p

  // debug
  ego.print_car_state();
  cout << target_lane << " " << target_vel << endl;

  return trajectory_generator.generate_trajectory(target_lane, ref_vel[target_lane], ego);
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
    double s;
    double d_x;
    double d_y;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          json msgJson;
          vector<vector<double>> map_waypoints = {map_waypoints_x, map_waypoints_y, map_waypoints_s,
                                                  map_waypoints_dx, map_waypoints_dy};

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<vector<double>> planed_path = generate_path(j[1], map_waypoints);
          next_x_vals = planed_path[0];
          next_y_vals = planed_path[1];

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
