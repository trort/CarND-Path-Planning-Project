#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include "spline.h"
#include "json.hpp"
#include "helper_functions.h"

using namespace std;
using json = nlohmann::json;

class Car{
public:
  const map<string, int> lane_delta = {{"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"PLCR", 1}, {"LCR", 1}};
  const double INTERVAL = 0.02;
  const double MAX_VEL = 49.5 / 2.24; // convert mph to m/s

  string state;
  int lane;

  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  // Previous path data given to the Planner
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  int prev_size;

  // state at the end of the previous path
  double end_x;
  double end_y;
  double end_s;
  double end_d;
  double end_yaw;
  double end_speed;
  double end_x_prev;
  double end_y_prev;

  // environment data
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Car(){
    state = "KL";
  }
  ~Car(){}

  void update_car(json j){
    car_x = j["x"];
    car_y = j["y"];
    car_s = j["s"];
    car_d = j["d"];
    car_yaw = deg2rad(j["yaw"]);
    car_speed = double(j["speed"]) / 2.24;
    end_s = j["end_path_s"];
    end_d = j["end_path_d"];
    lane = d2lane(car_d);

    prev_size = min(int(j["previous_path_x"].size()), 40);    // load no more than 40 points (0.8s) to allow path innovation
    previous_path_x.clear(); previous_path_y.clear();
    for (int i = 0; i < prev_size; ++i) {
      previous_path_x.push_back(j["previous_path_x"][i]);
      previous_path_y.push_back(j["previous_path_y"][i]);
    }

    if(prev_size < 2){
      end_x = car_x;
      end_y = car_y;
      end_yaw = car_yaw;
      end_speed = car_speed;
      end_x_prev = end_x - cos(end_yaw);
      end_y_prev = end_y - sin(end_yaw);
      end_s = car_s;
      end_d = car_d;
    }
    else {
      end_x = previous_path_x[prev_size-1];
      end_y = previous_path_y[prev_size-1];
      end_x_prev = previous_path_x[prev_size-2];
      end_y_prev = previous_path_y[prev_size-2];
      end_yaw = atan2(end_y - end_y_prev, end_x - end_x_prev);
      end_speed = distance(end_x, end_y, end_x_prev, end_y_prev) / INTERVAL;
    }
    print_car_state();
  }

  void update_map(vector<vector<double>> map_waypoints){
    map_waypoints_x = map_waypoints[0];
    map_waypoints_y = map_waypoints[1];
    map_waypoints_s = map_waypoints[2];
    map_waypoints_dx = map_waypoints[3];
    map_waypoints_dy = map_waypoints[4];
  }

  void print_car_state(){
    cout << "Car state: x="<< car_x << "; y=" << car_y << "; s=" << car_s << "; d=" << car_d
         << "; speed=" << car_speed << "; yaw=" << car_yaw << endl;
    cout << "end_x=" << end_x << "; end_y=" << end_y << "; end_x_prev=" << end_x_prev
         << "; end_y_prev=" << end_y_prev << "; end_speed=" << end_speed << "; end_yaw=" << end_yaw << endl;
  }

  vector<string> get_next_states(){
    vector<string> next_states;
    next_states.push_back("KL");
    if (state.compare("KL") == 0) {
      if (lane > 0) next_states.push_back("PLCL");
      if (lane < 2) next_states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0) {
      if (lane > 0) {
        next_states.push_back("PLCL");
        next_states.push_back("LCL");
      }
    }
    else if (state.compare("PLCR") == 0) {
      if (lane > 0) {
        next_states.push_back("PLCR");
        next_states.push_back("LCR");
      }
    }
    return next_states;
  }

  vector<vector<double>> generate_trajectory(int target_lane, double target_vel){
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // wide-spaced way points
    vector<double> ptsx;
    vector<double> ptsy;

    ptsx.push_back(end_x_prev);
    ptsx.push_back(end_x);
    ptsy.push_back(end_y_prev);
    ptsy.push_back(end_y);

    double car_d = 2 + 4 * target_lane;
    vector<double> next_wp0 = getXY(end_s + 30, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp0[0]); ptsy.push_back(next_wp0[1]);
    vector<double> next_wp1 = getXY(end_s + 60, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp1[0]); ptsy.push_back(next_wp1[1]);
    vector<double> next_wp2 = getXY(end_s + 90, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp2[0]); ptsy.push_back(next_wp2[1]);

    // transform to car coordinates and the end to ensure going forward
    for (int i = 0; i < ptsx.size(); ++i) {
      vector<double> transformed_xy = map_xy_to_ref(ptsx[i], ptsy[i], end_x, end_y, end_yaw);
      ptsx[i] = transformed_xy[0];
      ptsy[i] = transformed_xy[1];
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    for (int i = 0; i < prev_size; ++i) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    // fine-spaced points
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    double x_add_on = 0;

    double car_vel = end_speed;

    for (int i = 0; i < 80 - prev_size; ++i) {

      if (car_vel > target_vel) {
        car_vel -= 0.1;
      }
      else if (car_vel < target_vel) {
        car_vel += 0.1;
      }

      double N = target_dist / (INTERVAL * car_vel);
      double x_point = x_add_on + (target_x) / N;
      double y_point = s(x_point);

      x_add_on = x_point;

      vector<double> map_xy = car_xy_to_map(x_point, y_point, end_x, end_y, end_yaw);

      next_x_vals.push_back(map_xy[0]);
      next_y_vals.push_back(map_xy[1]);
    }
    return {next_x_vals, next_y_vals};
  }

  vector<vector<double>> generate_path(json j){
    update_car(j);
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = j["sensor_fusion"];

    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02
    // Generate OUTPUT DATA
    int ref_lane = 1;
    vector<double> ref_vel = {MAX_VEL, MAX_VEL, MAX_VEL};

    // sensor fusion part

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

          if ((check_car_s > car_s) && ((check_car_s - car_s) < 40)) {
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
    for (int check_lane = 0; check_lane < 3; ++ check_lane) {
      if (ref_vel[check_lane] > ref_vel[target_lane] + 1){
        target_lane = check_lane;
      }
    }


    // starting points as either the current state or the end of previous p

    // debug
    cout << target_lane << " " << ref_vel[target_lane] << endl;

    return generate_trajectory(target_lane, ref_vel[target_lane]);
  }
};

#endif //PATH_PLANNING_CAR_H
