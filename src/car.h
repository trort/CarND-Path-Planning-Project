#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include "spline.h"
#include "json.hpp"
#include "helper_functions.h"
#include "cost.h"

using namespace std;
using json = nlohmann::json;

class Car{
public:
  const map<string, int> lane_delta = {{"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"PLCR", 1}, {"LCR", 1}};
  const double INTERVAL = 0.02;
  const double MAX_VEL = 49.5 / 2.24; // convert mph to m/s

  string state;
  int lane;
  int target_lane;

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
    lane = 1;
    target_lane = -1;
  }
  ~Car(){}

  void update_car(json& j){
    car_x = j["x"];
    car_y = j["y"];
    car_s = j["s"];
    car_d = j["d"];
    car_yaw = deg2rad(j["yaw"]);
    car_speed = double(j["speed"]) / 2.24;
    end_s = j["end_path_s"];
    end_d = j["end_path_d"];
    lane = d2lane(car_d);
    if (target_lane == -1) target_lane = lane;

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

  void update_map(vector<vector<double>>& map_waypoints){
    map_waypoints_x = map_waypoints[0];
    map_waypoints_y = map_waypoints[1];
    map_waypoints_s = map_waypoints[2];
    map_waypoints_dx = map_waypoints[3];
    map_waypoints_dy = map_waypoints[4];
  }

  void print_car_state(){
    cout << "Car location: x="<< car_x << "; y=" << car_y << "; s=" << car_s << "; d=" << car_d
         << "; speed=" << car_speed << "; yaw=" << car_yaw << endl;
    //cout << "End location: x="<< end_x << "; y=" << end_y << "; s=" << end_s << "; d=" << end_d
    //     << "; speed=" << end_speed << "; yaw=" << end_yaw << endl;
  }

  vector<string> get_next_states(){
    vector<string> next_states;
    if (lane == target_lane) {     // not during lane change
      next_states.push_back("KL");
      if (state.compare("KL") == 0) {
        if (lane > 0) next_states.push_back("PLCL");
        if (lane < 2) next_states.push_back("PLCR");
      } else if (state.compare("PLCL") == 0) {
        if (lane > 0) {
          next_states.push_back("PLCL");
          next_states.push_back("LCL");
        }
      } else if (state.compare("PLCR") == 0) {
        if (lane < 2) {
          next_states.push_back("PLCR");
          next_states.push_back("LCR");
        }
      }
    }
    else {      // keep finish lane change
      next_states.push_back(state);
    }
    return next_states;
  }

  vector<vector<double>> generate_trajectory(string next_state, vector<double>& ref_vel){
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // wide-spaced way points
    vector<double> ptsx;
    vector<double> ptsy;

    ptsx.push_back(end_x_prev);
    ptsx.push_back(end_x);
    ptsy.push_back(end_y_prev);
    ptsy.push_back(end_y);

    int target_lane = lane + lane_delta.at(next_state);
    double target_vel = ref_vel[lane];
    if (next_state.compare("PLCL") == 0 || next_state.compare("PLCR") == 0) {
      target_vel = min(ref_vel[target_lane], target_vel);
    }
    else if (next_state.compare("LCL") == 0 || next_state.compare("LCR") == 0){
      target_vel = ref_vel[target_lane];
    }
    int next_lane = lane;
    if (next_state.compare("LCL") == 0 || next_state.compare("LCR") == 0) {
      next_lane = target_lane;
    }
    double next_d = 2 + 4 * next_lane;
    vector<double> next_wp0 = getXY(end_s + 30, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp0[0]); ptsy.push_back(next_wp0[1]);
    vector<double> next_wp1 = getXY(end_s + 60, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp1[0]); ptsy.push_back(next_wp1[1]);
    vector<double> next_wp2 = getXY(end_s + 90, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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

  vector<double> get_lane_vel(json& sensor_fusion){
    vector<double> ref_vel = {MAX_VEL, MAX_VEL, MAX_VEL};

    // sensor fusion analysis
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

        if (ss_d < (2 + 4 * check_lane + 2) && ss_d > (2 + 4 * check_lane - 2)) { // same lane
          double ss_speed = sqrt(ss_vx * ss_vx + ss_vy * ss_vy);
          double check_car_s = ss_s + prev_size * INTERVAL * ss_speed;

          if ((check_car_s >= end_s) && ((check_car_s - end_s) < 30)) {
            ref_vel[check_lane] = min(ref_vel[check_lane], ss_speed);
          }
        }
      }
    }
    return ref_vel;
  }

  vector<vector<double>> generate_path(json& j){
    update_car(j);
    vector<double> ref_vel = get_lane_vel(j["sensor_fusion"]);
    cout << "Lane speed: " << ref_vel[0] << " | " << ref_vel[1] << " | " << ref_vel[2] << endl;

    vector<string> next_states = get_next_states();
    vector<vector<vector<double>>> next_trajectories;
    vector<double> next_costs;
    for(int i = 0; i < next_states.size(); ++i) {
      string next_state = next_states[i];
      vector<vector<double>> next_trajectory = generate_trajectory(next_state, ref_vel);
      next_trajectories.push_back(next_trajectory);
      cout << "Cost for " << next_state << " is:";
      /*vector<double> next_trajectory_s;
      vector<double> next_trajectory_d;
      for (int time_i = 0; time_i < next_trajectory[0].size(); time_i ++){
        vector<double> next_sd = getFrenet(next_trajectory[0][time_i], next_trajectory[1][time_i], car_yaw, map_waypoints_x, map_waypoints_y);
        cout << next_trajectory[0][time_i] << "," << next_trajectory[1][time_i] << " ";
        next_trajectory_s.push_back(next_sd[0]);
        next_trajectory_d.push_back(next_sd[1]);
      }
      cout << endl;
      vector<vector<double>> next_trajectory_sd = {next_trajectory_s, next_trajectory_d};*/
      double next_cost = total_cost(next_trajectory, next_state, INTERVAL, MAX_VEL, j["sensor_fusion"],
                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
      next_costs.push_back(next_cost);
    }

    int best_next_idx = 0;
    double best_cost = next_costs[best_next_idx];
    for (int i = 1; i < next_costs.size(); ++i){
      if (next_costs[i] < best_cost){
        best_next_idx = i;
        best_cost = next_costs[i];
      }
    }
    state = next_states[best_next_idx];
    if (state.compare("LCL") == 0 || state.compare("LCR") == 0){
      target_lane = lane + lane_delta.at(state);
    }
    cout << "Selected state: " << state << "; current lane = " << lane << "; target lane = " << target_lane << endl;

    return next_trajectories[best_next_idx];
  }
};

#endif //PATH_PLANNING_CAR_H
