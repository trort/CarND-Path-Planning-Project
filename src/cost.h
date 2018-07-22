#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include <string>
#include <vector>
#include "json.hpp"
#include "helper_functions.h"

using namespace std;
using json = nlohmann::json;

double collision_cost(vector<double>& path_x, vector<double>& path_y, const double& dt, json& sensor_fusion,
                      const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
  double cost = 0;
  const double s_tolerance = 10;
  const double d_tolerance = 3.2;
  if (path_x.size() > 1 && sensor_fusion.size() >0) {
    for (int time_i = 1; time_i < path_x.size(); ++time_i) {
      double ego_x = path_x[time_i];
      double ego_y = path_y[time_i];
      double ego_yaw = atan2(path_y[time_i] - path_y[time_i - 1], path_x[time_i] - path_x[time_i]);

      for (int ss_i = 0; ss_i < sensor_fusion.size(); ++ ss_i) {
        double ss_x = sensor_fusion[ss_i][1];
        double ss_y = sensor_fusion[ss_i][2];
        double ss_vx = sensor_fusion[ss_i][3];
        double ss_vy = sensor_fusion[ss_i][4];
        double ss_s = sensor_fusion[ss_i][5];
        double ss_d = sensor_fusion[ss_i][6];
        double ss_speed = sqrt(ss_vx * ss_vx + ss_vy * ss_vy);

        ss_s += ss_speed * time_i * dt;
        ss_d += 0;

        vector<double> ss_xy = getXY(ss_s, ss_d, maps_s, maps_x, maps_y);
        vector<double> car_coord_sd = map_xy_to_ref(ss_xy[0], ss_xy[1], ego_x, ego_y, ego_yaw);

        if (abs(car_coord_sd[0]) < s_tolerance && abs(car_coord_sd[1]) < d_tolerance) {
          cost += (s_tolerance / abs(car_coord_sd[0]) + d_tolerance / abs(car_coord_sd[1]));
        }
      }
    }
  }
  return cost;
}

double speed_cost(vector<double>& path_x, vector<double>& path_y, const double& dt, const double& max_speed){
  double cost = 0;
  if (path_x.size() > 1) {
    for (int i = 1; i < path_x.size(); ++i) {
      double dx = (path_x[i] - path_x[i - 1]) / dt;
      double dy = (path_y[i] - path_y[i - 1]) / dt;
      double v = sqrt(dx * dx + dy * dy);
      cost += (v - max_speed) * (v - max_speed);
    }
    cost /= (path_x.size() - 1);
  }
  return cost;
}

double acceleration_cost(vector<double>& path_x, vector<double>& path_y, const double& dt){
  double cost = 0;
  if (path_x.size() > 2) {
    vector<double> vxs;
    vector<double> vys;
    for (int i = 1; i < path_x.size(); ++i) {
      double vx = (path_x[i] - path_x[i - 1]) / dt;
      double vy = (path_y[i] - path_y[i - 1]) / dt;
      vxs.push_back(vx);
      vys.push_back(vy);
    }
    for (int i = 1; i < vxs.size(); ++i) {
      double acc_s = (vxs[i] - vxs[i - 1]) / dt;
      double acc_d = (vys[i] - vys[i - 1]) / dt;
      cost += acc_s * acc_s + acc_d * acc_d;
    }
    cost /= (path_x.size() - 2);
  }
  return cost;
}

double lane_change_cost(string state){
  if (state.compare("LCL") == 0 || state.compare("LCR") == 0) return 1;  // try limit lane change
  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) return -0.1; // be more prepared
  else return 0;
}

double total_cost(vector<vector<double>>& trajectory, string& state, const double& dt, const double& max_speed,
                  json& sensor_fusion, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
  double collision_term = collision_cost(trajectory[0], trajectory[1], dt, sensor_fusion, maps_s, maps_x, maps_y);
  double speed_term = speed_cost(trajectory[0], trajectory[1], dt, max_speed);
  double acceleration_term = acceleration_cost(trajectory[0], trajectory[1], dt);
  double lane_change_term = lane_change_cost(state);
  double cost = 0;
  cost += 500 * collision_term;
  cost += 1 * speed_term;
  // cost += 1 * acceleration_term;
  cost += 1 * lane_change_term;
  cout << " total=" << cost << "; collision=" << collision_term << "; speed=" << speed_term
       << "; acc=" << acceleration_term << endl;
  return cost;
}

#endif //PATH_PLANNING_COST_H
