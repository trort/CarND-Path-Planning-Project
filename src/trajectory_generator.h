#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include <vector>
#include "helper_functions.h"
using namespace std;

class TrajectoryGenerator{
public:
  // map_data
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  TrajectoryGenerator(vector<vector<double>> map_waypoints){
    map_waypoints_x = map_waypoints[0];
    map_waypoints_y = map_waypoints[1];
    map_waypoints_s = map_waypoints[2];
    map_waypoints_dx = map_waypoints[3];
    map_waypoints_dy = map_waypoints[4];
  }

  ~TrajectoryGenerator() {}

  vector<vector<double>> generate_trajectory(int target_lane, double target_vel, Car ego){
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // wide-spaced way points
    vector<double> ptsx;
    vector<double> ptsy;

    /*if (ego.prev_size >= 2) {
      ptsx.push_back(ego.car_x);
      ptsy.push_back(ego.car_y);
    }*/
    ptsx.push_back(ego.end_x_prev);
    ptsx.push_back(ego.end_x);
    ptsy.push_back(ego.end_y_prev);
    ptsy.push_back(ego.end_y);

    double car_d = 2 + 4 * target_lane;
    vector<double> next_wp0 = getXY(ego.end_s + 30, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp0[0]); ptsy.push_back(next_wp0[1]);
    vector<double> next_wp1 = getXY(ego.end_s + 60, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp1[0]); ptsy.push_back(next_wp1[1]);
    vector<double> next_wp2 = getXY(ego.end_s + 90, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp2[0]); ptsy.push_back(next_wp2[1]);

    // transform to car coordinates and the end to ensure going forward
    for (int i = 0; i < ptsx.size(); ++i) {
      vector<double> transformed_xy = map_xy_to_ref(ptsx[i], ptsy[i], ego.end_x, ego.end_y, ego.end_yaw);
      ptsx[i] = transformed_xy[0];
      ptsy[i] = transformed_xy[1];
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    for (int i = 0; i < ego.prev_size; ++i) {
      next_x_vals.push_back(ego.previous_path_x[i]);
      next_y_vals.push_back(ego.previous_path_y[i]);
    }

    // fine-spaced points
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = distance(0, 0, target_x, target_y);
    double x_add_on = 0;

    double car_vel = ego.end_speed;

    for (int i = 0; i < 80 - ego.prev_size; ++i) {

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

      vector<double> map_xy = car_xy_to_map(x_point, y_point, ego.end_x, ego.end_y, ego.end_yaw);

      next_x_vals.push_back(map_xy[0]);
      next_y_vals.push_back(map_xy[1]);
    }
    return {next_x_vals, next_y_vals};
  }
};

#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
