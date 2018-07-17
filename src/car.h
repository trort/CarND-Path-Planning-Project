#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <algorithm>
#include "json.hpp"
#include "helper_functions.h"

using namespace std;
using json = nlohmann::json;

class Car{
public:
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

  Car(){}
  ~Car(){}

  void update(json j){
    car_x = j["x"];
    car_y = j["y"];
    car_s = j["s"];
    car_d = j["d"];
    car_yaw = deg2rad(j["yaw"]);
    car_speed = double(j["speed"]) / 2.24;
    end_s = j["end_path_s"];
    end_d = j["end_path_d"];

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

  void print_car_state(){
    cout << "Car state: x="<< car_x << "; y=" << car_y << "; s=" << car_s << "; d=" << car_d
         << "; speed=" << car_speed << "; yaw=" << car_yaw << endl;
    cout << "end_x=" << end_x << "; end_y=" << end_y << "; end_x_prev=" << end_x_prev
         << "; end_y_prev=" << end_y_prev << "; end_speed=" << end_speed << "; end_yaw=" << end_yaw << endl;
  }
};

#endif //PATH_PLANNING_CAR_H
