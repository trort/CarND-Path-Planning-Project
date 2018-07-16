#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <algorithm>
#include "json.hpp"

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

  Car(json j){
    car_x = j["x"];
    car_y = j["y"];
    car_s = j["s"];
    car_d = j["d"];
    car_yaw = j["yaw"];
    car_speed = j["speed"];

    prev_size = min(int(j["previous_path_x"].size()), 50);    // load no more than 30 points (0.6s) to allow path innovation
    previous_path_x.clear(); previous_path_y.clear();
    for (int i = 0; i < prev_size; ++i) {
      previous_path_x.push_back(j["previous_path_x"][i]);
      previous_path_y.push_back(j["previous_path_y"][i]);
    }
  }
  ~Car(){}

  void print_car_state(){
    cout << "Car state: x="<< car_x << "; y=" << car_y << "; s=" << car_s << "; d=" << car_d
         << "; speed=" << car_speed << endl;
  }
};

#endif //PATH_PLANNING_CAR_H
