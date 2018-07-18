#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include <string>

using namespace std;

double collision_cost(){
  return 0;
}

double speed_cost(){
  return 0;
}

double acceleration_cost(){
  return 0;
}

double lane_change_cost(string state){
  return 0;
}

double total_cost(string state){
  double cost = 0;
  cost += 1 * collision_cost();
  cost += 1 * speed_cost();
  cost += 1 * acceleration_cost();
  cost += 1 * lane_change_cost(state);
  return cost;
}

#endif //PATH_PLANNING_COST_H
