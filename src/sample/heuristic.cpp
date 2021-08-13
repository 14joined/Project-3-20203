#include<cmath>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<cstring>
#include<math.h>
#include<fstream>
#include <limits.h>
#include <vector>
#include <algorithm>

#include "heuristic.hpp"
#include "EVRP.hpp"

using namespace std;


solution *best_sol;   //see heuristic.hpp for the solution structure
double alpha, beta;
double** pheromone;

/*initialize the structure of your heuristic in this function*/
void initialize_heuristic(){

  best_sol = new solution;
  best_sol->tour = new int[NUM_OF_CUSTOMERS+1000];
  best_sol->id = 1;
  best_sol->steps = 0;
  best_sol->tour_length = INT_MAX;
}

int roulette_wheel_selection(vector<int> input, int i) {
  int result = 0;
  double max_value = 0;
  for (int j = 0; j < input.size(); ++j) {
    double prob = pheromone[i][input[j]] / get_distance(i, input[j]);
    if (max_value < prob) {
      max_value = prob;
      result = j;
    }
  }
  return result;
}

vector<vector<int>> route_construction() {
  vector<int> route, input;
  int i;
  for (i = 0; i <= NUM_OF_CUSTOMERS; ++i) {
    input.push_back(i);
  }
  while (!input.empty()) {
    i = roulette_wheel_selection(input, route.back());
    route.push_back(input[i]);
    swap(input[i], input.back());
    input.pop_back();
  }
  for (i = 0; i < route.size(); ++i) {
    if (route[i] == 0) break;
  }
  rotate(route.begin(), route.begin() + i, route.end());

  vector<double> v(NUM_OF_CUSTOMERS + 1, 2e9);
  v[0] = 0;
  vector<int> p(NUM_OF_CUSTOMERS + 1, 0);
  for (i = 1; i <= NUM_OF_CUSTOMERS; ++i) {
    int j = i, tc = 0, td = 0;
    while (j <= NUM_OF_CUSTOMERS && tc <= MAX_CAPACITY) {
      tc += get_customer_demand(route[j]);
      if (i == j) {
	td = get_distance(0, route[j]) + get_distance(route[j], 0);
      } else {
	td = td - get_distance(route[j - 1], 0) + get_distance(route[j - 1], route[j]) + get_distance(route[j], 0);
      }
      if (tc <= MAX_CAPACITY) {
	j += 1;
	if (v[i - 1] + td < v[j]) {
	  v[j] = v[i - 1] + td;
	  p[j] = i - 1;
	}
      }
    }
  }

  vector<vector<int>> T;
  for (i = NUM_OF_CUSTOMERS; i > 0; ) {
    vector<int> t;
    t.push_back(0);
    for (int j = p[i] + 1; j <= i; ++j) {
      t.push_back(route[j]);
    }
    t.push_back(0);
    T.push_back(t);
    i = p[i];
  }

  return T;
}

vector<int> removal_heuristic(vector<int> route) {
  double D = BATTERY_CAPACITY / get_consumption_rate();
  double ld = D;
  vector<int> opt_route;
  for (int i = 0; i < route.size() - 1; ++i) {
    opt_route.push_back(route[i]);
    int s = 0;
    double ds = 2e9;
    for (int j = NUM_OF_CUSTOMERS + 1; j <= ACTUAL_PROBLEM_SIZE; ++j) {
      double val = get_distance(route[i], j) + get_distance(j, route[i + 1]);
      if (ds > val && get_distance(route[i], j) < ld) {
	s = j;
	ds = val;
      }
      ld = D - get_distance(s, route[i + 1]);
    }
    opt_route.push_back(-s);
  }
  opt_route.push_back(route.back());

  while (opt_route.size() > route.size()) {
    int s = 0;
    double ds = 0;
    ld = 0;
    
    vector<int> next_station(opt_route.size());
    vector<double> total_distance;
    total_distance.push_back(0);
    for (int i = 1; i < opt_route.size(); ++i) {
      total_distance.push_back(opt_route[i] <= 0 ? 0 : total_distance.back() + get_distance(opt_route[i - 1], opt_route[i]));
    }
    for (int i = opt_route.size() - 1; i >= 0; --i) {
      next_station[i] = opt_route[i] <= 0 ? i : next_station[i + 1];
    }

    for (int i = 1; i < opt_route.size() - 1; ++i) {
      int prev = abs(opt_route[i - 1]);
      if (opt_route[i] > 0) {
	ld += get_distance(prev, opt_route[i]);
	continue;
      }
      int station = -opt_route[i];
      int next = abs(opt_route[i + 1]);
      if (ld + total_distance[next_station[i + 1]] - total_distance[i - 1] < D) {
	double loss = get_distance(prev, station) + get_distance(station, next) - get_distance(prev, next);
	if (ds < loss) {
	  ds = loss;
	  s = i;
	}
      }
      ld = 0;

      if (ds != 0) {
	opt_route.erase(opt_route.begin() + s);
      } else break;
    }
  }

  return opt_route;
}




/*implement your heuristic in this function*/
void run_heuristic(){
  
  /*generate a random solution for the random heuristic*/
  int i,help, object, tot_assigned =0;
  int *r;
  double energy_temp = 0.0; 
  double capacity_temp = 0.0;
  int from, to, temp;
  int charging_station;
  
  r = new int[NUM_OF_CUSTOMERS+1];
  //set indexes of objects
  for(i = 1; i <= NUM_OF_CUSTOMERS; i++){
    r[i-1]=i;

  }
  //randomly change indexes of objects
  for(i = 0; i <= NUM_OF_CUSTOMERS; i++){
    object = (int) ((rand()/(RAND_MAX+1.0)) * (double)(NUM_OF_CUSTOMERS-tot_assigned));
    help = r[i];
    r[i]=r[i+object];
    r[i+object]=help;
    tot_assigned++;
  }

  best_sol->steps = 0;
  best_sol->tour_length = INT_MAX;
  
  best_sol->tour[0] = DEPOT;
  best_sol->steps++;

  i = 0;
  while(i < NUM_OF_CUSTOMERS) {
    from = best_sol->tour[best_sol->steps-1];
    to = r[i];
    if((capacity_temp + get_customer_demand(to)) <= MAX_CAPACITY && energy_temp+get_energy_consumption(from,to) <= BATTERY_CAPACITY){
      capacity_temp  += get_customer_demand(to);
      energy_temp += get_energy_consumption(from,to);
      best_sol->tour[best_sol->steps] = to;
      best_sol->steps++;
      i++;
    } else if ((capacity_temp + get_customer_demand(to)) > MAX_CAPACITY){
      capacity_temp = 0.0;
      energy_temp = 0.0;
      best_sol->tour[best_sol->steps] = DEPOT;
      best_sol->steps++;
    } else if (energy_temp+get_energy_consumption(from,to) > BATTERY_CAPACITY){
      charging_station = rand() % (ACTUAL_PROBLEM_SIZE-NUM_OF_CUSTOMERS-1)+NUM_OF_CUSTOMERS+1;
      if(is_charging_station(charging_station)==true){
	energy_temp = 0.0;
	best_sol->tour[best_sol->steps] =  charging_station;
	best_sol->steps++;
      }
    } else {
      capacity_temp = 0.0;
      energy_temp = 0.0;
      best_sol->tour[best_sol->steps] =  DEPOT;
      best_sol->steps++;
    }
  }
 
  //close EVRP tour to return back to the depot
  if(best_sol->tour[best_sol->steps-1]!=DEPOT){
    best_sol->tour[best_sol->steps] = DEPOT;
    best_sol->steps++;
  }

  best_sol->tour_length = fitness_evaluation(best_sol->tour, best_sol->steps);


  //free memory
  delete[] r;
}




/*free memory structures*/
void free_heuristic(){

  delete[] best_sol->tour;


}

