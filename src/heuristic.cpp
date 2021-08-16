#include<cmath>
#include <ctime>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<cstring>
#include<math.h>
#include<fstream>
#include <limits.h>
#include <algorithm>

#include "heuristic.hpp"
#include "EVRP.hpp"

using namespace std;


solution *best_sol; // see heuristic.hpp for the solution structure

struct Route {
  int size;
  int* t;
  Route* next;

  Route(int max_size = 0) {
    size = 0;
    if (max_size > 0) t = new int[max_size];
    next = nullptr;
  }
};
Route *T;

double iter_value;
Route *iter_route;

Route *buffer1;
Route *buffer2;
int *ibuffer;
double *dbuffer;

double** pheromone;
double pher_max;
double pher_min;
double rho = 0.98;
double pr = 0.05;
double alpha0 = 1;
double beta0 = 2;

int NUM_OF_NODES;

void free_route(Route *route) {
  if (route == nullptr) return;
  if (route->next != nullptr) free_route(route->next);
  delete[] route->t;
  delete route;
}

/*initialize the structure of your heuristic in this function*/
void initialize_heuristic(){
  NUM_OF_NODES = NUM_OF_CUSTOMERS + 1;
  
  best_sol = new solution;
  best_sol->tour = new int[NUM_OF_CUSTOMERS+1000];
  best_sol->id = 1;
  best_sol->steps = 0;
  best_sol->tour_length = 385;

  T = nullptr;

  iter_route = new Route(NUM_OF_CUSTOMERS + 1000);
  iter_value = INT_MAX;

  ibuffer = new int[NUM_OF_CUSTOMERS + 1000];
  dbuffer = new double[NUM_OF_CUSTOMERS + 1000];
  buffer1 = new Route(NUM_OF_CUSTOMERS + 1000);
  buffer2 = new Route(NUM_OF_CUSTOMERS + 1000);

  pheromone = generate_2D_matrix_double(NUM_OF_NODES, NUM_OF_NODES);
}

void update_pheromone_matrix() {
  pher_max = 1.0 / ((1.0 - rho) * best_sol->tour_length);
  double prn = pow(pr, 1.0 / NUM_OF_NODES);
  pher_min = pher_max * (1.0 - prn) / ((1.0 * NUM_OF_NODES / 2 - 1) * prn);
  
  for (int i = 0; i < NUM_OF_NODES; ++i) {
    for (int j = 0; j < NUM_OF_NODES; ++j) {
      pheromone[i][j] = min(pher_max, max(pheromone[i][j] * rho, pher_min));
    }
  }
  for (int i = 1, last = 0; i < iter_route->size; ++i) {
    if (iter_route->t[i] <= NUM_OF_CUSTOMERS) {
      pheromone[last][iter_route->t[i]] = min(pher_max, max(pheromone[last][iter_route->t[i]] + 1.0 / iter_value, pher_min));
      last = iter_route->t[i];
    }
  }
}

void reset() {
  best_sol->id = 1;
  best_sol->steps = 0;
  best_sol->tour_length = INT_MAX;

  pher_max = 1.0 / ((1.0 - rho) * best_sol->tour_length);
  double prn = pow(pr, 1.0 / NUM_OF_NODES);
  pher_min = pher_max * (1.0 - prn) / ((1.0 * NUM_OF_NODES / 2 - 1) * prn);
  
  for (int i = 0; i < NUM_OF_NODES; ++i) {
    for (int j = 0; j < NUM_OF_NODES; ++j) {
      pheromone[i][j] = pher_max;
    }
  }
}

int roulette_wheel_selection(Route *input, int i) {
  int result = 0;
  double max_value = 0;
  for (int j = 0; j < input->size; ++j) {
    double prob = pow(pheromone[i][input->t[j]], alpha0) / pow(get_distance(i, input->t[j]), beta0);
    if (max_value < prob) {
      max_value = prob;
      result = j;
    }
  }
  return result;
}

double two_opt_distance(Route *route) {
  double result = 0;
  for (int i = 1; i < route->size; ++i) {
    result += get_distance(route->t[i - 1], route->t[i]);
  }
  return result;
}

double two_opt_distance(Route *route, int s, int t) {
  double result = get_distance(route->t[s - 1], route->t[t]) + get_distance(route->t[s], route->t[t + 1]);
  int i;
  for (i = 1; i <= s; ++i) {
    result += get_distance(route->t[i - 1], route->t[i]);
  }
  for (i = s + 1; i <= t; ++i) {
    result += get_distance(route->t[i], route->t[i - 1]);
  }
  for (i = t + 1; i < route->size; ++i) {
    result += get_distance(route->t[i - 1], route->t[i]);
  }
  return result;
}

void two_opt(Route *route) {
  double best = two_opt_distance(route);
  bool optimized = true;
  int i, j;
  while (optimized) {
    optimized = false;
    int n = route->size - 2;
    for (i = 1; i < n; ++i) {
      if (optimized) break;
      for (j = i + 1; j < n + 1; ++j) {
	double value = two_opt_distance(route, i, j);
	if (value < best) {
	  optimized = true;
	  best = value;
	  reverse(route->t + i, route->t + j + 1);
	  break;
	}
      }
    }
  }
}

void route_construction() {
  int i, start = rand() % NUM_OF_NODES;

  Route *route = buffer1;
  Route *input = buffer2;
  route->size = input->size = 0;
  
  for (i = 0; i < NUM_OF_NODES; ++i) {
    if (i == start) continue;
    input->t[input->size++] = i;
  }
  route->t[route->size++] = start;
  while (input->size > 0) {
    i = roulette_wheel_selection(input, route->t[route->size - 1]);
    route->t[route->size++] = input->t[i];
    swap(input->t[i], input->t[input->size - 1]);
    input->size--;
  }
  for (i = 0; i < route->size; ++i) {
    if (route->t[i] == 0) break;
  }
  rotate(route->t, route->t + i, route->t + route->size);

  int *p = ibuffer;
  double *v = dbuffer;
  for (i = 0; i < NUM_OF_NODES; ++i) {
    v[i] = 2e9;
    p[i] = 0;
  }
  v[0] = 0;
  for (i = 1; i <= NUM_OF_CUSTOMERS; ++i) {
    int j = i, tc = 0, td = 0;
    while (j <= NUM_OF_CUSTOMERS && tc <= MAX_CAPACITY) {
      tc += get_customer_demand(route->t[j]);
      if (i == j) {
	td = get_distance(0, route->t[j]) + get_distance(route->t[j], 0);
      } else {
	td = td - get_distance(route->t[j - 1], 0) + get_distance(route->t[j - 1], route->t[j]) + get_distance(route->t[j], 0);
      }
      if (tc <= MAX_CAPACITY) {
	if (v[i - 1] + td < v[j]) {
	  v[j] = v[i - 1] + td;
	  p[j] = i - 1;
	}
	j += 1;
      }
    }
  }

  if (T != nullptr) free_route(T);
  T = new Route;
  Route *node = T;
  for (i = NUM_OF_CUSTOMERS; i > 0; ) {
    node->t = new int[(i - p[i] + 2) * 2 + 1];
    
    node->t[node->size++] = 0;
    for (int j = p[i] + 1; j <= i; ++j) {
      node->t[node->size++] = route->t[j];
    }
    node->t[node->size++] = 0;
    
    two_opt(node);

    if (p[i] < 1) break;
    node->next = new Route;
    node = node->next;
    i = p[i];
  }
}

void removal_heuristic(Route *route) {
  double D = BATTERY_CAPACITY / get_consumption_rate();
  double ld = D;

  Route *opt_route = buffer2;
  opt_route->size = 0;
  for (int i = 0, n = route->size - 1; i < n; ++i) {
    opt_route->t[opt_route->size++] = route->t[i];
    int s = 0;
    double ds = 2e9;
    for (int j = NUM_OF_CUSTOMERS + 1; j < ACTUAL_PROBLEM_SIZE; ++j) {
      double val = get_distance(route->t[i], j) + get_distance(j, route->t[i + 1]);
      if (ds > val && get_distance(route->t[i], j) <= ld) {
	s = j;
	ds = val;
      }
    }
    ld = D - get_distance(s, route->t[i + 1]);
    opt_route->t[opt_route->size++] = s;
  }
  opt_route->t[opt_route->size++] = 0;

  while (opt_route->size > route->size) {
    int s = 0;
    double ds = 0;
    ld = 0;

    int *next_station = ibuffer;
    double *total_distance = dbuffer;
    total_distance[0] = 0.0;
    for (int i = 1; i < opt_route->size; ++i) {
      total_distance[i] = total_distance[i - 1] + get_distance(opt_route->t[i - 1], opt_route->t[i]);
    }
    for (int i = opt_route->size - 1; i >= 0; --i) {
      next_station[i] = opt_route->t[i] == 0 || opt_route->t[i] > NUM_OF_CUSTOMERS ? i : next_station[i + 1];
    }

    for (int i = 1, n = opt_route->size - 1; i < n; ++i) {
      if (opt_route->t[i] <= NUM_OF_CUSTOMERS) {
	ld += get_distance(opt_route->t[i - 1], opt_route->t[i]);
	continue;
      }
      int station = opt_route->t[i];
      if (ld + get_distance(opt_route->t[i - 1], opt_route->t[i + 1]) + total_distance[next_station[i + 1]] - total_distance[i + 1] <= D) {
	double loss = get_distance(opt_route->t[i - 1], station) + get_distance(station, opt_route->t[i + 1]) - get_distance(opt_route->t[i - 1], opt_route->t[i + 1]);
	if (ds < loss) {
	  ds = loss;
	  s = i;
	}
      }
      ld = 0;
    }
    if (ds > 0) {
      for (int i = s + 1; i < opt_route->size; ++i) {
	opt_route->t[i - 1] = opt_route->t[i];
      }
      opt_route->size--;
    } else break;
  }
  route->size = opt_route->size;
  for (int i = 0; i < route->size; ++i) {
    route->t[i] = opt_route->t[i];
  }
}

void baco() {
  time_t start_time = time(NULL);
  for (int gen = 0; ; ++gen) {
    if (difftime(time(NULL), start_time) > 3) break;
    
    iter_value = INT_MAX;
    iter_route->size = 0;

    for (int ant = 0; ant < NUM_OF_NODES; ++ant) {
      route_construction();

      Route *current = buffer1;
      current->size = 0;
      current->t[current->size++] = 0;
      
      Route *node = T;
      // cout << "******************\n";
      while (node != nullptr) {
	// cout << '\t';
	// for (int i = 0; i < node->size; ++i) {
	  // if (node->t[i] > NUM_OF_CUSTOMERS) cout << '<';
	  // cout << node->t[i] << (node->t[i] > NUM_OF_CUSTOMERS ? "> " : " ");
	// }
	// cout << '\n';
	removal_heuristic(node);
	// cout << "--->\t";
	for (int i = 0; i < node->size; ++i) {
	  // if (node->t[i] > NUM_OF_CUSTOMERS) cout << '<';
	  // cout << node->t[i] << (node->t[i] > NUM_OF_CUSTOMERS ? "> " : " ");
	  if (i > 0) current->t[current->size++] = node->t[i];
	}
	// cout << '\n';
	node = node->next;
      }

      double value = fitness_evaluation(current->t, current->size);
      // cout << value << '\n';
      if (iter_value > value) {
	iter_value = value;
	iter_route->size = current->size;
	copy(current->t, current->t + current->size, iter_route->t);
      }
    }
    if (best_sol->tour_length > iter_value) {
      best_sol->tour_length = iter_value;
      best_sol->steps = iter_route->size;
      copy(iter_route->t, iter_route->t + iter_route->size, best_sol->tour);
    }
    update_pheromone_matrix();
    // cout << "\tGen " << gen << ": " << best_sol->tour_length << '\n';
  }
}

/*implement your heuristic in this function*/
void run_heuristic() {}

/*free memory structures*/
void free_heuristic(){
  delete[] best_sol->tour;

  delete[] ibuffer;
  delete[] dbuffer;

  free_route(iter_route);
  free_route(buffer1);
  free_route(buffer2);

  for (int i = 0; i < NUM_OF_NODES; ++i) {
    delete[] pheromone[i];
  }
  delete[] pheromone;
}

