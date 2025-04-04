#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <algorithm>

using namespace std;

struct Customer {
    int id;
    int demand;
    int service_time;
    int earliest;
    int latest;
};

struct Route {
    vector<int> customers;  // includes depot at start and end
    int total_demand = 0;
    int total_distance = 0;
    int duration = 0;
};

using Solution = vector<Route>;

const int DEPOT = 0;
const int Q = 15; // Vehicle capacity
const int D = 200; // Max route duration
int M = 3; // Number of vehicles

vector<Customer> customers;
vector<vector<int>> dist;
vector<vector<int>> travel_time;

double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

Solution generate_initial_solution() {
    Solution sol(M);
    vector<bool> visited(customers.size(), false);
    visited[0] = true;

    int vehicle = 0;
    for (int i = 1; i < customers.size(); ++i) {
        if (vehicle >= M) break;
        if (sol[vehicle].total_demand + customers[i].demand <= Q) {
            sol[vehicle].customers.push_back(DEPOT);
            sol[vehicle].customers.push_back(i);
            sol[vehicle].customers.push_back(DEPOT);
            sol[vehicle].total_demand += customers[i].demand;
            visited[i] = true;
            vehicle++;
        }
    }
    return sol;
}

int calculate_cost(const Solution& sol) {
    int active_routes = 0;
    int total_distance = 0;

    for (const auto& r : sol) {
        if (r.customers.size() > 2) { // active (non-empty) route
            active_routes++;
            for (size_t i = 1; i < r.customers.size(); ++i) {
                int from = r.customers[i-1];
                int to = r.customers[i];
                total_distance += dist[from][to];
            }
        }
    }

    // Encode cost as: active_routes * large_penalty + distance
    // So fewer vehicles always preferred
    return active_routes * 100000 + total_distance;
}


bool is_feasible(const Route& route) {
    int demand = 0;
    int time = 0;

    for (size_t i = 1; i < route.customers.size(); ++i) {
        int from = route.customers[i-1];
        int to = route.customers[i];
        time += travel_time[from][to];
        time = max(time, customers[to].earliest);
        if (time > customers[to].latest) return false;
        time += customers[to].service_time;
        demand += customers[to].demand;
    }

    return demand <= Q && time <= D;
}

Solution generate_neighbor(const Solution& sol) {
    Solution neighbor = sol;
    int v1 = rand() % M;
    int v2 = rand() % M;

    if (neighbor[v1].customers.size() <= 2 || neighbor[v2].customers.size() <= 2) return neighbor;

    int i = rand() % (neighbor[v1].customers.size() - 2) + 1;
    int j = rand() % (neighbor[v2].customers.size() - 2) + 1;

    swap(neighbor[v1].customers[i], neighbor[v2].customers[j]);

    if (!is_feasible(neighbor[v1]) || !is_feasible(neighbor[v2])) {
        return sol;
    }

    return neighbor;
}

Solution simulated_annealing(Solution initial, double T, double alpha, int max_iter) {
    Solution current = initial;
    Solution best = current;

    for (int iter = 0; iter < max_iter; ++iter) {
        Solution neighbor = generate_neighbor(current);
        int delta = calculate_cost(neighbor) - calculate_cost(current);

        if (delta < 0 || rand01() < exp(-delta / T)) {
            current = neighbor;
            if (calculate_cost(current) < calculate_cost(best)) {
                best = current;
            }
        }

        T *= alpha;
        if (T < 1e-3) break;
    }

    return best;
}

void print_solution(const Solution& sol) {
    int active_routes = 0;
    int total_distance = 0;

    for (size_t i = 0; i < sol.size(); ++i) {
        if (sol[i].customers.size() > 2) {
            active_routes++;
            cout << "Route " << i + 1 << ": ";
            for (int c : sol[i].customers) {
                cout << c << " ";
            }
            cout << "\n";

            for (size_t j = 1; j < sol[i].customers.size(); ++j) {
                total_distance += dist[sol[i].customers[j-1]][sol[i].customers[j]];
            }
        }
    }
    cout << "Number of vehicles used: " << active_routes << "\n";
    cout << "Total distance: " << total_distance << "\n";
}


int main() {
    srand(time(nullptr));

    // Sample input (5 customers + depot)
    customers = {
        {0, 0, 0, 0, 999},     // depot
        {1, 4, 10, 10, 50},
        {2, 6, 10, 20, 70},
        {3, 5, 10, 30, 90},
        {4, 3, 10, 40, 100},
        {5, 2, 10, 50, 110}
    };

    dist = {
        {0, 10, 20, 30, 40, 50},
        {10, 0, 15, 25, 35, 45},
        {20, 15, 0, 14, 24, 34},
        {30, 25, 14, 0, 10, 20},
        {40, 35, 24, 10, 0, 10},
        {50, 45, 34, 20, 10, 0}
    };

    travel_time = dist; // same for simplicity

    Solution initial = generate_initial_solution();
    Solution best = simulated_annealing(initial, 1000.0, 0.995, 10000);

    print_solution(best);
    return 0;
}
