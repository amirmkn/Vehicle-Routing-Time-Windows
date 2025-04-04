#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <limits>
#include <climits>
#include <cstdlib>
#include <ctime>
#include <algorithm>

using namespace std;

struct Customer {
    int id;
    int x, y;
    int demand;
    int earliest;
    int latest;
    int service_time;
};

struct Route {
    vector<int> customers;  // includes depot at start and end
    int total_demand = 0;
    int total_distance = 0;
    int duration = 0;
};

using Solution = vector<Route>;

int DEPOT = 0;
int Q = 0; // Vehicle capacity (to be set from file)
int D = 2000; // Max route duration (can be tweaked)
int M = 0; // Number of vehicles

vector<Customer> customers;
vector<vector<int>> dist;
vector<vector<int>> travel_time;

double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

int euclidean_distance(const Customer& a, const Customer& b) {
    return static_cast<int>(round(sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))));
}

// Greedy initial solution: Try inserting customers into the best available vehicle
Solution generate_initial_solution() {
    Solution sol;
    
    for (int i = 0; i < M; ++i) {
        sol.push_back({{DEPOT, DEPOT}, 0, 0, 0});
    }

    vector<bool> assigned(customers.size(), false);
    assigned[0] = true; // Depot

    for (int i = 1; i < customers.size(); ++i) {
        int best_vehicle = -1;
        int best_cost = INT_MAX;

        for (int v = 0; v < M; ++v) {
            if (sol[v].total_demand + customers[i].demand > Q) continue;

            // Try inserting between DEPOT and last customer
            int last_cust = sol[v].customers[sol[v].customers.size() - 2];
            int cost = dist[last_cust][i] + dist[i][DEPOT] - dist[last_cust][DEPOT];

            if (cost < best_cost) {
                best_cost = cost;
                best_vehicle = v;
            }
        }

        if (best_vehicle != -1) {
            sol[best_vehicle].customers.insert(sol[best_vehicle].customers.end() - 1, i);
            sol[best_vehicle].total_demand += customers[i].demand;
        }
    }

    return sol;
}

// Cost function: minimize vehicle count first, then minimize distance
int calculate_cost(const Solution& sol) {
    int active_routes = 0;
    int total_distance = 0;

    for (const auto& r : sol) {
        if (r.customers.size() > 2) {
            active_routes++;
            for (size_t i = 1; i < r.customers.size(); ++i) {
                total_distance += dist[r.customers[i-1]][r.customers[i]];
            }
        }
    }

    return active_routes * 100000 + total_distance;
}

// Check if the route is feasible: demand and time windows
bool is_feasible(const Route& route) {
    int demand = 0;
    int time = 0;

    for (size_t i = 1; i < route.customers.size(); ++i) {
        int from = route.customers[i - 1];
        int to = route.customers[i];
        time += travel_time[from][to];
        time = max(time, customers[to].earliest);
        if (time > customers[to].latest) return false;
        time += customers[to].service_time;
        demand += customers[to].demand;
    }

    return demand <= Q && time <= D;
}

// Move a random customer to another route
Solution generate_neighbor(const Solution& sol) {
    Solution neighbor = sol;
    int v1 = rand() % M;
    int v2 = rand() % M;

    if (neighbor[v1].customers.size() <= 2 || neighbor[v2].customers.size() <= 2) return neighbor;

    int i = rand() % (neighbor[v1].customers.size() - 2) + 1;
    int customer = neighbor[v1].customers[i];

    neighbor[v1].customers.erase(neighbor[v1].customers.begin() + i);

    int insert_pos = rand() % (neighbor[v2].customers.size() - 1) + 1;
    neighbor[v2].customers.insert(neighbor[v2].customers.begin() + insert_pos, customer);

    if (!is_feasible(neighbor[v1]) || !is_feasible(neighbor[v2])) {
        return sol;
    }

    return neighbor;
}

// Simulated Annealing Optimization
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
                total_distance += dist[sol[i].customers[j - 1]][sol[i].customers[j]];
            }
        }
    }
    cout << "Number of vehicles used: " << active_routes << "\n";
    cout << "Total distance: " << total_distance << "\n";
}

// Read input file
void read_data(const string& filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file.\n";
        exit(1);
    }

    string line;
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            infile >> M >> Q;
        }
        if (line.find("CUSTOMER") != string::npos) break;
    }

    getline(infile, line);
    getline(infile, line);

    customers.clear();
    while (getline(infile, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        Customer c;
        iss >> c.id >> c.x >> c.y >> c.demand >> c.earliest >> c.latest >> c.service_time;
        customers.push_back(c);
    }

    int n = customers.size();
    dist = vector<vector<int>>(n, vector<int>(n));
    travel_time = vector<vector<int>>(n, vector<int>(n));

    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist[i][j] = travel_time[i][j] = euclidean_distance(customers[i], customers[j]);
}

int main() {
    srand(time(nullptr));

    read_data("25-rce-31.txt");

    Solution initial = generate_initial_solution();
    Solution best = simulated_annealing(initial, 1000.0, 0.995, 5000);

    print_solution(best);
    return 0;
}
