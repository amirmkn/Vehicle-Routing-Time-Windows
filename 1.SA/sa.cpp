#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <climits>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <iomanip>

using namespace std;

struct Customer {
    int id;
    int x, y;
    int demand;
    int earliest;    // e_i
    int latest;      // l_i
    int service_time; // τ_i
};

struct Route {
    vector<int> customers;  // route: depot ... customer ... depot
    int total_demand = 0;
};

using Solution = vector<Route>;

// Global parameters (set from file)
int DEPOT = 0;
double Q = 0.0; // Vehicle capacity
double D = numeric_limits<double>::max(); // maximum route duration (if used)
int M = 0; // Number of vehicles available

vector<Customer> customers;
vector<vector<double>> dist;         // distance matrix (double precision)
vector<vector<double>> travel_time;    // travel time matrix (we assume same as distance)

double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

// Euclidean distance (double precision)
double euclidean_distance(const Customer& a, const Customer& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

// Greedy initial solution (inserts customers into routes based on best insertion cost)
Solution generate_initial_solution() {
    // Create M empty routes, each starting and ending at depot.
    Solution sol;
    for (int i = 0; i < M; ++i) {
        // Each route starts and ends with the depot.
        sol.push_back({{DEPOT, DEPOT}, 0});
    }
    
    // Mark depot as assigned.
    vector<bool> assigned(customers.size(), false);
    assigned[DEPOT] = true;
    
    // For every customer (other than depot), try to insert into a route.
    // We use a simple best insertion heuristic.
    for (int i = 1; i < customers.size(); ++i) {
        double best_incr = numeric_limits<double>::max();
        int best_route = -1;
        int best_pos = -1;
        // Try to insert customer i into any route between two consecutive nodes.
        for (int r = 0; r < M; ++r) {
            // Only consider this route if adding the customer doesn't violate capacity.
            if (sol[r].total_demand + customers[i].demand > Q)
                continue;
            // Try all insertion positions (between two nodes)
            for (size_t pos = 1; pos < sol[r].customers.size(); ++pos) {
                int prev = sol[r].customers[pos - 1];
                int next = sol[r].customers[pos];
                double cost_removed = dist[prev][next];
                double cost_added = dist[prev][i] + dist[i][next];
                double incr = cost_added - cost_removed;
                if (incr < best_incr) {
                    best_incr = incr;
                    best_route = r;
                    best_pos = pos;
                }
            }
        }
        // If we found a route, insert customer i there.
        if (best_route != -1) {
            sol[best_route].customers.insert(sol[best_route].customers.begin() + best_pos, i);
            sol[best_route].total_demand += customers[i].demand;
            assigned[i] = true;
        }
    }
    
    // If any customer remains unassigned, try to create a new route (if vehicles available)
    // (Here, we assume that the provided instance is feasible with M vehicles.)
    for (int i = 1; i < customers.size(); ++i) {
        if (!assigned[i]) {
            for (int r = 0; r < M; ++r) {
                // Check if depot->customer->depot is feasible.
                if (customers[i].demand <= Q) {
                    sol[r].customers.insert(sol[r].customers.begin() + sol[r].customers.size() - 1, i);
                    sol[r].total_demand += customers[i].demand;
                    assigned[i] = true;
                    break;
                }
            }
        }
    }
    return sol;
}

// Compute the route cost (hierarchical: primary = number of active routes, secondary = total distance).
// Total distance is rounded to two decimals.
double calculate_cost(const Solution& sol) {
    int active_routes = 0;
    double total_distance = 0.0;
    for (const auto& r : sol) {
        if (r.customers.size() > 2) { // active route: depot -> ... -> depot
            active_routes++;
            for (size_t i = 1; i < r.customers.size(); ++i) {
                total_distance += dist[r.customers[i - 1]][r.customers[i]];
            }
        }
    }
    // Hierarchical objective: heavily penalize extra routes.
    return active_routes * 100000.0 + total_distance;
}

// Check feasibility of a route:
// - The route must start and end at depot.
// - The cumulative demand <= Q.
// - Time windows are met. We compute arrival and departure times.
//   * d₀ = 0 (departure from depot)
//   * aᵢ = max( dᵢ₋₁ + travel_time(prev, current), eᵢ )
//   * dᵢ = aᵢ + service_time(current)
//   * The arrival time back at depot (after the last customer) must be <= depot latest.
bool is_feasible(const Route& route) {
    double time = 0.0; // departure from depot is 0
    int load = 0;
    
    // The route must start with depot and end with depot.
    if (route.customers.front() != DEPOT || route.customers.back() != DEPOT)
        return false;
    
    // Traverse the route sequentially.
    for (size_t i = 1; i < route.customers.size(); ++i) {
        int from = route.customers[i - 1];
        int to = route.customers[i];
        // Arrival time to customer 'to'
        time = time + travel_time[from][to];
        // Wait until the earliest time if arrived early.
        time = max(time, static_cast<double>(customers[to].earliest));
        // If arrival is after the latest allowable time, infeasible.
        if (time > customers[to].latest)
            return false;
        // Add service time (departure time)
        time += customers[to].service_time;
        load += customers[to].demand;
        if (load > Q)
            return false;
    }
    // Ensure that the final arrival at the depot is within the depot's time window.
    if (time > customers[DEPOT].latest)
        return false;
    
    return true;
}

// Generate a neighbor solution by reassigning a customer from one route to another.
// Here we remove a random customer (not depot) from one route and insert it in another route at a random position.
Solution generate_neighbor(const Solution& sol) {
    Solution neighbor = sol;
    int v1 = rand() % M;
    int v2 = rand() % M;
    // Ensure route v1 has at least one customer (other than depot start/end)
    if (neighbor[v1].customers.size() <= 2)
        return neighbor;
    // Select a random customer index in route v1 (excluding first and last, which are depot).
    int pos = rand() % (neighbor[v1].customers.size() - 2) + 1;
    int cust = neighbor[v1].customers[pos];
    // Remove the customer from route v1.
    neighbor[v1].customers.erase(neighbor[v1].customers.begin() + pos);
    neighbor[v1].total_demand -= customers[cust].demand;
    
    // Try to insert into route v2 at a random valid position (between depots).
    int insert_pos = rand() % (neighbor[v2].customers.size() - 1) + 1;
    neighbor[v2].customers.insert(neighbor[v2].customers.begin() + insert_pos, cust);
    neighbor[v2].total_demand += customers[cust].demand;
    
    // Check feasibility; if not feasible, return original solution.
    if (!is_feasible(neighbor[v1]) || !is_feasible(neighbor[v2])) {
        return sol;
    }
    
    return neighbor;
}

// Simulated Annealing optimization
Solution simulated_annealing(Solution initial, double T, double alpha, int max_iter) {
    Solution current = initial;
    Solution best = current;
    double current_cost = calculate_cost(current);
    double best_cost = current_cost;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        Solution neighbor = generate_neighbor(current);
        double neighbor_cost = calculate_cost(neighbor);
        double delta = neighbor_cost - current_cost;
        
        if (delta < 0 || rand01() < exp(-delta / T)) {
            current = neighbor;
            current_cost = neighbor_cost;
            if (current_cost < best_cost) {
                best = current;
                best_cost = current_cost;
            }
        }
        
        T *= alpha;
        if (T < 1e-3)
            break;
    }
    
    return best;
}

void print_solution(const Solution& sol) {
    int active_routes = 0;
    double total_distance = 0.0;
    
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
    // Round total_distance to 2 decimals.
    total_distance = round(total_distance * 100) / 100.0;
    cout << "Number of vehicles used: " << active_routes << "\n";
    cout << "Total distance: " << fixed << setprecision(2) << total_distance << "\n";
}

// Read instance from file
// Expects a file formatted like your sample instance.
void read_data(const string& filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file " << filename << "\n";
        exit(1);
    }
    
    string line;
    // Read vehicle information
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line); // skip header
            infile >> M >> Q;
        }
        if (line.find("CUSTOMER") != string::npos)
            break;
    }
    // Skip header lines for customer data
    getline(infile, line);
    getline(infile, line);
    
    customers.clear();
    while (getline(infile, line)) {
        if (line.empty())
            continue;
        istringstream iss(line);
        Customer c;
        iss >> c.id >> c.x >> c.y >> c.demand >> c.earliest >> c.latest >> c.service_time;
        customers.push_back(c);
    }
    
    int n = customers.size();
    dist = vector<vector<double>>(n, vector<double>(n, 0.0));
    travel_time = vector<vector<double>>(n, vector<double>(n, 0.0));
    
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            dist[i][j] = euclidean_distance(customers[i], customers[j]);
            travel_time[i][j] = dist[i][j]; // Assume travel time equals distance.
        }
}

int main() {
    srand(time(nullptr));
    
    // Read instance file (adjust filename as needed)
    read_data("200-rce-42.txt");
    
    // Generate initial solution and optimize using Simulated Annealing.
    Solution initial = generate_initial_solution();
    Solution best = simulated_annealing(initial, 1000.0, 0.995, 5000);
    
    print_solution(best);
    return 0;
}
