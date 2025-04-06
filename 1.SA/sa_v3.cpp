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

// Global parameters (set from file)
int DEPOT = 0;
double Q = 0.0;
double D = numeric_limits<double>::max();
int M = 0;

struct Customer {
    int id;
    int x, y;
    int demand;
    int earliest;
    int latest;
    int service_time;
};

struct Route {
    vector<int> customers;
    int total_demand = 0;
};

using Solution = vector<Route>;

vector<Customer> customers;
vector<vector<double>> dist;
vector<vector<double>> travel_time;

bool is_feasible(const Route& route);

bool can_insert_customer(const Route& route, int customer_id, int insert_pos) {
    Route temp = route;
    temp.customers.insert(temp.customers.begin() + insert_pos, customer_id);
    return is_feasible(temp);
}

bool is_solution_feasible(const Solution& sol) {
    for (const auto& route : sol) {
        if (route.customers.size() > 2 && !is_feasible(route)) {
            return false;
        }
    }
    return true;
}

bool all_customers_served_once(const Solution& sol, int customer_count) {
    vector<int> visit_count(customer_count, 0);
    for (const auto& route : sol) {
        for (int cust : route.customers) {
            visit_count[cust]++;
        }
    }
    for (int i = 1; i < customer_count; ++i) {
        if (visit_count[i] != 1)
            return false;
    }
    return true;
}

bool check_demands(const Solution& sol) {
    for (const auto& route : sol) {
        if (route.total_demand > Q)
            return false;
    }
    return true;
}

bool check_time_windows(const Solution& sol) {
    for (const auto& route : sol) {
        double time = 0.0;
        for (size_t i = 0; i < route.customers.size() - 1; ++i) {
            const Customer& current = customers[route.customers[i]];
            const Customer& next = customers[route.customers[i + 1]];
            time = max(time, static_cast<double>(current.earliest));
            time += current.service_time;
            time += dist[current.id][next.id];
            if (time > next.latest) {
                cout << "⚠️ Customer " << next.id
                     << " violated time window. Arrival: " << time
                     << ", Latest: " << next.latest << endl;
                return false;
            }
        }
    }
    return true;
}

double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

double euclidean_distance(const Customer& a, const Customer& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

bool is_feasible(const Route& route) {
    double time = 0.0;
    int load = 0;
    if (route.customers.front() != DEPOT || route.customers.back() != DEPOT)
        return false;
    for (size_t i = 1; i < route.customers.size(); ++i) {
        int from = route.customers[i - 1];
        int to = route.customers[i];
        time += travel_time[from][to];
        time = max(time, static_cast<double>(customers[to].earliest));
        if (time > customers[to].latest)
            return false;
        time += customers[to].service_time;
        load += customers[to].demand;
        if (load > Q)
            return false;
    }
    if (time > customers[DEPOT].latest)
        return false;
    return true;
}

Solution generate_initial_solution() {
    Solution sol;
    for (int i = 0; i < M; ++i) {
        sol.push_back({{DEPOT, DEPOT}, 0});
    }

    vector<bool> assigned(customers.size(), false);
    assigned[DEPOT] = true;

    for (int i = 1; i < customers.size(); ++i) {
        double best_incr = numeric_limits<double>::max();
        int best_route = -1;
        int best_pos = -1;
        for (int r = 0; r < M; ++r) {
            if (sol[r].total_demand + customers[i].demand > Q)
                continue;
            for (size_t pos = 1; pos < sol[r].customers.size(); ++pos) {
                int prev = sol[r].customers[pos - 1];
                int next = sol[r].customers[pos];
                double cost_removed = dist[prev][next];
                double cost_added = dist[prev][i] + dist[i][next];
                double incr = cost_added - cost_removed;
                if (incr < best_incr && can_insert_customer(sol[r], i, pos)) {
                    best_incr = incr;
                    best_route = r;
                    best_pos = pos;
                }
            }
        }
        if (best_route != -1) {
            sol[best_route].customers.insert(sol[best_route].customers.begin() + best_pos, i);
            sol[best_route].total_demand += customers[i].demand;
            assigned[i] = true;
        }
    }

    for (int i = 1; i < customers.size(); ++i) {
        if (!assigned[i]) {
            for (int r = 0; r < M; ++r) {
                int pos = sol[r].customers.size() - 1;
                if (customers[i].demand <= Q &&
                    can_insert_customer(sol[r], i, pos)) {
                    sol[r].customers.insert(sol[r].customers.begin() + pos, i);
                    sol[r].total_demand += customers[i].demand;
                    assigned[i] = true;
                    break;
                }
            }
        }
    }
    return sol;
}

double calculate_cost(const Solution& sol) {
    int active_routes = 0;
    double total_distance = 0.0;
    for (const auto& r : sol) {
        if (r.customers.size() > 2) {
            active_routes++;
            for (size_t i = 1; i < r.customers.size(); ++i) {
                total_distance += dist[r.customers[i - 1]][r.customers[i]];
            }
        }
    }
    return active_routes * 100000.0 + total_distance;
}

Solution generate_neighbor(const Solution& sol) {
    Solution neighbor = sol;
    int v1 = rand() % M;
    int v2 = rand() % M;

    if (neighbor[v1].customers.size() <= 2)
        return neighbor;

    int pos = rand() % (neighbor[v1].customers.size() - 2) + 1;
    int cust = neighbor[v1].customers[pos];

    neighbor[v1].customers.erase(neighbor[v1].customers.begin() + pos);
    neighbor[v1].total_demand -= customers[cust].demand;

    int insert_pos = rand() % (neighbor[v2].customers.size() - 1) + 1;

    if (neighbor[v2].total_demand + customers[cust].demand <= Q &&
        can_insert_customer(neighbor[v2], cust, insert_pos)) {
        neighbor[v2].customers.insert(neighbor[v2].customers.begin() + insert_pos, cust);
        neighbor[v2].total_demand += customers[cust].demand;
        return neighbor;
    }

    return sol;
}

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
                total_distance += dist[sol[i].customers[j - 1]][sol[i].customers[j]];
            }
        }
    }
    total_distance = round(total_distance * 100) / 100.0;
    cout << "Number of vehicles used: " << active_routes << "\n";
    cout << "Total distance: " << fixed << setprecision(2) << total_distance << "\n";
}

void read_data(const string& filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file " << filename << "\n";
        exit(1);
    }

    string line;
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            infile >> M >> Q;
        }
        if (line.find("CUSTOMER") != string::npos)
            break;
    }
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
            travel_time[i][j] = dist[i][j];
        }
}

int main() {
    srand(time(nullptr));

    read_data("25-rce-31.txt");

    Solution initial = generate_initial_solution();
    Solution best = simulated_annealing(initial, 1000.0, 0.995, 5000);

    print_solution(best);

    bool feasible = is_solution_feasible(best);
    bool all_served = all_customers_served_once(best, customers.size());
    bool demand_ok = check_demands(best);
    bool time_window_ok = check_time_windows(best);

    if (feasible)
        cout << "✅ Feasibility check passed.\n";
    else
        cout << "❌ Route feasibility check failed.\n";

    if (all_served)
        cout << "✅ All customers visited exactly once.\n";
    else
        cout << "❌ Some customers were missed or visited more than once.\n";

    if (demand_ok)
        cout << "✅ Route demands within vehicle capacity.\n";
    else
        cout << "❌ One or more routes exceed vehicle capacity!\n";

    if (time_window_ok)
        cout << "✅ Time window constraints satisfied.\n";
    else
        cout << "❌ Time window violation detected!\n";

    if (feasible && all_served && demand_ok && time_window_ok)
        cout << "🎉 Final solution is VALID.\n";
    else
        cout << "🚨 Final solution is INVALID.\n";

    return 0;
}
