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
#include <chrono>
#include <random>
#include <numeric>
#include "validation.h"

using namespace std;
using namespace std::chrono;

// Global parameters (set from file)
int DEPOT = 0; //Depot ID
double Q = 0.0; //Maximum capacity of each vehicle
int M = 0; // Number of vehicles


using Solution = vector<Route>;

vector<Customer> customers; // List of customers
vector<vector<double>> dist; // Distance matrix
vector<vector<double>> travel_time;// Travel time matrix

int evaluation_count = 0;
int max_evaluations = 0;
double max_exec_seconds = 0.0;
time_point<steady_clock> start_time;// Start time for execution

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

bool can_insert_customer(const Route& route, int customer_id, int insert_pos) {
    Route temp = route;
    temp.customers.insert(temp.customers.begin() + insert_pos, customer_id);
    return is_feasible(temp);
}

bool is_solution_feasible(const Solution& sol) {
    for (const auto& route : sol) {
        if (route.customers.size() > 2 && !is_feasible(route))
            return false;
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


// Function to generate the initial solution using a greedy approach
Solution generate_greedy_solution(){
    Solution sol;
    for (int i = 0; i < M; ++i) {
        sol.push_back({{DEPOT, DEPOT}, 0});
    }

    vector<bool> assigned(customers.size(), false);
    assigned[DEPOT] = true;

    // Insertion heuristic
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

    // Insert any unassigned customers at the end
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

// Function to generate the initial solution using a random approach
Solution generate_random_solution() {
    Solution sol;
    for (int i = 0; i < M; ++i) {
        sol.push_back({{DEPOT, DEPOT}, 0});
    }

    vector<bool> assigned(customers.size(), false);
    assigned[DEPOT] = true;

    // Create a shuffled list of customer IDs (excluding depot)
    vector<int> cust_ids;
    for (int i = 1; i < customers.size(); ++i) {
        cust_ids.push_back(i);
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(cust_ids.begin(), cust_ids.end(), g);

    for (int cust : cust_ids) {
        // Try to assign the customer to a feasible route
        vector<int> route_order(M);
        std::iota(route_order.begin(), route_order.end(), 0);  // Fill with 0, 1, ..., M-1
        std::shuffle(route_order.begin(), route_order.end(), g);

        bool inserted = false;
        for (int r : route_order) {
            for (size_t pos = 1; pos < sol[r].customers.size(); ++pos) {
                if (sol[r].total_demand + customers[cust].demand <= Q &&
                    can_insert_customer(sol[r], cust, pos)) {
                    sol[r].customers.insert(sol[r].customers.begin() + pos, cust);
                    sol[r].total_demand += customers[cust].demand;
                    inserted = true;
                    break;
                }
            }
            if (inserted) break;
        }
        assigned[cust] = inserted;
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

bool time_or_eval_limit_reached() {
    if (max_evaluations > 0 && evaluation_count >= max_evaluations)
        return true;
    if (max_exec_seconds > 0) {
        auto elapsed = duration_cast<seconds>(steady_clock::now() - start_time).count();
        if (elapsed >= max_exec_seconds)
            return true;
    }
    return false;
}

// Enhanced neighbor generation: randomly chooses either a removal–insertion or a swap move.
Solution generate_neighbor(const Solution& sol) {
    Solution neighbor = sol;
    int moveType = rand() % 2; // 0: removal-insertion, 1: swap

    if (moveType == 0) {
        // Removal–insertion move
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
    } else {
        // Swap move between two routes
        int r1 = rand() % M;
        int r2 = rand() % M;
        if (r1 == r2 ||
            neighbor[r1].customers.size() <= 2 ||
            neighbor[r2].customers.size() <= 2)
            return neighbor;

        int pos1 = rand() % (neighbor[r1].customers.size() - 2) + 1;
        int pos2 = rand() % (neighbor[r2].customers.size() - 2) + 1;
        int cust1 = neighbor[r1].customers[pos1];
        int cust2 = neighbor[r2].customers[pos2];

        // Check capacity feasibility for swap
        if (neighbor[r1].total_demand - customers[cust1].demand + customers[cust2].demand <= Q &&
            neighbor[r2].total_demand - customers[cust2].demand + customers[cust1].demand <= Q) {

            // Tentatively swap customers
            swap(neighbor[r1].customers[pos1], neighbor[r2].customers[pos2]);
            if (is_feasible(neighbor[r1]) && is_feasible(neighbor[r2])) {
                neighbor[r1].total_demand = neighbor[r1].total_demand - customers[cust1].demand + customers[cust2].demand;
                neighbor[r2].total_demand = neighbor[r2].total_demand - customers[cust2].demand + customers[cust1].demand;
                return neighbor;
            }
            // Otherwise, swap back if infeasible
            swap(neighbor[r1].customers[pos1], neighbor[r2].customers[pos2]);
        }
    }
    return sol;
}

Solution simulated_annealing(Solution initial, double T, double alpha, int max_iter) {
    Solution current = initial;
    Solution best = current;
    double current_cost = calculate_cost(current);
    double best_cost = current_cost;
    int stagnation_counter = 0;
    int max_stagnation = 200;
    int reheats = 0;
    double init_temp = T;

    std::ofstream logFile("log.txt");

    for (int iter = 0; !time_or_eval_limit_reached(); ++iter) {
        evaluation_count++;

        Solution neighbor = generate_neighbor(current);
        double neighbor_cost = calculate_cost(neighbor);
        double delta = neighbor_cost - current_cost;

        // Acceptance criterion
        if (delta < 0 || rand01() < exp(-delta / T)) {
            current = neighbor;
            current_cost = neighbor_cost;
            // Reset stagnation counter if an improvement in the best solution is found
            if (current_cost < best_cost) {
                best = current;
                best_cost = current_cost;
                stagnation_counter = 0;
            } else {
                stagnation_counter++;
            }
        } else {
            stagnation_counter++;
        }

        // Logging iteration info
        logFile << "Iter: " << iter
                << ", Temp: " << T
                << ", Best: " << best_cost
                << ", Eval: " << evaluation_count
                << std::endl;

        // Update temperature
        T *= alpha ;

        // Reheat if stagnation is detected
        if (stagnation_counter >= max_stagnation) {
            T = init_temp*2; // Reset to initial temperature
            stagnation_counter = 0;
            reheats++;
            logFile << ">>> Reheating! Temp reset to: " << T << std::endl;
        }
        // Check for low temperature and reheat if necessary
        if (T < 1e-3) {
            T = init_temp;
            logFile << ">>> Reheating due to low temp. Temp reset to: " << T << std::endl;
        }
    }
    logFile << "Total reheats: " << reheats << std::endl;
    logFile.close();

    auto now = steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;

    cout << "🔁 Evaluations done: " << evaluation_count << endl;
    cout << "⏱️  Time elapsed: " << elapsed.count() << " seconds" << endl;
    cout << "🔥 Final temperature: " << T << endl;

    return best;
}


// This function writes the final solution and checks to an output file.
void write_solution_to_file(const Solution& sol, const string& instance_name) {
    string out_filename = instance_name + "_output.txt";
    ofstream outFile(out_filename);

    if (!outFile.is_open()) {
        cerr << "Error opening output file: " << out_filename << endl;
        return;
    }

    int active_routes = 0;
    double total_distance = 0.0;
    outFile << fixed << setprecision(2);
    
    int route_number = 1;  // Start numbering from 1

    for (size_t i = 0; i < sol.size(); ++i) {
        if (sol[i].customers.size() > 2) {
            active_routes++;
            outFile << "Route " << route_number << ": ";
            route_number++;
            // Skip the first and last element (depot = 0)
            for (size_t j = 1; j + 1 < sol[i].customers.size(); ++j) {
                outFile << sol[i].customers[j];
                if (j + 2 < sol[i].customers.size()) outFile << " ";
            }
            outFile << "\n";

            for (size_t j = 1; j < sol[i].customers.size(); ++j) {
                total_distance += dist[sol[i].customers[j - 1]][sol[i].customers[j]];
            }
            outFile << "\n";
        }
    }

    total_distance = round(total_distance * 100) / 100.0;
    outFile << "Vehicles: " << active_routes << "\n\n";
    outFile << "Distance: " << total_distance << "\n\n";

    cout << "Final result saved to " << out_filename << endl;
}

// Utility function to extract instance name from a file path.
// It strips directory and extension. For example, "data/C101.txt" becomes "C101".
string extract_instance_name(const string& file_path) {
    // Find the last '/' or '\\'
    size_t pos = file_path.find_last_of("/\\");
    string filename = (pos == string::npos) ? file_path : file_path.substr(pos + 1);
    // Remove extension if present
    pos = filename.find_last_of(".");
    if (pos != string::npos)
        filename = filename.substr(0, pos);
    return filename;
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

int main(int argc, char* argv[]) {
    string file_path = argv[1];  // File path from argument
    max_exec_seconds = atof(argv[2]);  // Max execution time in seconds
    max_evaluations = atoi(argv[3]);  // Max evaluations
    string init_method = "greedy";  // Default initialization method

    
    srand(time(nullptr));

    read_data(file_path);

    start_time = steady_clock::now();

    // Adjusted parameters: initial temperature and a slower cooling schedule.
    double initial_temp = 1000.0;
    double cooling_factor = 0.997;
    int max_iter = 10000000;

    // Choose the initialization method based on the input argument
    Solution initial;
    if (init_method == "greedy") {
        initial = generate_greedy_solution();
    } else if (init_method == "random") {
        initial = generate_random_solution();
    } else {
        cerr << "Invalid initialization method. Use 'greedy' or 'random'.\n";
        return 1;
    }
    
    // Call simulated annealing with the selected initial solution
    Solution best = simulated_annealing(initial, initial_temp, cooling_factor, max_iter);

    // Extract instance name from file path
    string instance_name = extract_instance_name(file_path);

    // Write solution and checks to output file
    write_solution_to_file(best, instance_name);
    // cout << validate_solution(best);


    return 0;
}
