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
#include <cfloat>
#include "validation.h"
#include <vector>
#include <utility>

using namespace std;
using namespace std::chrono;
std::ofstream log_file("local_search_log.txt");

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

double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

double euclidean_distance(const Customer& a, const Customer& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

// A small exception to unwind out of deeply‐nested loops:
struct StopException : public std::exception {
    const char* what() const noexcept override {
        return "Execution stopped due to time/evaluation limit";
    }
};

// Create a new route with a given customer inserted.
void create_route(Solution &sol, int customer_index) {
    Route new_route;
    new_route.customers.push_back(DEPOT);
    new_route.customers.push_back(customer_index);
    new_route.customers.push_back(DEPOT);
    new_route.total_demand = customers[customer_index].demand;
    sol.push_back(new_route);
}

double compute_insertion_cost(const vector<int>& route, int insert_pos, const Customer& new_cust,
    const vector<Customer>& customers) {
int prev = route[insert_pos - 1];
int next = route[insert_pos];

double removed = euclidean_distance(customers[prev], customers[next]);
double added = euclidean_distance(customers[prev], new_cust) + euclidean_distance(new_cust, customers[next]);

return added - removed;
}

// Compute the cost increase of inserting customer `cust` into `route`
// at position `pos` (between customers[pos-1] and customers[pos]).
double insertion_cost(const Route &route,
    int cust,
    int pos,
    const vector<vector<double>> &dist) {
int prev = route.customers[pos - 1];
int next = route.customers[pos];

double removed = dist[prev][next];
double added   = dist[prev][cust] + dist[cust][next];
return added - removed;
}

// Compute the total cost of serving `cust` in a brand‑new route
// (depot → cust → depot).
double route_cost_for_new_customer(int cust,
                 const vector<vector<double>> &dist) {
return dist[DEPOT][cust] + dist[cust][DEPOT];
}


// --- NEW HELPERS ---

// 1) Compute departure time at the last visited customer
double get_time_at_last(const Route &route) {
    double t = 0.0;
    // skip the final depot—iterate only up to the last real customer
    for (size_t p = 1; p + 1 < route.customers.size(); ++p) {
        int u = route.customers[p-1];
        int v = route.customers[p];
        t += travel_time[u][v];
        t = max(t, (double)customers[v].earliest);
        t += customers[v].service_time;
    }
    return t;
}
// Greedy construction based on sweep (angle) criteria.
// α ∈ [0,1] controls greed vs. randomness
Solution construct_rp_solution(int Ncand) {
    Solution sol;
    // 1) Open one empty route per vehicle
    for (int v = 0; v < M; ++v) {
        Route r;
        r.customers = { DEPOT, DEPOT };
        r.total_demand = 0;
        sol.push_back(r);
    }

    // 2) All customers initially unassigned
    vector<int> unassigned;
    for (int j = 1; j < customers.size(); ++j)
        unassigned.push_back(j);

    // 3) Fill routes in parallel rounds
    while (!unassigned.empty()) {
        bool any_insert = false;

        // for each route k
        for (int k = 0; k < sol.size(); ++k) {
            Route &route = sol[k];

            int i = route.customers[route.customers.size() - 2]; 
            double current = get_time_at_last(route);

            // build (cust, slack) list
            vector<pair<int,double>> cand;
            for (int j : unassigned) {
                // 1) capacity check
                if (route.total_demand + customers[j].demand > Q)
                    continue;

                // 2) time‐window & return‐to‐depot slack
                double Edat = max(current + travel_time[i][j],
                                  (double)customers[j].earliest);
                if (Edat > customers[j].latest) 
                    continue;

                double departure_j = Edat + customers[j].service_time;
                double slack = customers[DEPOT].latest 
                               - travel_time[j][DEPOT]
                               - departure_j;
                if (slack < 0) 
                    continue;

                cand.emplace_back(j, slack);
            }
            if (cand.empty()) 
                continue;

            // sort by decreasing slack, keep top Ncand
            sort(cand.begin(), cand.end(),
                 [](auto &a, auto &b){ return a.second > b.second; });
            int limit = min(Ncand, (int)cand.size());
            int sel = rand() % limit;
            int j_sel = cand[sel].first;

            // now find *best* insertion position by cost
            double best_cost = numeric_limits<double>::infinity();
            int best_pos = -1;
            for (int p = 1; p < route.customers.size(); ++p) {
                if (!can_insert_customer(route, j_sel, p)) 
                    continue;
                double inc = insertion_cost(route, j_sel, p, dist);
                if (inc < best_cost) {
                    best_cost = inc;
                    best_pos = p;
                }
            }

            // if we found a feasible pos, do it
            if (best_pos != -1) {
                route.customers.insert(
                  route.customers.begin() + best_pos, j_sel);
                route.total_demand += customers[j_sel].demand;
                unassigned.erase(
                  remove(unassigned.begin(), unassigned.end(), j_sel),
                  unassigned.end());
                any_insert = true;
            }
        }

        // if no route could take any more customers, break and repair later
        if (!any_insert) 
            break;
    }

    return sol;
}


// Function to generate the initial solution using a random approach
// Solution generate_random_solution() {
//     Solution sol;
//     for (int i = 0; i < M; ++i) {
//         sol.push_back({{DEPOT, DEPOT}, 0});
//     }

//     vector<bool> assigned(customers.size(), false);
//     assigned[DEPOT] = true;

//     // Create a shuffled list of customer IDs (excluding depot)
//     vector<int> cust_ids;
//     for (int i = 1; i < customers.size(); ++i) {
//         cust_ids.push_back(i);
//     }

//     std::random_device rd;
//     std::mt19937 g(rd());
//     std::shuffle(cust_ids.begin(), cust_ids.end(), g);

//     for (int cust : cust_ids) {
//         // Try to assign the customer to a feasible route
//         vector<int> route_order(M);
//         std::iota(route_order.begin(), route_order.end(), 0);  // Fill with 0, 1, ..., M-1
//         std::shuffle(route_order.begin(), route_order.end(), g);

//         bool inserted = false;
//         for (int r : route_order) {
//             for (size_t pos = 1; pos < sol[r].customers.size(); ++pos) {
//                 if (sol[r].total_demand + customers[cust].demand <= Q &&
//                     can_insert_customer(sol[r], cust, pos)) {
//                     sol[r].customers.insert(sol[r].customers.begin() + pos, cust);
//                     sol[r].total_demand += customers[cust].demand;
//                     inserted = true;
//                     break;
//                 }
//             }
//             if (inserted) break;
//         }
//         assigned[cust] = inserted;
//     }

//     return sol;
// }

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
        auto elapsed = duration_cast<seconds>(
            steady_clock::now() - start_time
        ).count();
        if (elapsed >= max_exec_seconds)
            return true;
    }
    return false;
}


bool is_different(const Solution& a, const Solution& b) {
    for (int i = 0; i < a.size(); ++i) {
        if (a[i].customers != b[i].customers)
            return true;
    }
    return false;
}



// Compute total route cost (depot->...->depot)
double route_cost(const Route &route) {
    double cost = 0.0;
    int prev = DEPOT;
    for (int cust : route.customers) {
        cost += dist[prev][cust];
        prev = cust;
    }
    cost += dist[prev][DEPOT];
    return cost;
}

double solution_cost(const Solution &sol) {
    double total = 0.0;
    for (auto &r : sol)
        total += route_cost(r);
    return total;
}

// Intra-route relocate: move customer at 'from' to 'to' in same route
bool relocate_intra_move(Route &route, int from, int to) {
    if (from == to) return false;
    int cust = route.customers[from];
    Route temp = route;
    temp.customers.erase(temp.customers.begin() + from);
    // adjust 'to' if after erase
    if (to > from) --to;
    temp.customers.insert(temp.customers.begin() + to, cust);
    if (!is_feasible(temp)) return false;
    double before = route_cost(route);
    double after  = route_cost(temp);
    evaluation_count++;
    if (after + 1e-6 < before) {
        route = std::move(temp);
        return true;
    }
    return false;
}

// Inter-route relocate: move customer from route i to route j
bool relocate_inter_move(Solution &sol, int ri, int pos_i, int rj, int pos_j) {
    int cust = sol[ri].customers[pos_i];
    Route temp_i = sol[ri];
    Route temp_j = sol[rj];
    temp_i.customers.erase(temp_i.customers.begin() + pos_i);
    temp_i.total_demand -= customers[cust].demand;
    temp_j.customers.insert(temp_j.customers.begin() + pos_j, cust);
    temp_j.total_demand += customers[cust].demand;
    if (!is_feasible(temp_i) || !is_feasible(temp_j)) return false;
    double before = route_cost(sol[ri]) + route_cost(sol[rj]);
    double after  = route_cost(temp_i) + route_cost(temp_j);
    evaluation_count++;
    if (after + 1e-6 < before) {
        sol[ri] = std::move(temp_i);
        sol[rj] = std::move(temp_j);
        return true;
    }
    return false;
}

// Intra-route swap: swap two customers in same route
bool swap_intra_move(Route &route, int p1, int p2) {
    if (p1 == p2) return false;
    Route temp = route;
    std::swap(temp.customers[p1], temp.customers[p2]);
    if (!is_feasible(temp)) return false;
    double before = route_cost(route);
    double after  = route_cost(temp);
    evaluation_count++;
    if (after + 1e-6 < before) {
        route = std::move(temp);
        return true;
    }
    return false;
}

// Inter-route swap: swap customer p1 in route r1 with p2 in r2
bool swap_inter_move(Solution &sol, int r1, int p1, int r2, int p2) {
    int c1 = sol[r1].customers[p1];
    int c2 = sol[r2].customers[p2];
    Route temp1 = sol[r1];
    Route temp2 = sol[r2];
    temp1.customers[p1] = c2;
    temp2.customers[p2] = c1;
    temp1.total_demand = temp1.total_demand - customers[c1].demand + customers[c2].demand;
    temp2.total_demand = temp2.total_demand - customers[c2].demand + customers[c1].demand;
    if (!is_feasible(temp1) || !is_feasible(temp2)) return false;
    double before = route_cost(sol[r1]) + route_cost(sol[r2]);
    double after  = route_cost(temp1) + route_cost(temp2);
    evaluation_count++;
    if (after + 1e-6 < before) {
        sol[r1] = std::move(temp1);
        sol[r2] = std::move(temp2);
        return true;
    }
    return false;
}

// Intra-route 2-opt: reverse subsequence between i and j
bool two_opt_move(Route &route, int i, int j) {
    if (i >= j) return false;
    Route temp = route;
    std::reverse(temp.customers.begin() + i, temp.customers.begin() + j + 1);
    if (!is_feasible(temp)) return false;
    double before = route_cost(route);
    double after  = route_cost(temp);
    evaluation_count++;
    if (after + 1e-6 < before) {
        route = std::move(temp);
        return true;
    }
    return false;
}

// Perform first-improvement local search with the five neighborhoods
bool first_improvement_local_search(Solution &sol) {
    int R = sol.size();
        // 1) Intra-route relocate
        
        for (int r = 0; r < R; ++r) {
            int n = sol[r].customers.size();
            for (int i = 1; i < n - 1; ++i)
                for (int j = 1; j < n - 1; ++j) {
                    log_file << "[Intra-relocate] route=" << r << " from=" << i << " to=" << j << "\n";
                    if (relocate_intra_move(sol[r], i, j)) {
                        log_file << "---> Improved\n";
                        return true;
                    }
                }
        }

        // 2) Inter-route relocate
        for (int r1 = 0; r1 < R; ++r1) {
            for (int r2 = 0; r2 < R; ++r2) {
                if (r1 != r2) {
                    int n1 = sol[r1].customers.size();
                    int n2 = sol[r2].customers.size();
                    for (int i = 1; i < n1 - 1; ++i)
                        for (int j = 1; j < n2; ++j) {
                            log_file << "[Inter-relocate] from route=" << r1 << " pos=" << i << " to route=" << r2 << " pos=" << j << "\n";
                            if (relocate_inter_move(sol, r1, i, r2, j)) {
                                log_file << "---> Improved\n";
                                return true;
                            }
                        }
                }
            }
        }

        // 3) Intra-route swap
        for (int r = 0; r < R; ++r) {
            int n = sol[r].customers.size();
            for (int i = 1; i < n - 1; ++i)
                for (int j = i + 1; j < n - 1; ++j) {
                    log_file << "[Intra-swap] route=" << r << " p1=" << i << " p2=" << j << "\n";
                    if (swap_intra_move(sol[r], i, j)) {
                        log_file << "---> Improved\n";
                        return true;
                    }
                }
        }

        // 4) Inter-route swap
        for (int r1 = 0; r1 < R; ++r1) {
            for (int r2 = r1 + 1; r2 < R; ++r2) {
                int n1 = sol[r1].customers.size();
                int n2 = sol[r2].customers.size();
                for (int i = 1; i < n1 - 1; ++i)
                    for (int j = 1; j < n2 - 1; ++j) {
                        log_file << "[Inter-swap] r1=" << r1 << " p1=" << i << " r2=" << r2 << " p2=" << j << "\n";
                        if (swap_inter_move(sol, r1, i, r2, j)) {
                            log_file << "---> Improved\n";
                            return true;
                        }
                    }
            }
        }

        // 5) Intra-route 2-opt
        for (int r = 0; r < R; ++r) {
            if (time_or_eval_limit_reached()) break;
            int n = sol[r].customers.size();
            for (int i = 1; i < n - 2; ++i)
                for (int j = i + 1; j < n - 1; ++j) {
                    log_file << "[2-opt] route=" << r << " i=" << i << " j=" << j << "\n";
                    if (two_opt_move(sol[r], i, j)) {
                        log_file << "---> Improved\n";
                        return true;
                    }
                }
        }

    return false;
}


// Function to calculate the cost of a solution and increment the evaluation count
double cost_and_count(const Solution &s) {
    ++evaluation_count;
    return calculate_cost(s);
}

// ------------------------------------------------------------------------------
//1) Local‐search descent: keep applying the first‐improving move until none left
// ------------------------------------------------------------------------------
Solution descent_local_search(Solution sol) {
    // we assume first_improvement_local_search(sol) applies one move and returns true
    // if it found an improving, feasible move
    while ( first_improvement_local_search(sol) &&
            !time_or_eval_limit_reached()) {
        // keep going until no move is found
    }
    return sol;
}


// -----------------------------------------------------------------------------
//2) The GRASP driver, updated to use the new constructor and local‐search
// -----------------------------------------------------------------------------
Solution GRASP(int max_iters,int Ncand) 
{
    Solution best_sol;
    double best_cost = numeric_limits<double>::infinity();

    while(!time_or_eval_limit_reached()) {
            for (int iter = 0; iter < max_iters; ++iter) {
                if (time_or_eval_limit_reached()) break;

                // --- Phase 1: Construct initial solution with RP heuristic
                Solution sol = construct_rp_solution(Ncand);
                if (time_or_eval_limit_reached()) break;

                // --- Phase 2: Descent local search using all 5 neighborhoods
                sol = descent_local_search(sol);

                // --- Evaluate & update best
                double c = cost_and_count(sol);
                if (c < best_cost) {
                    best_cost = c;
                    best_sol  = sol;
                    cout << "GRASP iter " << iter
                         << " new best = " << best_cost
                         << "  (evals=" << evaluation_count << ")\n";
                }
        }
    }


    return best_sol;
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
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <input_file> <max_exec_seconds> <max_evaluations>\n";
        return 1;
    }

    string file_path = argv[1];  // File path from argument
    max_exec_seconds = atof(argv[2]);  // Max execution time in seconds
    max_evaluations = atoi(argv[3]);  // Max evaluations
    
    srand(time(nullptr));
    read_data(file_path);
    start_time = steady_clock::now();

    // Run GRASP
    int Ncand = 5;
    int max_iters = 100;

    Solution best;
    best = GRASP(max_iters, Ncand);

    log_file.close();

    // stop the clock
    auto end_time = steady_clock::now();
    double elapsed_s = duration_cast<duration<double>>(end_time - start_time).count();
    
    // Extract instance name from file path
    string instance_name = extract_instance_name(file_path);

    // Write solution and checks to output file
    write_solution_to_file(best, instance_name);
    cout << validate_solution(best);

    // REPORT the two budgets you actually used:
    std::cout << "Total cost‑evaluations performed: " 
            << evaluation_count << "\n";
    std::cout << "Total elapsed time: " 
                << elapsed_s << " seconds\n";



    return 0;
}