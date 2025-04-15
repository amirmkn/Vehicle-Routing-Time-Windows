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

// bool is_solution_feasible(const Solution& sol) {
//     for (const auto& route : sol) {
//         if (route.customers.size() > 2 && !is_feasible(route))
//             return false;
//     }
//     return true;
// }


double rand01() {
    return static_cast<double>(rand()) / RAND_MAX;
}

double euclidean_distance(const Customer& a, const Customer& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}


Solution generate_greedy_solution(){
    Solution sol;  // Initially empty: no routes

    // The depot is considered already "assigned" (if needed in your logic)
    vector<bool> assigned(customers.size(), false);
    assigned[DEPOT] = true;

    // Function to create a new route with a given customer inserted
    auto create_route = [&](int customer_index) {
        Route new_route;
        new_route.customers.push_back(DEPOT);
        new_route.customers.push_back(customer_index);
        new_route.customers.push_back(DEPOT);
        new_route.total_demand = customers[customer_index].demand;
        sol.push_back(new_route);
    };

    // Insertion heuristic: try to insert customer into existing routes
    for (int i = 1; i < customers.size(); ++i) {
        if (assigned[i])
            continue;
        double best_incr = numeric_limits<double>::max();
        int best_route = -1;
        int best_pos = -1;
        for (int r = 0; r < sol.size(); ++r) {
            // Check if the customer can be feasibly added considering the capacity
            if (sol[r].total_demand + customers[i].demand > Q)
                continue;
            // Try every possible insertion position within route r (between depots and inserted customers)
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
        // If found a feasible insertion in an existing route, insert customer i there
        if (best_route != -1) {
            sol[best_route].customers.insert(sol[best_route].customers.begin() + best_pos, i);
            sol[best_route].total_demand += customers[i].demand;
            assigned[i] = true;
        } else {
            // No feasible insertion found in any existing route, start a new route with this customer
            if(customers[i].demand <= Q) {
                create_route(i);
                assigned[i] = true;
            }
        }
    }

    // In the rare instance that some customers remain unassigned (e.g., due to routing restrictions),
    // try to insert them at the end of any route where they fit.
    for (int i = 1; i < customers.size(); ++i) {
        if (!assigned[i]) {
            for (int r = 0; r < sol.size(); ++r) {
                int pos = sol[r].customers.size() - 1; // before final depot
                if (sol[r].total_demand + customers[i].demand <= Q &&
                    can_insert_customer(sol[r], i, pos)) {
                    sol[r].customers.insert(sol[r].customers.begin() + pos, i);
                    sol[r].total_demand += customers[i].demand;
                    assigned[i] = true;
                    break;
                }
            }
            // If still not assigned, you might need to create a new route (if feasible) or handle infeasibility.
            if (!assigned[i] && customers[i].demand <= Q) {
                create_route(i);
                assigned[i] = true;
            }
        }
    }

    // Finally, ensure that we only return routes that have actual customers (besides the depots)
    // This is more of a safeguard since the above logic should avoid creating empty routes.
    Solution sol_nonempty;
    for (const auto& route : sol) {
        if (route.customers.size() > 2) {  // more than just the two depot entries
            sol_nonempty.push_back(route);
        }
    }
    return sol_nonempty;
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
    return active_routes * 10000.0 + total_distance;
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

bool is_different(const Solution& a, const Solution& b) {
    for (int i = 0; i < a.size(); ++i) {
        if (a[i].customers != b[i].customers)
            return true;
    }
    return false;
}


// Operator weighting parameters for adaptive operator selection:
const int NUM_OPERATORS = 5; // 0: Rem-In, 1: Swap, 2: 2-opt, 3: Or-opt, 4: Relocate
double op_weight[NUM_OPERATORS] = {1.0, 1.0, 1.0, 1.0, 1.0};
int op_total[NUM_OPERATORS] = {0,0,0,0,0};
int op_success[NUM_OPERATORS] = {0,0,0,0,0};

// === Penalty-related functions ===

// Returns the capacity violation for a single route.
double capacity_violation(const Route &route) {
    int violation = route.total_demand - Q;
    return (violation > 0) ? violation : 0;
}

// Returns the total penalty for a solution (here, only capacity violations are penalized).
double total_penalty(const Solution &sol) {
    double penalty = 0.0;
    for (const auto &route : sol) {
        penalty += capacity_violation(route);
    }
    return penalty;
}

// Returns the penalized cost: raw cost + λ * (total penalty)
double calculate_penalized_cost(const Solution &sol, double lambda) {
    return calculate_cost(sol) + lambda * total_penalty(sol);
}

// Returns an operator index [0,3] based on weights.
int select_operator() {
    double sum = 0.0;
    for (int i = 0; i < 4; i++) {
        sum += op_weight[i];
    }
    double rnd = ((double) rand() / RAND_MAX) * sum;
    double cumulative = 0;
    for (int i = 0; i < 5; i++) {
        cumulative += op_weight[i];
        if (rnd < cumulative)
            return i;
    }
    return 0; // fallback
}
double route_cost(const Route& route) {
    if (route.customers.empty()) return 0.0;

    double cost = 0.0;
    int prev = 0;  // Assuming route starts at depot (index 0)

    for (int cust : route.customers) {
        cost += dist[prev][cust];
        prev = cust;
    }

    cost += dist[prev][0];  // Return to depot
    return cost;
}

// Intra-route 2-opt reversal operator (operator 2)
// Reverse a subsequence within a route.
bool two_opt_move(Route &route, std::ofstream& logFile) {
    int n = route.customers.size();
    if (n <= 3) return false;

    double old_cost = route_cost(route);  // Add your cost function here

    const int max_attempts = 10;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        int i = rand() % (n - 2) + 1;
        int j = rand() % (n - i - 1) + i + 1;

        std::reverse(route.customers.begin() + i, route.customers.begin() + j + 1);

        if (!is_feasible(route)) {
            std::reverse(route.customers.begin() + i, route.customers.begin() + j + 1);
            continue;
        }

        double new_cost = route_cost(route);
        if (fabs(new_cost - old_cost) > 1e-3) {
            logFile << "2-opt accepted: (" << i << ", " << j << "), cost delta: " << (new_cost - old_cost) << "\n";
            return true;
        }

        std::reverse(route.customers.begin() + i, route.customers.begin() + j + 1); // revert if no gain
    }

    return false;
}


// Or-opt move (operator 3)
// Move a block of 1 or 2 consecutive customers from one route to another.
bool or_opt_move(Solution &sol, std::ofstream& logFile) {
    if (sol.empty()) return false;
    int solSize = sol.size();
    int r1 = rand() % solSize;
    int r2 = rand() % solSize;
    if (r1 == r2 || sol[r1].customers.size() <= 3)
        return false;
    int r1Size = sol[r1].customers.size();
    int max_block = std::min(2, r1Size - 2);
    if (max_block <= 0) return false;
    int block_len = rand() % max_block + 1;
    if (r1Size - block_len <= 2)
        return false;
    int max_pos_from = r1Size - block_len - 1;
    if (max_pos_from < 1) return false;
    int pos_from = rand() % max_pos_from + 1;
    if (pos_from + block_len > sol[r1].customers.size() - 1)
        return false;
    int block_demand = 0;
    std::vector<int> block;
    for (int i = 0; i < block_len; ++i) {
        int idx = pos_from + i;
        int cust = sol[r1].customers[idx];
        if (cust <= 0 || cust >= (int)customers.size())
            return false;
        block.push_back(cust);
        block_demand += customers[cust].demand;
    }
    if (sol[r2].customers.size() <= 1)
        return false;
    int r2Size = sol[r2].customers.size();
    int max_insert_pos = r2Size - block_len;
    if (max_insert_pos < 1) return false;
    int insert_pos = rand() % max_insert_pos + 1;
    if (sol[r2].total_demand + block_demand > Q)
        return false;
    for (int i = 0; i < block_len; ++i) {
        if (!can_insert_customer(sol[r2], block[i], insert_pos + i))
            return false;
    }
    sol[r1].customers.erase(sol[r1].customers.begin() + pos_from,
                             sol[r1].customers.begin() + pos_from + block_len);
    for (int cust : block) {
        sol[r1].total_demand -= customers[cust].demand;
    }
    sol[r2].customers.insert(sol[r2].customers.begin() + insert_pos,
                             block.begin(), block.end());
    for (int cust : block) {
        sol[r2].total_demand += customers[cust].demand;
    }
    if (!is_feasible(sol[r1]) || !is_feasible(sol[r2]))
        return false;
    return true;
}

// Returns a random route index from the solution that contains at least one real customer.
int getNonEmptyRoute(const Solution &sol) {
    std::vector<int> validRoutes;
    for (int i = 0; i < (int)sol.size(); i++) {
        if (sol[i].customers.size() > 2)
            validRoutes.push_back(i);
    }
    if (validRoutes.empty())
        return -1;
    return validRoutes[rand() % validRoutes.size()];
}

// Returns a random route index among those with available capacity and at least one real customer.
int getRouteWithCapacity(const Solution &sol, int custDemand) {
    std::vector<int> validRoutes;
    for (int i = 0; i < (int)sol.size(); i++) {
        if ((sol[i].total_demand + custDemand <= Q) && (sol[i].customers.size() > 2))
            validRoutes.push_back(i);
    }
    if (validRoutes.empty())
        return -1;
    return validRoutes[rand() % validRoutes.size()];
}

Solution perturb_solution(const Solution& sol) {
    Solution perturbed = sol;

    int num_perturbations = std::max(3, static_cast<int>(customers.size() * 0.05));  // 5% of customers
    for (int i = 0; i < num_perturbations; ++i) {
        int from_route = getNonEmptyRoute(perturbed);
        int to_route = getRouteWithCapacity(perturbed, 0);

        if (from_route == -1 || to_route == -1 || from_route == to_route) continue;

        if (perturbed[from_route].customers.size() <= 3) continue;

        int pos = rand() % (perturbed[from_route].customers.size() - 2) + 1;
        int cust = perturbed[from_route].customers[pos];

        perturbed[from_route].customers.erase(perturbed[from_route].customers.begin() + pos);
        perturbed[from_route].total_demand -= customers[cust].demand;

        int insert_pos = (perturbed[to_route].customers.size() > 1) ?
                         rand() % (perturbed[to_route].customers.size() - 1) + 1 : 1;

        perturbed[to_route].customers.insert(perturbed[to_route].customers.begin() + insert_pos, cust);
        perturbed[to_route].total_demand += customers[cust].demand;
    }

    return perturbed;
}

// Global parameter for penalizing infeasibilities.
const double lambda = 100.0;  // Adjust according to your problem scale

Solution generate_neighbor(const Solution& sol) {
    std::ofstream logFile("neighbor_generation.txt", std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Error opening log file" << std::endl;
        return sol;
    }

    // Compute current penalized cost.
    double current_penalized = calculate_penalized_cost(sol, lambda);

    Solution neighbor = sol;
    int op;
    if (rand() % 10 == 0) {
        op = rand() % NUM_OPERATORS;  // 10% chance to pick randomly
    } else {
        op = select_operator();         // weighted choice of operator
    }
    op_total[op]++;
    bool moveApplied = false;

    // Try applying one of the operators.
    switch (op) {
        case 0: {  // Smart Removal–Insertion
            logFile << "Operator: Smart Removal–Insertion" << std::endl;
            for (int attempt = 0; attempt < 10; ++attempt) {
                int v1 = getNonEmptyRoute(neighbor);
                int v2 = getRouteWithCapacity(neighbor, 0); // still helps selecting a route with some spare capacity
                if (v1 == -1 || v2 == -1 || v1 == v2) continue;
                if (neighbor[v1].customers.size() <= 3) continue;

                int pos = rand() % (neighbor[v1].customers.size() - 2) + 1;
                int cust = neighbor[v1].customers[pos];
                Route origV1 = neighbor[v1], origV2 = neighbor[v2];

                neighbor[v1].customers.erase(neighbor[v1].customers.begin() + pos);
                neighbor[v1].total_demand -= customers[cust].demand;

                int best_pos = -1;
                double best_penalized = DBL_MAX;
                // Instead of checking feasibility, evaluate the penalized cost.
                for (int i = 1; i < neighbor[v2].customers.size(); ++i) {
                    if (neighbor[v2].total_demand + customers[cust].demand <= Q || true) {
                        neighbor[v2].customers.insert(neighbor[v2].customers.begin() + i, cust);
                        double test_cost = calculate_penalized_cost(neighbor, lambda);
                        if (test_cost < best_penalized) {
                            best_penalized = test_cost;
                            best_pos = i;
                        }
                        neighbor[v2].customers.erase(neighbor[v2].customers.begin() + i);
                    }
                }

                if (best_pos != -1) {
                    neighbor[v2].customers.insert(neighbor[v2].customers.begin() + best_pos, cust);
                    neighbor[v2].total_demand += customers[cust].demand;
                    double new_penalized = calculate_penalized_cost(neighbor, lambda);
                    if (new_penalized < current_penalized - 1e-3) {
                        moveApplied = true;
                        logFile << "Inserted customer " << cust << " from route " << v1 << " to route " << v2 
                                << ", penalized Δcost: " << (new_penalized - current_penalized) << std::endl;
                        break;
                    }
                }
                // Revert changes if no improvement.
                neighbor[v1] = origV1;
                neighbor[v2] = origV2;
            }
            break;
        }
        case 1: {  // Swap
            logFile << "Operator: Swap" << std::endl;
            const int max_attempts = 10;
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                int r1 = getNonEmptyRoute(neighbor);
                int r2 = getNonEmptyRoute(neighbor);
                if (r1 == -1 || r2 == -1 || r1 == r2) continue;
                if (neighbor[r1].customers.size() <= 3 || neighbor[r2].customers.size() <= 3) continue;
        
                int pos1 = rand() % (neighbor[r1].customers.size() - 2) + 1;
                int pos2 = rand() % (neighbor[r2].customers.size() - 2) + 1;
        
                int cust1 = neighbor[r1].customers[pos1];
                int cust2 = neighbor[r2].customers[pos2];
        
                double current_pen = calculate_penalized_cost(neighbor, lambda);
        
                std::swap(neighbor[r1].customers[pos1], neighbor[r2].customers[pos2]);
                
                double new_pen = calculate_penalized_cost(neighbor, lambda);
                if (new_pen < current_pen - 1e-3) {
                    // Update total demands
                    neighbor[r1].total_demand = neighbor[r1].total_demand - customers[cust1].demand + customers[cust2].demand;
                    neighbor[r2].total_demand = neighbor[r2].total_demand - customers[cust2].demand + customers[cust1].demand;
                    moveApplied = true;
                    logFile << "Swapped customer " << cust1 << " (r" << r1 << ") with " << cust2 
                            << " (r" << r2 << "), penalized Δcost: " << (new_pen - current_pen) << "\n";
                    break;
                }
        
                std::swap(neighbor[r1].customers[pos1], neighbor[r2].customers[pos2]);  // revert
            }
            break;
        }
        case 2: {  // Intra-route 2-opt move
            logFile << "Operator: Intra-route 2-opt" << std::endl;
            for (int attempt = 0; attempt < 10; ++attempt) {
                int r = getNonEmptyRoute(neighbor);
                if (r == -1 || neighbor[r].customers.size() <= 3) continue;
                // Capture current penalized cost for this route change.
                double current_pen = calculate_penalized_cost(neighbor, lambda);
                
                if (two_opt_move(neighbor[r], logFile)) {
                    double new_pen = calculate_penalized_cost(neighbor, lambda);
                    if (new_pen < current_pen - 1e-3) {
                        moveApplied = true;
                        break;
                    }
                }
            }
            break;
        }
        case 3: {  // Or-opt move
            logFile << "Operator: Or-opt" << std::endl;
            {
                double current_pen = calculate_penalized_cost(neighbor, lambda);
                if (or_opt_move(neighbor, logFile)) {
                    double new_pen = calculate_penalized_cost(neighbor, lambda);
                    if (new_pen < current_pen - 1e-3) {
                        moveApplied = true;
                    }
                }
            }
            break;
        }
        case 4: {  // Perturbation
            logFile << "Operator: Perturbation" << std::endl;
            Solution temp = perturb_solution(sol);
            double new_pen = calculate_penalized_cost(temp, lambda);
            if (new_pen < current_penalized - 1e-3) {
                neighbor = temp;
                moveApplied = true;
            }
            break;
        }
        default:
            break;
    }

    // Update operator weights based on penalized improvement.
    double new_penalized = calculate_penalized_cost(neighbor, lambda);
    if (moveApplied && new_penalized < current_penalized - 1e-3 && neighbor != sol) {
        op_success[op]++;
        op_weight[op] += 0.1;
    } else {
        op_weight[op] = std::max(0.1, op_weight[op] - 0.05);
    }

    logFile << "Old penalized cost: " << current_penalized
            << ", New penalized cost: " << new_penalized << std::endl;
    logFile.close();
    return (moveApplied) ? neighbor : sol;
}


void update_operator_weights() {
    for (int i = 0; i < 4; i++) {
        if (op_total[i] > 0) {
            double success_rate = static_cast<double>(op_success[i]) / op_total[i];
            op_weight[i] = 0.8 * op_weight[i] + 0.2 * success_rate;
        }
        op_success[i] = 0;
        op_total[i] = 0;
    }
}

void print_solution(const Solution& sol, const std::string& filename) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Error opening " << filename << std::endl;
        return;
    }

    out << "=== Initial Solution ===\n";
    for (size_t i = 0; i < sol.size(); ++i) {
        out << "Route " << i << " (Demand: " << sol[i].total_demand << "): ";
        for (int cust : sol[i].customers) {
            out << cust << " ";
        }
        out << "\n";
    }

    out << "Total Cost: " << calculate_cost(sol) << "\n";
    out.close();
}

// ----- Tabu Search Implementation -----
#include <cmath>
// (Include other necessary headers as before)

struct TabuEntry {
    Solution sol;
    int tenure;
};

bool solution_equal(const Solution &s1, const Solution &s2) {
    if (s1.size() != s2.size()) return false;
    for (size_t i = 0; i < s1.size(); i++) {
        if (!(s1[i] == s2[i])) return false;
    }
    return true;
}

bool is_tabu(const Solution &candidate, const vector<TabuEntry> &tabu_list) {
    for (const auto &entry : tabu_list) {
        if (solution_equal(candidate, entry.sol))
            return true;
    }
    return false;
}

Solution tabu_search(Solution initial, int tabu_tenure) {
    // Base exploration parameter (lambda) for probabilistic acceptance.
    const double lambda = 100.0;              // Normal mode: acceptance probability factor.
    const double lambda_explore = 1000.0;       // Exploratory mode: higher lambda => more likely to accept worse moves.
    const int consistency_threshold = 100;      // Trigger exploration after 100 evaluations without improvement.
    
    Solution current = initial;
    print_solution(initial, "initial_solution.txt");
    Solution best = current;
    double current_cost = calculate_cost(current);
    double best_cost = current_cost;
    vector<TabuEntry> tabu_list;
    
    // Count iterations during which best hasn't improved.
    int consistency_count = 0;
    
    ofstream logFile("log.txt");
    if (!logFile.is_open()) {
        cerr << "Unable to open log file!" << endl;
        return initial;
    }
    
    int iter = 0;
    while (!time_or_eval_limit_reached()) {
        evaluation_count++;
        
        // Generate a candidate neighbor solution.
        Solution candidate = generate_neighbor(current);
        double candidate_cost = calculate_cost(candidate);
        
        // Check Tabu list with aspiration criterion
        int trials = 0;
        const int max_trials = 10;
        while (is_tabu(candidate, tabu_list) && candidate_cost >= best_cost && trials < max_trials) {
            candidate = generate_neighbor(current);
            candidate_cost = calculate_cost(candidate);
            trials++;
        }
        
        logFile << "Iter " << iter << ": Candidate generated with cost " << candidate_cost << std::endl;
        
        // Determine if candidate is accepted.
        double delta = candidate_cost - current_cost;
        // Use an effective lambda that increases (making acceptance more generous)
        // if the solution has been consistent (i.e., no improvement) for a while.
        double effective_lambda = (consistency_count >= consistency_threshold) ? lambda_explore : lambda;
        
        if (delta < 0) {
            // Improvement: always accept and reset consistency counter.
            current = candidate;
            current_cost = candidate_cost;
            consistency_count = 0;
            logFile << "Accepted better candidate." << std::endl;
        } else {
            // Accept worse candidate with probability p = exp(-delta / effective_lambda)
            double acceptance_probability = exp(-delta / effective_lambda);
            if (rand01() < acceptance_probability) {
                current = candidate;
                current_cost = candidate_cost;
                logFile << "Accepted worse candidate with probability " << acceptance_probability << std::endl;
            } else {
                logFile << "Rejected worse candidate (probability " << acceptance_probability << ")." << std::endl;
            }
            consistency_count++; // Increment consistency count since no improvement on best.
        }
        
        // Check and update best solution if an improvement is found.
        if (current_cost < best_cost) {
            best = current;
            best_cost = current_cost;
            logFile << ">>> New best found with cost " << best_cost << std::endl;
            consistency_count = 0;  // Reset, since improvement was found.
        }
        
        // Optionally, if you're entering exploration mode, log it.
        if (consistency_count == consistency_threshold)
            logFile << ">>> Exploration mode triggered after " << consistency_threshold << " consistent evaluations." << std::endl;
        
        // Add current solution to the Tabu list.
        tabu_list.push_back({current, tabu_tenure});
        for (auto it = tabu_list.begin(); it != tabu_list.end(); ) {
            it->tenure--;
            if (it->tenure <= 0)
                it = tabu_list.erase(it);
            else
                ++it;
        }
        
        logFile << "Iter: " << iter
                << ", Current Cost: " << current_cost
                << ", Best Cost: " << best_cost
                << ", Tabu List Size: " << tabu_list.size()
                << ", Consistency Count: " << consistency_count
                << ", Eval: " << evaluation_count
                << std::endl;
        
        iter++;
    }
    
    logFile << "Tabu Search completed after " << iter << " iterations." << std::endl;
    logFile.close();
    
    auto now = chrono::steady_clock::now();
    chrono::duration<double> elapsed = now - start_time;
    cout << "🔁 Evaluations done: " << evaluation_count << endl;
    cout << "⏱️  Time elapsed: " << elapsed.count() << " seconds" << endl;
    cout << "🔥 Best cost found: " << best_cost << endl;
    
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

    // Tabu Search Parameters
    // int max_iter = 100000;         // You can tune this based on your instance size
    int tabu_tenure = 5;          // Tabu list size or tenure (typical range: 5–100)

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
    
    // Run Tabu Search
    Solution best = tabu_search(initial,  tabu_tenure);

    // Extract instance name from file path
    string instance_name = extract_instance_name(file_path);

    // Write solution and checks to output file
    write_solution_to_file(best, instance_name);
    cout << validate_solution(best);


    return 0;
}
