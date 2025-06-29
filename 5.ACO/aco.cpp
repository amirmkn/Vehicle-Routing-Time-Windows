#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ctime>
#include <random>
#include <limits>
#include <chrono>
#include <unordered_set>
#include "validation.h"

using namespace std;

// Global problem data
int DEPOT = 0;
double Q;                         // Vehicle capacity
int M;                            // Max vehicles
int maxEvaluations = 0;     // default maximum number of objective evaluations
int maxTimeSeconds = 0;         // default maximum execution time in seconds
int vehiclePenalty = 10000;
vector<Customer> customers;
vector<vector<double>> dist, travel_time;
vector<Route> bestSolution;
double bestDistance;

struct SolutionScore {
    int numVehicles;
    double totalDistance;

    bool operator<(const SolutionScore& other) const {
        if (numVehicles != other.numVehicles)
            return numVehicles < other.numVehicles;
        return totalDistance < other.totalDistance;
    }
};

string get_output_filename(const string& inputPath) {
    size_t slash = inputPath.find_last_of("/");
    string filename = (slash == string::npos) ? inputPath : inputPath.substr(slash + 1);
    size_t dot = filename.find_last_of(".");
    if (dot != string::npos) filename = filename.substr(0, dot);
    return filename + "_output.txt";
}

// ACO parameters
int resetCountdown   = 2;    // how many iterations remain with β reduced
const int RESET_ITER =  20;   // number of iterations to keep β low after a reset
int numAnts = 10;
int maxIterations = 1000;
double alpha = 3.0;     // Influence of pheromone // the more, the more exploitation
double beta = 1.0;      // Influence of heuristic (1/distance) //the more the greedier(shortest edges preferred)
const double BETA_LOW =  0.1;  // reduced β during reset-mode
const double BETA_High =  1.0;  // restored β after reset-mode
double rho = 0.2;       // Evaporation rate (the more the faster the evaporation gets, limits early convergence, we have more exploration)
vector<vector<double>> pheromone;  // Pheromone matrix
double a_factor   = 1.0;    // iteration‐best scale
double b_factor   = 1.0;    // global‐best scale
int    sigma      = 5;      // top‐ranked ants to deposit



double euclidean(const Customer &a, const Customer &b) {
    return hypot(a.x - b.x, a.y - b.y);
}

void read_input(const string &filename) {
    ifstream infile(filename);
    if (!infile) { cerr << "Error opening " << filename << "\n"; exit(1); }
    string line;
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            getline(infile, line);
            istringstream iss(line);
            iss >> M >> Q;
            break;
        }
    }
    while (getline(infile, line) && line.find("CUSTOMER") == string::npos);
    getline(infile, line); // header
    getline(infile, line); // header
    customers.clear();
    while (getline(infile, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        Customer c;
        iss >> c.id >> c.x >> c.y >> c.demand >> c.earliest >> c.latest >> c.service_time;
        customers.push_back(c);
    }
}

// --------------------------------------------------------------------------------------------------
// Nearest‐Neighbor (NN) Heuristic for VRPTW
// --------------------------------------------------------------------------------------------------
//
// Starting at depot, greedily choose the closest feasible unvisited customer (respecting capacity
// and time‐window).  Once no more customers can fit on this vehicle/route, return to depot,
// then open a new vehicle (if you still have vehicles left and unserved customers).
//
// Return value: total distance of the combined NN routes.  `nnRoutes` will hold all routes.
//
// If any customer cannot be served at all (e.g., its demand > Q or its time‐window is impossible),
// we still attempt to serve as many as possible; you might check feasibility before calling.

double nearest_neighbor_solution(vector<Route> &nnRoutes) {
    int n = customers.size();
    vector<bool> visited(n, false);
    visited[DEPOT] = true;
    int remaining = n - 1;

    nnRoutes.clear();
    double totalDistance = 0.0;

    for (int vehicle = 0; vehicle < M && remaining > 0; ++vehicle) {
        Route route;
        route.total_demand = 0;
        int current = DEPOT;
        double load = 0.0;
        double currentTime = 0.0;

        route.customers.push_back(DEPOT);

        // Build one route greedily
        while (true) {
            // 1) Find the nearest feasible unvisited customer j
            int   bestJ     = -1;
            double bestDist = numeric_limits<double>::infinity();

            for (int j = 1; j < n; ++j) {
                if (visited[j]) 
                    continue;
                if (load + customers[j].demand > Q) 
                    continue;

                double travel = travel_time[current][j];
                double arrival = max(currentTime + travel, (double)customers[j].earliest);
                if (arrival > customers[j].latest) 
                    continue;

                // Among all feasible j, pick the one with minimum travel_time[current][j]
                if (travel < bestDist) {
                    bestDist = travel;
                    bestJ = j;
                }
            }

            // 2) If no feasible j found, close this route
            if (bestJ < 0) 
                break;

            // 3) Commit to bestJ
            visited[bestJ] = true;
            --remaining;

            route.customers.push_back(bestJ);
            route.total_demand += customers[bestJ].demand;

            double travel   = travel_time[current][bestJ];
            double arrival  = max(currentTime + travel, (double)customers[bestJ].earliest);
            currentTime     = arrival + customers[bestJ].service_time;
            load           += customers[bestJ].demand;
            totalDistance  += travel;   // Add leg distance right away
            current         = bestJ;
        }

        // 4) Come back to depot if we visited anyone beyond depot
        if (route.customers.size() > 1) {
            // Add the return leg (current -> DEPOT)
            double backHome = travel_time[current][DEPOT];
            totalDistance += backHome;
            route.customers.push_back(DEPOT);
            nnRoutes.push_back(route);
        }
        else {
            // We added only depot and immediately found no one feasible—no need to push a trivial route
        }
    }

    return totalDistance;
}

void init_pheromone() {
    int n = customers.size();
    pheromone.assign(n, vector<double>(n, 1.0));
}

double heuristic(int i, int j) {
    double d = dist[i][j];
    return (d > 0 ? 1.0 / d : 1e6);
}

SolutionScore build_solution(int antId, vector<Route> &antRoutes) {
    vector<bool> visited(customers.size(), false);
    visited[DEPOT] = true;
    antRoutes.clear();
    int remaining = customers.size() - 1;

    while (remaining > 0 && (int)antRoutes.size() < M) {
        Route route;
        int current = DEPOT;
        double load = 0;
        double currentTime = 0;

        route.customers.push_back(DEPOT); // Start at depot

        while (true) {
            vector<int> cands;
            vector<double> probs;
            double sumProb = 0;
            for (int j = 1; j < (int)customers.size(); ++j) {
                if (visited[j]) continue;
                if (load + customers[j].demand > Q) continue;
                double travel = travel_time[current][j];
                double arrival = max(currentTime + travel, (double)customers[j].earliest);
                if (arrival > customers[j].latest) continue;

                cands.push_back(j);
                double tau = pow(pheromone[current][j], alpha);
                double eta = pow(heuristic(current, j), beta);
                double prob = tau * eta;
                probs.push_back(prob);
                sumProb += prob;
            }

            if (cands.empty()) break;

            double pick = ((double) rand() / RAND_MAX) * sumProb;
            double cum = 0;
            int next = cands.back();
            for (size_t k = 0; k < cands.size(); ++k) {
                cum += probs[k];
                if (cum >= pick) {
                    next = cands[k];
                    break;
                }
            }

            visited[next] = true;
            --remaining;
            route.customers.push_back(next);
            route.total_demand += customers[next].demand;
            double travel = travel_time[current][next];
            currentTime = max(currentTime + travel, (double)customers[next].earliest) + customers[next].service_time;
            load += customers[next].demand;
            current = next;
        }

        route.customers.push_back(DEPOT); // End at depot
        if (route.customers.size() > 2)
        antRoutes.push_back(route);
    }

    double totalDist = 0;
    for (auto &r : antRoutes) {
        for (size_t i = 1; i < r.customers.size(); ++i) {
            totalDist += travel_time[r.customers[i - 1]][r.customers[i]];
        }
    }
    return { (int)antRoutes.size(), totalDist };
}

// void update_pheromone(const vector<vector<Route>> &allRoutes, const vector<double> &allDistances) {
//     int n = customers.size();
//     for (int i = 0; i < n; ++i)
//         for (int j = 0; j < n; ++j)
//             pheromone[i][j] *= (1.0 - rho);

//     int bestIdx = min_element(allDistances.begin(), allDistances.end()) - allDistances.begin();
//     const auto &bestRoutes = allRoutes[bestIdx];
//     double invDist = 1.0 / allDistances[bestIdx];

//     for (auto &r: bestRoutes) {
//         for (size_t i = 1; i < r.customers.size(); ++i) {
//             int from = r.customers[i - 1];
//             int to = r.customers[i];
//             pheromone[from][to] += invDist;
//             pheromone[to][from] += invDist;
//         }
//     }
// }

// void update_pheromone(const vector<vector<Route>> &allRoutes,
//                       const vector<double>            &allDistances) {
//     // 1) Evaporate
//     for (int i = 0; i < n; ++i)
//       for (int j = 0; j < n; ++j)
//         pheromone[i][j] *= (1.0 - rho);

//     // 2) Deposit for iteration‐best (even if worse than global best)
//     int iterBest = min_element(allDistances.begin(), allDistances.end()) - allDistances.begin();
//     double iterDelta = 1.0 / allDistances[iterBest];
//     for (auto &r : allRoutes[iterBest]) {
//       for (size_t k = 1; k < r.customers.size(); ++k) {
//         int u = r.customers[k - 1];
//         int v = r.customers[k];
//         pheromone[u][v] += iterDelta;
//         pheromone[v][u] += iterDelta;
//       }
//     }

//     // 3) If this iteration improved upon the _global_ best, add an elitist bonus
//     if (allDistances[iterBest] < bestDistance) {
//       bestDistance = allDistances[iterBest];
//       bestSolution = allRoutes[iterBest];
//       double eliteDelta = 2.0 / bestDistance;  
//       // (a slightly larger deposit for the global best)
//       for (auto &r : bestSolution) {
//         for (size_t k = 1; k < r.customers.size(); ++k) {
//           int u = r.customers[k - 1];
//           int v = r.customers[k];
//           pheromone[u][v] += eliteDelta;
//           pheromone[v][u] += eliteDelta;
//         }
//       }
//     }
// }


void update_pheromone_ranked(
    const vector<vector<Route>>  &allAntRoutes,
    const vector<SolutionScore>  &allScores,
    double                        rho,
    int                           sigma,
    double                        a_factor,
    double                        b_factor,
    double                        vehiclePenalty,
    SolutionScore                 bestScore
) {
    int n = customers.size();

    // 1) Evaporate all pheromone entries:
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            pheromone[i][j] *= (1.0 - rho);
            if (pheromone[i][j] < 1e-12) {
                pheromone[i][j] = 1e-12;  // floor to avoid zeros
            }
        }
    }

    // 2) Build a list of ant indices 0..numAnts-1, then sort by lexicographic score:
    int numAntsLocal = (int)allAntRoutes.size();
    vector<int> antIdx(numAntsLocal);
    iota(antIdx.begin(), antIdx.end(), 0);

    // Sort so that the ant with fewer vehicles (then shorter distance) comes first
    sort(antIdx.begin(), antIdx.end(),
         [&](int u, int v) {
             // Use the overloaded operator< on SolutionScore
             return allScores[u] < allScores[v];
         });

    // 3) Let the top 'sigma' ants deposit pheromone
    int topCount = min(sigma, numAntsLocal);
    for (int rank = 0; rank < topCount; ++rank) {
        int antId = antIdx[rank];
        const SolutionScore &sc = allScores[antId];

        // Compute a single‐scalar “fitness” so that fewer vehicles always wins.
        // (You chose vehiclePenalty large enough so reducing 1 vehicle > any distance gain.)
        double fitnessMu = sc.numVehicles * vehiclePenalty + sc.totalDistance;
        if (fitnessMu <= 0) 
            continue;   // skip degenerate cases (shouldn't really happen)

        // Weight for this ant: (sigma - rank) / fitnessMu, scaled by a_factor
        double depositMu = a_factor * ((double)(sigma - rank) / fitnessMu);

        // Now deposit on every edge in this ant's routes
        for (const Route &r : allAntRoutes[antId]) {
            for (size_t k = 1; k < r.customers.size(); ++k) {
                int u = r.customers[k - 1];
                int v = r.customers[k];
                if (u < 0 || u >= n || v < 0 || v >= n) 
                    continue;
                pheromone[u][v] += depositMu;
                pheromone[v][u] += depositMu;
            }
        }
    }

    // 4) Deposit for the global best‐so‐far solution (lexicographic best)
    if (!bestSolution.empty()) {
        // Compute the global best's fitness
        SolutionScore &g = bestScore;
        double bestFitness = g.numVehicles * vehiclePenalty + g.totalDistance;
        if (bestFitness > 0) {
            double depositStar = b_factor * (1.0 / bestFitness);
            for (const Route &r : bestSolution) {
                for (size_t k = 1; k < r.customers.size(); ++k) {
                    int u = r.customers[k - 1];
                    int v = r.customers[k];
                    if (u < 0 || u >= n || v < 0 || v >= n) 
                        continue;
                    pheromone[u][v] += depositStar;
                    pheromone[v][u] += depositStar;
                }
            }
        }
    }
}

void reset_pheromone(double baseValue = 1.0) {
    int n = customers.size();
    // Simply set every pheromone[i][j] = baseValue.
    // If you want a slight bias on the global best edges, you can do that after.
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            pheromone[i][j] = baseValue;
        }
    }
}
int main(int argc, char* argv[]) {
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <input_file> [max_time_sec] [max_evals] \n";
        return 1;
    }

    string inputFile = argv[1];
    maxTimeSeconds = stoi(argv[2]);
    maxEvaluations = stoi(argv[3]);

    read_input(inputFile);
    
    int    resetThreshold= 50;    // If no improvement in 20 iterations → reset

    int n = customers.size();
    dist.resize(n, vector<double>(n));
    travel_time.resize(n, vector<double>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            dist[i][j] = euclidean(customers[i], customers[j]);
            travel_time[i][j] = dist[i][j];
        }
    }

    // 1) Running NN first, so we have a baseline solution
    vector<Route> nnRoutes;
    double nnDist = nearest_neighbor_solution(nnRoutes);
    int nnVehicles = (int)nnRoutes.size();
    SolutionScore bestScore = { nnVehicles, nnDist };
    bestSolution = nnRoutes;
    bestDistance = nnDist;

    // Printing the NN baseline:
    cout << "NN heuristic distance = " << nnDist
         << "  with " << nnRoutes.size() << " vehicles\n";
    
    // 2) Initialize ACO’s bestDistance to the NN cost (so ACO only tries to beat it)
    bestScore.numVehicles = nnRoutes.size();
    bestScore.totalDistance = nnDist;
    bestSolution = nnRoutes;

    // No NN used — start with worst possible score
    // SolutionScore bestScore = { INT_MAX, numeric_limits<double>::infinity() };
    // vector<Route> bestSolution;




    init_pheromone();

    srand(time(0));
    // srand(42); // For reproducibility

    vector<vector<Route>> allAntRoutes(numAnts);
    vector<SolutionScore> allScores(numAnts);
    // vector<Route> bestSolution;
    // double bestDistance = numeric_limits<double>::infinity();

    // 3) ACO loop, with a “no‐improve” counter
    int noImproveCount = 0;
    double lastBest = bestDistance;

    int evals = 0;
    auto startTime = chrono::steady_clock::now();
    
    int bestAntId = -1;
    for (int iter = 0; iter < maxIterations; ++iter) {
        for (int k = 0; k < numAnts; ++k) {
            if (evals >= maxEvaluations && maxEvaluations > 0) break;

            allScores[k] = build_solution(k, allAntRoutes[k]);
                ++evals;

        // update best if this ant actually built something
        if (!allAntRoutes[k].empty() && allScores[k] < bestScore) {
                bestScore = allScores[k];
                bestSolution = allAntRoutes[k];
                bestAntId = k;
            }
        
                // b) Check if global best improved
        if (bestDistance < lastBest) {
            // We got a strict improvement this iteration
            lastBest = bestDistance;
            noImproveCount = 0;       // reset the “no‐improve” counter
        } else {
            // No improvement on global best this iteration
            noImproveCount++;
        }

        // c) If no improvement for resetThreshold consecutive iters → reset
        // if (noImproveCount >= resetThreshold) {
        //     cout << "→ No improvement for " << resetThreshold 
        //          << " iters; resetting pheromone to uniform.\n";
        //     reset_pheromone(1.0);
        //     noImproveCount = 0;      // start counting afresh
        //     resetCountdown = RESET_ITER;   // enter reset-mode for the next few iterations
        // }
        //  (d) Adjust β if we’re in reset-mode
        // if (resetCountdown > 0) {
        //     cout << "→ Reset-mode active: β = " << BETA_LOW << "\n";
        //     beta = BETA_LOW;
        //     resetCountdown--;
        //     if (resetCountdown == 0) {
        //         cout << "→ Exiting reset-mode, restoring β = " << BETA_High << "\n";
        //         beta = BETA_High;
        //     }
        // } else {
        //     beta = BETA_High;
        // }


            
        // only break on time‐limit if maxTimeSeconds > 0
        if (maxTimeSeconds > 0) {
                auto now = chrono::steady_clock::now();
                int elapsedSec = chrono::duration_cast<chrono::seconds>(now - startTime).count();
                if (elapsedSec >= maxTimeSeconds)
                    break;
            }
        }

        // after all ants, also respect both limits before updating pheromone
        bool stopByEvals = (maxEvaluations > 0 && evals >= maxEvaluations);
        bool stopByTime  = false;
        auto now = chrono::steady_clock::now();
        int elapsedSec = chrono::duration_cast<chrono::seconds>(now - startTime).count();
        stopByTime = (elapsedSec >= maxTimeSeconds);

        if (stopByEvals || stopByTime) {
            cout << "Stopping early: "
                << (stopByEvals ? "max evaluations" : "time limit")
                << " reached.\n";
            break;
        }
        // update_pheromone(allAntRoutes, allDistances);
        update_pheromone_ranked(
            allAntRoutes,
            allScores,
            rho,
            sigma,
            a_factor,
            b_factor,
            vehiclePenalty,
            bestScore
        );        
        cout << "Iter " << iter + 1 << " | Best Distance = " << bestScore.totalDistance
             <<"| Best Vehicles = " << bestScore.numVehicles 
             << " | NoImprove = " << noImproveCount 
             << " | Evals = " << evals 
             << " | Time = " << elapsedSec << "s\n";
    }


cerr << "DEBUG: bestSolution has " << bestScore.numVehicles << " routes; bestDistance = "
     << bestScore.totalDistance << "\n";
// Write result to file
ofstream out(get_output_filename(inputFile));

for (size_t i = 0; i < bestSolution.size(); ++i) {
    out << "  Route " << i+1 << ": ";
    const vector<int>& custs = bestSolution[i].customers;
    for (size_t j= 1 ; j + 1 < custs.size(); ++j){
        out << custs[j] << " ";
}
    out << "(demand = " << bestSolution[i].total_demand << ")\n";
}
out << "Vehicles: " << bestScore.numVehicles << "\n";
out << "Distance: " << bestScore.totalDistance << "\n";
out.close();


cout << "Best solution found by Ant #" << bestAntId
     << " | Vehicles = " << bestScore.numVehicles
     << " | Distance = " << bestScore.totalDistance << "\n";

// Print summary to console
// cout << "\n✅ Final Solution Summary:\n";
// cout << "  Vehicles used: " << bestScore.numVehicles << "\n";
// cout << "  Total distance: " << bestScore.totalDistance << "\n\n";

// Validate
Solution sol = bestSolution;
cout << validate_solution(sol) << endl;

    return 0;
}