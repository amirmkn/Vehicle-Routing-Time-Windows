#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ctime>
#include <random>
#include <limits>
#include <unordered_set>
#include "validation.h"
using namespace std;
using Chromosome = std::vector<int>;  // A flat permutation of customer indices


ofstream errorLog("error_log.txt");
ofstream logFile("global_log.txt");

struct Individual {
    Chromosome chromosome;
    vector<vector<int>> routes;
    double fitness;
};

int DEPOT = 0;
double Q; // max capacity of each vehicle
vector<vector<double>> dist;
vector<vector<double>> travel_time;
int M;  // max allowed vehicles, from input file



vector<Customer> customers;
int vehicleCapacity =0;
double bestFitness = numeric_limits<double>::max();
Individual bestIndividual;
int maxTime = 0, maxEvaluations = 0;
int evaluationCount = 0;
clock_t startTime;




// Convert Individual to Solution (vector<Route>) in order to use the validation function we had
Solution convert_to_solution(const Individual& ind) {
    Solution sol;
    for (const auto& vec : ind.routes) {
        Route r;
        r.customers.push_back(0);  // start at depot
        r.customers.insert(r.customers.end(), vec.begin(), vec.end());
        r.customers.push_back(0);  // return to depot

        r.total_demand = 0;
        for (int cid : vec) {
            r.total_demand += customers[cid].demand;
        }

        sol.push_back(r);
    }
    return sol;
}
string get_output_filename(const string& inputPath) {
    size_t slash = inputPath.find_last_of("/");
    string filename = (slash == string::npos) ? inputPath : inputPath.substr(slash + 1);
    size_t dot = filename.find_last_of(".");
    if (dot != string::npos) filename = filename.substr(0, dot);
    return filename + "_output.txt";
}
bool is_feasible(const vector<int>& route) {
    double time = 0.0;
    int load = 0;

    int prev = DEPOT;  // Start at depot (usually index 0)

    for (int cid : route) {
        const Customer& curr = customers[cid];
        time += travel_time[prev][cid];
        time = max(time, static_cast<double>(curr.earliest));

        if (time > curr.latest)
            return false;

        time += curr.service_time;
        load += curr.demand;

        if (load > Q)
            return false;

        prev = cid;
    }

    // Return to depot
    time += travel_time[prev][DEPOT];
    if (time > customers[DEPOT].latest)
        return false;

    return true;
}

// void try_merge_routes(Individual& ind) {
//     for (size_t i = 0; i < ind.routes.size(); ++i) {
//         for (size_t j = i + 1; j < ind.routes.size();) {
//             vector<int> merged = ind.routes[i];
//             merged.insert(merged.end(), ind.routes[j].begin(), ind.routes[j].end());

//             if (is_feasible(merged)) {
//                 ind.routes[i] = merged;
//                 ind.routes.erase(ind.routes.begin() + j);
//             } else {
//                 ++j;
//             }
//         }
//     }
// }





bool is_time_exceeded() {
    if (maxTime == 0) return false;
    double elapsed = double(clock() - startTime) / CLOCKS_PER_SEC;
    return elapsed >= maxTime;
}

bool is_evaluation_exceeded() {
    if (maxEvaluations == 0) return false;
    return evaluationCount >= maxEvaluations;
}

bool time_or_eval_exceeded() {
    return is_time_exceeded() || is_evaluation_exceeded();
}

double euclidean_distance(const Customer& a, const Customer& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Attempt to merge route j into route i at the best insertion point.
// Returns true if merge succeeded.
bool try_merge_pair(vector<int>& base, vector<int>& addon) {
    int n = base.size(), m = addon.size();
    double bestDelta = numeric_limits<double>::infinity();
    vector<int> bestMerged;

    // Try inserting the entire 'addon' block at each position in 'base'
    for (int pos = 0; pos <= n; ++pos) {
        vector<int> merged;
        merged.reserve(n + m);
        merged.insert(merged.end(), base.begin(), base.begin() + pos);
        merged.insert(merged.end(), addon.begin(), addon.end());
        merged.insert(merged.end(), base.begin() + pos, base.end());

        if (!is_feasible(merged)) continue;

        // compute distance delta
        auto distOf = [&](const vector<int>& route) {
            double d = 0;
            int prev = DEPOT;
            for (int cid : route) {
                d += travel_time[prev][cid];
                prev = cid;
            }
            d += travel_time[prev][DEPOT];
            return d;
        };
        double d0 = distOf(base), d1 = distOf(merged);
        double delta = d1 - d0;
        if (delta < bestDelta) {
            bestDelta = delta;
            bestMerged = std::move(merged);
        }
    }

    if (bestDelta < numeric_limits<double>::infinity()) {
        base = std::move(bestMerged);
        return true;
    }
    return false;
}

// Aggressive merge: for every pair (i,j), try both directions,
// pick any successful merge and restart scan.
void try_merge_routes(Individual& ind) {
    bool merged = true;
    while (merged) {
        if (time_or_eval_exceeded()) break;   // <-- time guard
        merged = false;
        int R = ind.routes.size();
        for (int i = 0; i < R && !merged; ++i) {
            for (int j = i+1; j < R; ++j) {
                if (time_or_eval_exceeded()) break;
                if (try_merge_pair(ind.routes[i], ind.routes[j])) {
                    ind.routes.erase(ind.routes.begin()+j);
                    merged = true;
                    break;
                }
                if (try_merge_pair(ind.routes[j], ind.routes[i])) {
                    ind.routes.erase(ind.routes.begin()+i);
                    merged = true;
                    break;
                }
            }
        }
    }
}


// Compute the total distance of a depot-start/end route
static double route_distance(const vector<int>& route) {
    double d = 0;
    int prev = DEPOT;
    for (int cid : route) {
        d += travel_time[prev][cid];
        prev = cid;
    }
    d += travel_time[prev][DEPOT];
    return d;
}

void rebalance_smallest_route(Individual& ind) {
    if (ind.routes.size() < 2) return;

    // 1) Find index of the route with the fewest customers
    size_t minIdx = 0;
    for (size_t i = 1; i < ind.routes.size(); ++i) {
        if (ind.routes[i].size() < ind.routes[minIdx].size()) {
            minIdx = i;
        }
    }
    auto smallRoute = ind.routes[minIdx];

    // 2) Try to insert each customer from smallRoute into another route
    struct Insertion { size_t route, pos; double delta; };
    vector<vector<int>>& R = ind.routes;

    for (int cid : smallRoute) {
        // Check time/eval limit before each customer
        if (time_or_eval_exceeded()) return;

        Insertion best{0, 0, std::numeric_limits<double>::infinity()};
        bool found = false;

        for (size_t j = 0; j < R.size(); ++j) {
            if (j == minIdx) continue;

            // Check again inside inner loop
            if (time_or_eval_exceeded()) return;

            auto& route = R[j];
            for (size_t k = 0; k <= route.size(); ++k) {
                if (time_or_eval_exceeded()) return;

                vector<int> trial = route;
                trial.insert(trial.begin() + k, cid);
                if (!is_feasible(trial)) continue;

                double before = route_distance(route);
                double after  = route_distance(trial);
                double delta  = after - before;
                if (delta < best.delta) {
                    best = {j, k, delta};
                    found = true;
                }
            }
        }

        if (!found) {
            // Abort if we can't reassign this customer
            return;
        }

        // Perform the best insertion
        R[best.route].insert(R[best.route].begin() + best.pos, cid);
    }

    // 3) All customers moved successfully → remove the now-empty route
    ind.routes.erase(ind.routes.begin() + minIdx);
}




// --------------- DECODER ------------------ //
// Decode a chromosome into an Individual
vector<vector<int>> decode_chromosome(const Chromosome& chromosome) {
    vector<vector<int>> routes;
    vector<int> current_route;
    int    current_load   = 0;
    double departure_time = 0.0;
    int    prev           = DEPOT;

    for (int cid : chromosome) {
        const Customer& cust = customers[cid];

        double travel    = euclidean_distance(customers[prev], cust);
        double arrival   = max(departure_time + travel, (double)cust.earliest);
        double leave     = arrival + cust.service_time;
        int    load_after= current_load + cust.demand;

        bool fitsCap = (load_after <= vehicleCapacity);
        bool fitsTW  = (arrival    <= cust.latest);

        if (!fitsCap || !fitsTW) {
            if (!current_route.empty())
                routes.push_back(current_route);
            current_route.clear();
            current_route.push_back(cid);
            current_load     = cust.demand;
            departure_time   = max(
                                  euclidean_distance(customers[DEPOT], cust),
                                  (double)cust.earliest
                              ) + cust.service_time;
        }
        else {
            current_route.push_back(cid);
            current_load     = load_after;
            departure_time   = leave;
        }
        prev = cid;
    }

    if (!current_route.empty())
        routes.push_back(current_route);

    return routes;
}

// ------------- EVALUATOR ----------------- //
// Evaluate an Individual
double evaluate(const Individual& ind) {
    evaluationCount++;
    double totalDistance = 0.0;
    double penalty = 0.0;

    const double vehiclePenalty     = 10000.0;
    const double capacityPenalty    = 100.0;
    const double timeWindowPenalty  = 100.0;
    const double depotLatePenalty   = 100.0;

    if (ind.routes.size() > M) {
        penalty += (ind.routes.size() - M) * vehiclePenalty;
        logFile << "[Penalty] Extra vehicles used: " << ind.routes.size() - M << "\n";
    }

    for (const auto& route : ind.routes) {
        if (route.empty()) continue;

        double departure_time = 0.0;
        double arrival_time = 0.0;
        int load = 0;
        double route_distance = 0.0;
        int prev = 0;

        for (int cid : route) {
            const Customer& curr = customers[cid];
            double travel = euclidean_distance(customers[prev], curr);

            arrival_time = max(departure_time + travel, (double)curr.earliest);
            if (arrival_time > curr.latest) {
                double late = arrival_time - curr.latest;
                penalty += late * timeWindowPenalty;
                errorLog << "[TW Violation] Customer " << cid << ": late by " << late << "\n";
            }

            departure_time = arrival_time + curr.service_time;
            load += curr.demand;
            if (load > vehicleCapacity) {
                penalty += (load - vehicleCapacity) * capacityPenalty;
                errorLog << "[Capacity Violation] Load: " << load << ", cap: " << vehicleCapacity << "\n";
            }

            route_distance += travel;
            prev = cid;
        }

        // Return to depot
        double return_travel = euclidean_distance(customers[prev], customers[0]);
        departure_time += return_travel;
        if (departure_time > customers[0].latest) {
            double lateDepot = departure_time - customers[0].latest;
            penalty += lateDepot * depotLatePenalty;
            errorLog << "[Depot Late Return] Late by " << lateDepot << "\n";
        }

        route_distance += return_travel;
        totalDistance += route_distance;
    }

    double fitness = totalDistance + penalty;
    logFile << "[Eval] Vehicles: " << ind.routes.size() << ", Distance: " << totalDistance
            << ", Penalty: " << penalty << ", Fitness: " << fitness << "\n";

    return fitness;
}


// ------------- RANDOM individual ----------------- //
// Create a random individual by shuffling customer indices
Individual create_random_individual(mt19937& rng) {
    // 1. Build flat permutation
    Chromosome c;
    for (int i = 1; i < (int)customers.size(); ++i) c.push_back(i);
    shuffle(c.begin(), c.end(), rng);

    // 2. Decode & evaluate
    Individual ind;
    ind.chromosome = c;
    ind.routes     = decode_chromosome(c);
    ind.fitness    = evaluate(ind);
    return ind;
}


//--------- SOLOMON INITIALIZATION -----------//
vector<int> solomon_initialization(const vector<Customer>& customers, int vehicle_capacity) {
    vector<int> chromosome;
    vector<bool> visited(customers.size(), false);
    visited[DEPOT] = true;

    while (true) {
        vector<int> route;
        int    load           = 0;
        double current_time   = 0;
        int    current_cust   = DEPOT;

        while (true) {
            int    next = -1;
            double best = numeric_limits<double>::max();

            for (int i = 1; i < (int)customers.size(); ++i) {
                if (visited[i])                         continue;
                if (load + customers[i].demand > vehicle_capacity) continue;

                double travel  = euclidean_distance(customers[current_cust], customers[i]);
                double arrival = max(current_time + travel, (double)customers[i].earliest);
                if (arrival > customers[i].latest)      continue;

                double wait = max(0.0, customers[i].earliest - arrival);
                double score = arrival + wait;  // you can weight distance vs. wait

                if (score < best) {
                    best = score;
                    next = i;
                }
            }
            if (next < 0) break;

            visited[next] = true;
            route.push_back(next);
            double travel = euclidean_distance(customers[current_cust], customers[next]);
            current_time  = max(current_time + travel, (double)customers[next].earliest)
                          + customers[next].service_time;
            load         += customers[next].demand;
            current_cust  = next;
        }

        if (route.empty()) break;
        chromosome.insert(chromosome.end(), route.begin(), route.end());
    }
    return chromosome;
}


// ------- Create a mixed initial population: AMALGAM ----------
using Population = std::vector<Individual>;

// Create a mixed initial population:
//  - `popSize` total individuals
//  - `solomonCount` of them from the Solomon heuristic
//  - the rest purely random
Population initialize_population(int popSize,
                                 int solomonCount,
                                 mt19937& rng) {
    Population pop;
    pop.reserve(popSize);

    int randomCount = popSize - solomonCount;

    // 1) Solomon‐seeded individuals
    for (int i = 0; i < solomonCount; ++i) {
        Chromosome chrom = solomon_initialization(customers, vehicleCapacity);
        Individual ind;
        ind.chromosome = chrom;
        ind.routes     = decode_chromosome(chrom);
        ind.fitness    = evaluate(ind);
        pop.push_back(ind);
    }

    // 2) Purely random individuals
    for (int i = 0; i < randomCount; ++i) {
        Individual ind = create_random_individual(rng);
        pop.push_back(ind);
    }

    return pop;
}




Chromosome order_crossover(const Chromosome& A, const Chromosome& B, mt19937& rng) {
    int n = A.size();
    uniform_int_distribution<> d(0, n-1);
    int i = d(rng), j = d(rng);
    if (i>j) swap(i,j);

    // child initialized with “holes”
    Chromosome child(n, -1);
    // copy A[i..j]
    for (int k = i; k <= j; ++k) child[k] = A[k];

    // fill rest from B in order
    int pos = (j+1)%n;
    for (int k = 0; k < n; ++k) {
        int idx = (j+1 + k) % n;
        int gene = B[idx];
        if (find(child.begin()+i, child.begin()+j+1, gene) == child.begin()+j+1) {
            child[pos] = gene;
            pos = (pos+1)%n;
        }
    }
    return child;
}

Individual crossover(
    const Individual& p1,
    const Individual& p2,
    std::mt19937& rng,
    double cx_rate
) {
    std::uniform_real_distribution<> prob(0.0, 1.0);
    Chromosome childChrom;

    if (prob(rng) < cx_rate) {
        // perform Order-Crossover
        childChrom = order_crossover(p1.chromosome, p2.chromosome, rng);
    } else {
        // no crossover → copy parent1
        childChrom = p1.chromosome;
    }

    Individual child;
    child.chromosome = std::move(childChrom);
    child.routes     = decode_chromosome(child.chromosome);
    child.fitness    = evaluate(child);
    return child;
}

// -------------- Mutation ----------------

void mutate(
    Individual& ind,
    std::mt19937& rng,
    double mut_rate
) {
    std::uniform_real_distribution<> prob(0.0, 1.0);
    if (prob(rng) < mut_rate) {
        int n = ind.chromosome.size();
        std::uniform_int_distribution<> d(0, n - 1);
        int i = d(rng), j = d(rng);
        std::swap(ind.chromosome[i], ind.chromosome[j]);
        ind.routes  = decode_chromosome(ind.chromosome);
        ind.fitness = evaluate(ind);
    }
}

// ------------------ INVERSION MUTATION ------------------------- //
void inversion_mutation(Individual& ind, mt19937& rng, double mutation_rate) {
    uniform_real_distribution<> prob(0.0, 1.0);
    if (prob(rng) < mutation_rate) {
        int n = ind.chromosome.size();
        if (n < 2) return; // no meaningful inversion possible

        uniform_int_distribution<> dist(0, n - 1);
        int i = dist(rng);
        int j = dist(rng);

        if (i > j) swap(i, j);
        if (i == j) return; // same point, skip

        reverse(ind.chromosome.begin() + i, ind.chromosome.begin() + j + 1);

        // Update the individual's routes and fitness
        ind.routes = decode_chromosome(ind.chromosome);
        ind.fitness = evaluate(ind);
    }
}


void write_to_file(const Individual& ind, const string& inputFilePath) {
    ofstream out(get_output_filename(inputFilePath));

    for (size_t i = 0; i < ind.routes.size(); ++i) {
        out << "Route " << (i + 1) << ": ";
        for (int cid : ind.routes[i]) {
            out << cid << " ";
        }
        out << "\n";
    }

    out << "\nVehicles: " << ind.routes.size() << "\n";
    out << "Distance: " << fixed << setprecision(2) << ind.fitness << "\n";

    out.close();
}


// ------------- INPUT READING ----------------- //
// Read input from file
void read_input(const string& filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file " << filename << "\n";
        exit(1);
    }

    string line;
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            getline(infile, line);
            istringstream vehicle_line(line);
            vehicle_line >> M >> vehicleCapacity;
            Q = vehicleCapacity;
            M = M;

        }
        if (line.find("CUSTOMER") != string::npos)
            break;
    }

    getline(infile, line); // Skip header
    getline(infile, line); // Skip another header line

    customers.clear();
    while (getline(infile, line)) {
        if (line.empty())
            continue;
        istringstream iss(line);
        Customer c;
        iss >> c.id >> c.x >> c.y >> c.demand >> c.earliest >> c.latest >> c.service_time;
        customers.push_back(c);
    }

    infile.close();
    cout << "Input file read successfully." << endl;
    cout << "Number of customers: " << customers.size()- 1 << endl;
    cout << "Vehicle capacity: " << vehicleCapacity << endl;
    cout << "Number of vehicles: " << M << endl;
}




int main(int argc, char* argv[]) {
    logFile << "Starting Genetic Algorithm...\n";
    
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <input_file> <max_time> <max_evaluations>\n";
        return 1;
    }

    string inputFile = argv[1];
    maxTime = atoi(argv[2]);
    maxEvaluations = atoi(argv[3]);

    read_input(inputFile);

    // Initialize distance and travel time matrices
    int n = customers.size();
    dist.resize(n, vector<double>(n));
    travel_time.resize(n, vector<double>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double d = euclidean_distance(customers[i], customers[j]);
            dist[i][j] = d;
            travel_time[i][j] = d;
        }
    }

    startTime = clock();
    mt19937 rng(time(0));

    // GA parameters
    int populationSize  = 1000;
    int solomonSeeds    = 300;
    double crossover_rate = 0.8;
    double mutation_rate = 0.1;
    double elite_rate     = 0.10;
    double selection_rate = 0.90;
    int eliteCount        = max(1, int(ceil(elite_rate * populationSize)));
    int replaceCount      = int(round((1.0 - elite_rate) * populationSize));

    uniform_real_distribution<> uni(0.0, 1.0);
    auto population = initialize_population(populationSize, solomonSeeds, rng);

    bestFitness = numeric_limits<double>::max();
    for (const auto& ind : population) {
        if (ind.fitness < bestFitness) {
            bestFitness = ind.fitness;
            bestIndividual = ind;
            logFile << "[Initial best] Distance: " << bestFitness
                    << ", vehicles used: " << bestIndividual.routes.size() << "\n";
        }
    }

    while (!time_or_eval_exceeded()) {
        logFile << "\n--- New Generation ---\n";

        // Sort population and preserve elites
        sort(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
            return a.fitness < b.fitness;
        });

        vector<Individual> newPop;
        for (int i = 0; i < eliteCount; ++i) {
            newPop.push_back(population[i]);
            logFile << "[Elitism] Preserved individual " << i + 1
                    << ", fitness: " << population[i].fitness << "\n";
        }

        // Selection helpers
        auto roulette_select = [&](mt19937& rng, const vector<Individual>& pop) {
            double fitness_sum = 0.0;
            for (const auto& ind : pop) fitness_sum += 1.0 / (ind.fitness + 1e-6);

            double pick = uniform_real_distribution<>(0.0, fitness_sum)(rng);
            double current = 0.0;

            for (const auto& ind : pop) {
                current += 1.0 / (ind.fitness + 1e-6);
                if (current >= pick) return ind;
            }
            return pop.back();
        };

        while (newPop.size() < populationSize) {
            Individual p1, p2;

            if (uni(rng) < selection_rate)
                p1 = roulette_select(rng, population);
            else
                p1 = population[uniform_int_distribution<>(0, eliteCount - 1)(rng)];

            if (uni(rng) < selection_rate)
                p2 = roulette_select(rng, population);
            else
                p2 = population[uniform_int_distribution<>(0, eliteCount - 1)(rng)];

            logFile << "Parent 1 fitness: " << p1.fitness << ", routes: " << p1.routes.size() << "\n";
            logFile << "Parent 2 fitness: " << p2.fitness << ", routes: " << p2.routes.size() << "\n";

            Individual child = crossover(p1, p2, rng, crossover_rate);
            try_merge_routes(child);
            logFile << "Routes merged after crossover.\n";


            if (rand() % 100 < 20) {
                inversion_mutation(child, rng, mutation_rate);
                try_merge_routes(child);
                logFile << "Mutation applied\n";
            } else {
                logFile << "Mutation skipped\n";
            }

            logFile << "Child fitness: " << child.fitness
                    << ", vehicles used: " << child.routes.size() << "\n";

            newPop.push_back(child);

            if (child.fitness < bestFitness) {
                bestFitness = child.fitness;
                bestIndividual = child;

                logFile << "[New Best] Distance: " << bestFitness
                        << ", vehicles used: " << bestIndividual.routes.size() << "\n";
            }
        }

        population = newPop;
    }

    // Final report
    errorLog << "Best distance: " << bestIndividual.fitness
             << ", vehicles used: " << bestIndividual.routes.size() << "\n";
    cout << "Total evaluations performed: " << evaluationCount << "\n";
    errorLog << "Total evaluations performed: " << evaluationCount << "\n";
    // errorLog << "[Before rebalance] Route count: " << bestIndividual.routes.size() << "\n";
    // try_merge_routes(bestIndividual);
    // bestIndividual.routes = decode_chromosome(bestIndividual.chromosome);
    // errorLog << "[After rebalance] Route count: " << bestIndividual.routes.size() << "\n";
    errorLog.close();
    write_to_file(bestIndividual, inputFile);
    cout << "Number of vehicles used: " << bestIndividual.routes.size() << "\n";

    Solution sol = convert_to_solution(bestIndividual);
    cout << validate_solution(sol);

    return 0;
}

