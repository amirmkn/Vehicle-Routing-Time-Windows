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




// Convert Individual to Solution (vector<Route>)
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
        // 🚫 Hard reject if more than allowed vehicles
        if (ind.routes.size() > M) {
            logFile << "[Reject] Too many vehicles: " << ind.routes.size() << " > " << M << "\n";
            return numeric_limits<double>::max();
        }
    double totalDistance = 0.0;

    for (const auto& route : ind.routes) {
        if (route.empty()) continue;

        double departure_time = 0.0;
        double arrival_time = 0.0;
        int load = 0;
        double route_distance = 0.0;

        int prev = 0;  // start from depot

        for (int cid : route) {
            const Customer& curr = customers[cid];

            // Travel from previous to current
            double travel = euclidean_distance(customers[prev], curr);

            // Arrival at current = max(depart prev + travel, earliest)
            arrival_time = max(departure_time + travel, (double)curr.earliest);
            if (arrival_time > curr.latest) {
                errorLog << "Time window violation at customer " << cid
                        << ", arrival: " << arrival_time
                        << ", window: [" << curr.earliest << ", " << curr.latest << "]\n";
                return numeric_limits<double>::max();
            }

            // Departure from current = arrival + service
            departure_time = arrival_time + curr.service_time;

            // Load update and capacity check
            load += curr.demand;
            if (load > vehicleCapacity) {
                errorLog << "Capacity violation on route, load: " << load << " > " << vehicleCapacity << "\n";
                return numeric_limits<double>::max();
            }

            route_distance += travel;
            prev = cid;
        }

        // Return to depot from last customer
        double return_travel = euclidean_distance(customers[prev], customers[0]);
        departure_time += return_travel;

        if (departure_time > customers[0].latest) {
            errorLog << "Depot return too late. Return time: " << departure_time
                    << ", latest allowed: " << customers[0].latest << "\n";
            return numeric_limits<double>::max();
        }

        route_distance += return_travel;
        totalDistance += route_distance;

        double fitness = ind.routes.size() * 1e6 + totalDistance;  // prioritize fewer vehicles
        logFile << "[Eval] Vehicles: " << ind.routes.size() << ", Distance: " << totalDistance
                << ", Fitness: " << fitness << "\n";
    }

    errorLog << "Evaluation " << evaluationCount << " succeeded. Total route distance: " << totalDistance << "\n";
    return totalDistance;
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


// Individual create_random_individual(mt19937& rng) {
//     vector<int> cust_ids;
//     for (size_t i = 1; i < customers.size(); ++i)
//         cust_ids.push_back(i);

//     shuffle(cust_ids.begin(), cust_ids.end(), rng);

//     Individual ind;
//     vector<int> current_route;
//     int current_load = 0;
//     double departure_time = 0.0;
//     int prev = 0; // start from depot

//     for (int cid : cust_ids) {
//         const Customer& cust = customers[cid];

//         // Estimate arrival and departure times
//         double travel = euclidean_distance(customers[prev], cust);
//         double arrival = max(departure_time + travel, (double)cust.earliest);
//         double leave = arrival + cust.service_time;
//         int load_after = current_load + cust.demand;

//         bool feasible = (arrival <= cust.latest) && (load_after <= vehicleCapacity);

//         if (!feasible) {
//             // Finish current route and start a new one
//             if (!current_route.empty())
//                 ind.routes.push_back(current_route);

//             current_route.clear();
//             current_route.push_back(cid);
//             current_load = cust.demand;
//             departure_time = max(euclidean_distance(customers[0], cust), (double)cust.earliest) + cust.service_time;
//             prev = cid;
//         } else {
//             current_route.push_back(cid);
//             departure_time = leave;
//             current_load = load_after;
//             prev = cid;
//         }
//     }

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

Individual crossover(const Individual& p1, const Individual& p2, mt19937& rng) {
    Chromosome c = order_crossover(p1.chromosome, p2.chromosome, rng);
    Individual child;
    child.chromosome = c;
    child.routes     = decode_chromosome(c);
    child.fitness    = evaluate(child);
    return child;
}

// Individual crossover(const Individual& p1, const Individual& p2, mt19937& rng) {
//     Individual child;
//     unordered_set<int> used;
//     for (const auto& route : p1.routes) {
//         for (int cid : route) used.insert(cid);
//         child.routes.push_back(route);
//         if (rand() % 2 == 0) break;
//     }
//     for (const auto& route : p2.routes) {
//         vector<int> newRoute;
//         for (int cid : route) {
//             if (!used.count(cid)) {
//                 newRoute.push_back(cid);
//                 used.insert(cid);
//             }
//         }
//         if (!newRoute.empty()) child.routes.push_back(newRoute);
//     }
//     child.fitness = evaluate(child);
//     return child;
// }

// void mutate(Individual& ind, mt19937& rng) {
//     uniform_int_distribution<> dist(0, ind.routes.size() - 1);
//     int i = dist(rng);
//     if (ind.routes[i].size() < 2) return;
//     uniform_int_distribution<> dist2(0, ind.routes[i].size() - 1);
//     int a = dist2(rng), b = dist2(rng);
//     swap(ind.routes[i][a], ind.routes[i][b]);
//     ind.fitness = evaluate(ind);
// }

void mutate(Individual& ind, mt19937& rng) {
    uniform_real_distribution<> prob(0,1);
    if (prob(rng) < 0.2) {
        int n = ind.chromosome.size();
        uniform_int_distribution<> d(0, n-1);
        int i = d(rng), j = d(rng);
        swap(ind.chromosome[i], ind.chromosome[j]);
        ind.routes  = decode_chromosome(ind.chromosome);
        ind.fitness = evaluate(ind);
    }
}


void write_to_file(const Individual& ind, const string& inputFilePath) {
    ofstream out(get_output_filename(inputFilePath));
    out << "Total Distance: " << ind.fitness << "\n";
    out << "Vehicle Count: " << ind.routes.size() << "\n";
    for (const auto& route : ind.routes) {
        out << "0 ";
        for (int cid : route) out << cid << " ";
        out << "0\n";
    }
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
        cerr << "Usage: " << argv[0] << " <input_file> <max_time> <max_evaluations>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    maxTime = atoi(argv[2]);
    maxEvaluations = atoi(argv[3]);

    read_input(inputFile);
    // --- after read_input(inputFile); ---
    int n = customers.size();
    dist.resize(n, vector<double>(n));
    travel_time.resize(n, vector<double>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double d = euclidean_distance(customers[i], customers[j]);
            dist[i][j] = d;
            travel_time[i][j] = d;  // or if you have a separate travel_time, compute accordingly
        }
}
    startTime = clock();
    mt19937 rng(time(0));

    const int populationSize = 50;
    vector<Individual> population;
    for (int i = 0; i < populationSize; ++i) {
        // Individual ind = create_random_individual(rng);

        // 1) generate the flat-chromosome
        Chromosome chrom = solomon_initialization(customers, Q);

        // 2) build an Individual from it
        Individual ind;
        ind.chromosome = chrom;                          // store the permutation
        ind.routes     = decode_chromosome(chrom);       // segment into routes
        ind.fitness    = evaluate(ind);                  // compute fitness

population.push_back(ind);
        population.push_back(ind);
        if (ind.fitness < bestFitness) {
            bestFitness = ind.fitness;
            bestIndividual = ind;
            logFile << "[Initial best] Distance: " << bestIndividual.fitness
        << ", Routes: " << bestIndividual.routes.size() << "\n";
        }
    }

    while (!time_or_eval_exceeded()) {
        logFile << "\n--- New Generation ---\n";
            
        // Elitism: preserve top N best individuals
    std::sort(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
        return a.fitness < b.fitness;  // lower fitness = better
    });

    int eliteCount = std::max(1, populationSize / 10); // top 10%
    vector<Individual> newPop;
    for (int i = 0; i < eliteCount; ++i) {
        newPop.push_back(population[i]);
        logFile << "[Elitism] Preserved individual " << i + 1
                << ", fitness: " << population[i].fitness << "\n";
    }
    // Roulette Wheel Selection

    auto roulette_select = [&](mt19937& rng, const vector<Individual>& pop) {
        double fitness_sum = 0.0;
        for (const auto& ind : pop) fitness_sum += 1.0 / (ind.fitness + 1e-6);  // inverse for minimization

        double pick = uniform_real_distribution<>(0.0, fitness_sum)(rng);
        double current = 0.0;

        for (const auto& ind : pop) {
            current += 1.0 / (ind.fitness + 1e-6);
            if (current >= pick) return ind;
        }
        return pop.back();  // fallback
};


        // vector<Individual> newPop;
        while (newPop.size() < populationSize) {
            uniform_int_distribution<> dist(0, populationSize - 1);
            // Individual p1 = population[dist(rng)];
            // Individual p2 = population[dist(rng)];
            
            // Tournament selection
            auto tournament_select = [&](int k) {
                Individual best;
                bool initialized = false;
                for (int i = 0; i < k; ++i) {
                    int idx = dist(rng);
                    if (!initialized || population[idx].fitness < best.fitness) {
                        best = population[idx];
                        initialized = true;
                    }
                }
                return best;
            };
            
            // Individual p1 = tournament_select(3);
            // Individual p2 = tournament_select(3);
            Individual p1 = roulette_select(rng, population);
            Individual p2 = roulette_select(rng, population);
            // logFile << "[Selection] p1 fitness: " << p1.fitness << ", p2 fitness: " << p2.fitness << "\n";
            logFile << "Parent 1 fitness: " << p1.fitness << ", routes: " << p1.routes.size() << "\n";
            logFile << "Parent 2 fitness: " << p2.fitness << ", routes: " << p2.routes.size() << "\n";


            Individual child = crossover(p1, p2, rng);
            if (rand() % 100 < 20) {
                mutate(child, rng);
                logFile << "Mutation applied\n";
        } else {
    logFile << "Mutation skipped\n";
}
/////// ----------- ADDED DETAILED LOG ----------------- ////////
logFile << "Child fitness: " << child.fitness << ", routes: " << child.routes.size() << "\n";

int route_idx = 1;
for (const auto& route : child.routes) {
    logFile << "  Route " << route_idx++ << ": 0 ";
    for (int cid : route) logFile << cid << " ";
    logFile << "0\n";

    double time = 0.0, departure = 0.0;
    int prev = 0;
    for (int cid : route) {
        const Customer& cust = customers[cid];
        double travel = euclidean_distance(customers[prev], cust);
        double arrival = max(departure + travel, (double)cust.earliest);
        departure = arrival + cust.service_time;
        logFile << "    Customer " << cid << ": arrival = " << arrival
                << ", departure = " << departure << ", window = ["
                << cust.earliest << ", " << cust.latest << "]\n";
        prev = cid;
    }
    double return_time = departure + euclidean_distance(customers[prev], customers[0]);
    logFile << "    Return to depot at time " << return_time
            << ", latest allowed: " << customers[0].latest << "\n";
}

            newPop.push_back(child);
            if (child.fitness < bestFitness) {
                logFile << "Child became new best solution.\n";
                logFile << "[New best] Distance improved: " << bestFitness
                << " -> " << child.fitness
                << ", Routes: " << bestIndividual.routes.size()
                << " -> " << child.routes.size() << "\n";
                bestFitness = child.fitness;
                bestIndividual = child;
            }
        }
        population = newPop;
    }
    
    errorLog << "Best distance: " << bestIndividual.fitness
    << ", vehicles used: " << bestIndividual.routes.size() << "\n";
    errorLog.close();
    // write_to_file(bestIndividual, inputFile);
        // Ensure the best individual's routes are decoded
        bestIndividual.routes = decode_chromosome(bestIndividual.chromosome);

        // Write to file as before
        write_to_file(bestIndividual, inputFile);
    
        // Build the Solution object and validate
        Solution sol = convert_to_solution(bestIndividual);
        cout << validate_solution(sol);
        return 0;
    }
