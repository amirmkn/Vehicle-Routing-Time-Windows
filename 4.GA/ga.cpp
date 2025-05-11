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
#include <fstream>
using namespace std;

ofstream errorLog("error_log.txt");
ofstream logFile("global_log.txt");


struct Customer {
    int id;
    int x, y;
    int demand;
    int earliest;
    int latest;
    int service_time;
};

struct Individual {
    vector<vector<int>> routes;
    double fitness;
};

vector<Customer> customers;
int vehicleCapacity =0;
double bestFitness = numeric_limits<double>::max();
Individual bestIndividual;
int maxTime = 0, maxEvaluations = 0;
int evaluationCount = 0;
clock_t startTime;

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

double evaluate(const Individual& ind) {
    evaluationCount++;
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
    }

    errorLog << "Evaluation " << evaluationCount << " succeeded. Total route distance: " << totalDistance << "\n";
    return totalDistance;
}




// Individual create_random_individual(mt19937& rng) {
//     vector<int> cust_ids;
//     for (size_t i = 1; i < customers.size(); ++i) cust_ids.push_back(i);
//     shuffle(cust_ids.begin(), cust_ids.end(), rng);

//     Individual ind;
//     vector<int> currentRoute;
//     int currentLoad = 0;

//     for (int cid : cust_ids) {
//         const Customer& cust = customers[cid];
//         if (currentLoad + cust.demand > vehicleCapacity) {
//             ind.routes.push_back(currentRoute);
//             currentRoute.clear();
//             currentLoad = 0;
//         }
//         currentRoute.push_back(cid);
//         currentLoad += cust.demand;
//     }
//     if (!currentRoute.empty()) ind.routes.push_back(currentRoute);

//     ind.fitness = evaluate(ind);
//     return ind;
// }

Individual create_random_individual(mt19937& rng) {
    vector<int> cust_ids;
    for (size_t i = 1; i < customers.size(); ++i)
        cust_ids.push_back(i);

    shuffle(cust_ids.begin(), cust_ids.end(), rng);

    Individual ind;
    vector<int> current_route;
    int current_load = 0;
    double departure_time = 0.0;
    int prev = 0; // start from depot

    for (int cid : cust_ids) {
        const Customer& cust = customers[cid];

        // Estimate arrival and departure times
        double travel = euclidean_distance(customers[prev], cust);
        double arrival = max(departure_time + travel, (double)cust.earliest);
        double leave = arrival + cust.service_time;
        int load_after = current_load + cust.demand;

        bool feasible = (arrival <= cust.latest) && (load_after <= vehicleCapacity);

        if (!feasible) {
            // Finish current route and start a new one
            if (!current_route.empty())
                ind.routes.push_back(current_route);

            current_route.clear();
            current_route.push_back(cid);
            current_load = cust.demand;
            departure_time = max(euclidean_distance(customers[0], cust), (double)cust.earliest) + cust.service_time;
            prev = cid;
        } else {
            current_route.push_back(cid);
            departure_time = leave;
            current_load = load_after;
            prev = cid;
        }
    }

    if (!current_route.empty())
        ind.routes.push_back(current_route);

    ind.fitness = evaluate(ind);
    return ind;
}


Individual crossover(const Individual& p1, const Individual& p2, mt19937& rng) {
    Individual child;
    unordered_set<int> used;
    for (const auto& route : p1.routes) {
        for (int cid : route) used.insert(cid);
        child.routes.push_back(route);
        if (rand() % 2 == 0) break;
    }
    for (const auto& route : p2.routes) {
        vector<int> newRoute;
        for (int cid : route) {
            if (!used.count(cid)) {
                newRoute.push_back(cid);
                used.insert(cid);
            }
        }
        if (!newRoute.empty()) child.routes.push_back(newRoute);
    }
    child.fitness = evaluate(child);
    return child;
}

void mutate(Individual& ind, mt19937& rng) {
    uniform_int_distribution<> dist(0, ind.routes.size() - 1);
    int i = dist(rng);
    if (ind.routes[i].size() < 2) return;
    uniform_int_distribution<> dist2(0, ind.routes[i].size() - 1);
    int a = dist2(rng), b = dist2(rng);
    swap(ind.routes[i][a], ind.routes[i][b]);
    ind.fitness = evaluate(ind);
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

void read_input(const string& filename) {
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file " << filename << "\n";
        exit(1);
    }

    string line;
    int M = 0;
    while (getline(infile, line)) {
        if (line.find("VEHICLE") != string::npos) {
            getline(infile, line);
            getline(infile, line);
            istringstream vehicle_line(line);
            vehicle_line >> M >> vehicleCapacity;
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
    startTime = clock();
    mt19937 rng(time(0));

    const int populationSize = 50;
    vector<Individual> population;
    for (int i = 0; i < populationSize; ++i) {
        Individual ind = create_random_individual(rng);
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
        vector<Individual> newPop;
        while (newPop.size() < populationSize) {
            uniform_int_distribution<> dist(0, populationSize - 1);
            // Individual p1 = population[dist(rng)];
            // Individual p2 = population[dist(rng)];
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
            
            Individual p1 = tournament_select(3);
            Individual p2 = tournament_select(3);
            logFile << "Parent 1 fitness: " << p1.fitness << ", routes: " << p1.routes.size() << "\n";
            logFile << "Parent 2 fitness: " << p2.fitness << ", routes: " << p2.routes.size() << "\n";


            Individual child = crossover(p1, p2, rng);
            if (rand() % 100 < 20) {
                mutate(child, rng);
                logFile << "Mutation applied\n";
        } else {
    logFile << "Mutation skipped\n";
}
/////// ----------- ADDED DETSILED LOG ----------------- ////////
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
    write_to_file(bestIndividual, inputFile);
    return 0;
}
