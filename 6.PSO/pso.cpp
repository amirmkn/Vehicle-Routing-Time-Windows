#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <chrono>

using namespace std;
using namespace chrono;

struct Customer {
    int id;
    double x, y, demand;
    double ready_time, due_date, service_time;
};

struct Particle {
    vector<int> position;     // permutation of customers
    vector<int> best_position;
    double fitness = 1e9;
    double best_fitness = 1e9;
    double velocity = 0; // PSO velocity is abstract for discrete problems
};

vector<Customer> customers;
int vehicle_capacity, num_vehicles;
int depot_id = 0;
int max_evaluations = 10000;
int evaluation_count = 0;
int swarm_size = 50;
int max_iter = 1000;

random_device rd;
mt19937 rng(rd());

double distance(const Customer &a, const Customer &b) {
    return hypot(a.x - b.x, a.y - b.y);
}

vector<vector<double>> dist_matrix;

void compute_distance_matrix() {
    int n = customers.size();
    dist_matrix.assign(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            dist_matrix[i][j] = distance(customers[i], customers[j]);
}

double evaluate(const vector<int>& route) {
    ++evaluation_count;

    double total_cost = 0;
    int load = 0;
    double time = 0;
    int vehicle_count = 1;

    int prev = depot_id;

    for (int i = 0; i < route.size(); ++i) {
        int curr = route[i];
        const Customer &c = customers[curr];

        if (load + c.demand > vehicle_capacity || time + dist_matrix[prev][curr] > c.due_date) {
            total_cost += dist_matrix[prev][depot_id]; // return to depot
            vehicle_count++;
            load = 0;
            time = 0;
            prev = depot_id;
        }

        time += dist_matrix[prev][curr];
        if (time < c.ready_time) time = c.ready_time;
        if (time > c.due_date) return 1e9;

        time += c.service_time;
        load += c.demand;
        total_cost += dist_matrix[prev][curr];
        prev = curr;
    }

    total_cost += dist_matrix[prev][depot_id]; // return final to depot
    return total_cost + vehicle_count * 1000; // penalize more vehicles
}

vector<int> generate_initial_solution() {
    vector<int> order;
    for (int i = 1; i < customers.size(); ++i) order.push_back(i);
    shuffle(order.begin(), order.end(), rng);
    return order;
}

void load_customers(const string &filename) {
    ifstream in(filename);
    string line;
    int n;

    while (getline(in, line)) {
        if (line.find("NUMBER") != string::npos) break;
    }

    in >> n;
    while (getline(in, line)) {
        if (line.find("CUST") != string::npos) break;
    }

    int id;
    while (in >> id) {
        Customer c;
        c.id = id;
        in >> c.x >> c.y >> c.demand >> c.ready_time >> c.due_date >> c.service_time;
        customers.push_back(c);
    }

    cout << "Loaded " << customers.size() << " customers\n";
}

void PSO(int max_evals) {
    max_evaluations = max_evals;
    vector<Particle> swarm(swarm_size);

    for (auto &p : swarm) {
        p.position = generate_initial_solution();
        p.fitness = evaluate(p.position);
        p.best_position = p.position;
        p.best_fitness = p.fitness;
    }

    vector<int> global_best;
    double global_best_cost = 1e9;

    for (auto &p : swarm) {
        if (p.fitness < global_best_cost) {
            global_best_cost = p.fitness;
            global_best = p.position;
        }
    }

    uniform_real_distribution<double> prob(0.0, 1.0);

    for (int iter = 0; iter < max_iter && evaluation_count < max_evaluations; ++iter) {
        for (auto &p : swarm) {
            vector<int> new_pos = p.position;

            if (prob(rng) < 0.5) {
                // Apply crossover with global best (discrete recombination)
                int i = rand() % new_pos.size();
                int j = rand() % new_pos.size();
                if (i > j) swap(i, j);
                vector<int> segment(global_best.begin() + i, global_best.begin() + j);
                vector<int> new_route;
                for (int k = 0; k < new_pos.size(); ++k) {
                    if (find(segment.begin(), segment.end(), new_pos[k]) == segment.end())
                        new_route.push_back(new_pos[k]);
                }
                new_route.insert(new_route.begin() + i, segment.begin(), segment.end());
                new_pos = new_route;
            }

            if (prob(rng) < 0.3) {
                // Mutation (swap)
                int i = rand() % new_pos.size();
                int j = rand() % new_pos.size();
                swap(new_pos[i], new_pos[j]);
            }

            double cost = evaluate(new_pos);
            if (cost < p.best_fitness) {
                p.best_fitness = cost;
                p.best_position = new_pos;
            }
            if (cost < global_best_cost) {
                global_best_cost = cost;
                global_best = new_pos;
            }
            p.position = new_pos;
        }

        cout << "Iteration " << iter << ", Best Cost: " << global_best_cost << endl;
    }

    // Output
    ofstream out("pso_output.txt");
    out << "Best cost: " << global_best_cost << "\nRoute:\n";
    for (int id : global_best) out << id << " ";
    out << endl;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        cerr << "Usage: ./vrptw_pso <input_file> <vehicle_capacity> <max_evaluations>\n";
        return 1;
    }

    string filename = argv[1];
    vehicle_capacity = stoi(argv[2]);
    max_evaluations = stoi(argv[3]);

    load_customers(filename);
    compute_distance_matrix();

    auto start = high_resolution_clock::now();
    PSO(max_evaluations);
    auto end = high_resolution_clock::now();
    cout << "Execution time: " << duration_cast<seconds>(end - start).count() << "s\n";

    return 0;
}
