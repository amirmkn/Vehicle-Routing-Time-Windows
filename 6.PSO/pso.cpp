#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <chrono>
#include <random>
#include <limits>
#include <numeric>
#include <filesystem>
#include <iomanip>
#include <unordered_set>
#include "validation.h"

using namespace std;

// Problem parameters
typedef vector<int> Sequence;
int DEPOT = 0;
double Q;  // capacity
int M;     // max vehicles
int P = 20;
vector<Customer> customers;
vector<vector<double>> dist, travel_time;

int MUT_FREQ = max(10, min(40, static_cast<int>(2000.0 / customers.size())));   // every 20 iters
const double MUT_RATE = 0.3; // mutate 30% of particles

int RI_FREQ = max(10, min(60, static_cast<int>(2500.0 / customers.size())));        // every 50 iters, do remove–reinsertion
const double RI_PERCENT = 0.1; // remove 10% of customers

int CROSS_FREQ = max(5, min(50, static_cast<int>(3000.0 / customers.size())));

int HYPER_FREQ = max(20, min(100, static_cast<int>(5000.0 / customers.size())));     // try hyper‑mutation every 50 iters
int stagnation_iters = 0;      // count iterations since gbest last improved


const bool USE_MUTATION = false;
const bool USE_CROSSOVER = false;
const bool USE_REINSERTION = false;
const bool USE_HYPERMUTATION = false;

bool USE_FULL_REGRET2 = true;


typedef std::chrono::steady_clock Clock;

struct SolutionScore {
    int vehicles;
    double distance;
    double penalized;
    bool operator<(const SolutionScore& o) const {
        if (vehicles != o.vehicles) return vehicles < o.vehicles;
        return distance < o.distance;
    }
};
struct Insertion {
    int cust;
    int route_id;
    int pos;
    double c1;
    double c2;
    double regret;
};

template<typename T> T getLineToken(istringstream &iss) { T val; iss >> val; return val; }
void read_instance(const string &f) {
    ifstream in(f);
    if (!in) throw runtime_error("Cannot open file");
    string line;
    while (getline(in, line)) {
        if (line.rfind("VEHICLE", 0) == 0) {
            getline(in, line);
            getline(in, line);
            istringstream iss(line);
            iss >> M >> Q;
            break;
        }
    }
    while (getline(in, line) && line.find("CUSTOMER") == string::npos);
    getline(in, line);
    getline(in, line);
    customers.clear();
    while (getline(in, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        Customer c;
        c.id = getLineToken<int>(iss);
        c.x = getLineToken<double>(iss);
        c.y = getLineToken<double>(iss);
        c.demand = getLineToken<double>(iss);
        c.earliest = getLineToken<double>(iss);
        c.latest = getLineToken<double>(iss);
        c.service_time = getLineToken<double>(iss);
        customers.push_back(c);
    }
}

void build_matrices() {
    int n = customers.size();
    dist.assign(n, vector<double>(n));
    travel_time = dist;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double d = hypot(customers[i].x - customers[j].x,
                             customers[i].y - customers[j].y);
            dist[i][j] = travel_time[i][j] = d;
        }
    }
}
string get_output_filename(const string& inputPath) {
    size_t slash = inputPath.find_last_of("/");
    string filename = (slash == string::npos) ? inputPath : inputPath.substr(slash + 1);
    size_t dot = filename.find_last_of(".");
    if (dot != string::npos) filename = filename.substr(0, dot);
    return filename + "_output.txt";
}

vector<vector<int>> infoMatrix;

bool feasible_insertion(const vector<int>& route, int cust, int& bestPos, double& bestCost, double& secondCost) {
    bestCost = secondCost = 1e18;
    bestPos = -1;
    int rsize = route.size();
    for (int pos = 1; pos < rsize; ++pos) {
        vector<int> tmp = route;
        tmp.insert(tmp.begin() + pos, cust);
        double time = 0.0;
        bool ok = true;
        for (int i = 1; i < tmp.size(); ++i) {
            int u = tmp[i-1], v = tmp[i];
            double travel = travel_time[u][v];
            double service = (i==1 ? 0.0 : customers[u].service_time);
            time = max(time + service + travel,
                       static_cast<double>(customers[v].earliest));
            if (time > customers[v].latest) { ok = false; break; }
        }
        if (!ok) continue;
        int u = route[pos-1], v = route[pos];
        double base = travel_time[u][v];
        double extra = travel_time[u][cust] + travel_time[cust][v] - base;
        if (extra < bestCost) { secondCost = bestCost; bestCost = extra; bestPos = pos; }
        else if (extra < secondCost) { secondCost = extra; }
    }
    return bestPos >= 0;
}

vector<Route> decode_full_regret2(const Sequence &seq) {
    int nCust = customers.size();
    vector<bool> used(nCust, false);
    used[DEPOT] = true;
    vector<Route> routes;

    while (true) {
        Route R;
        R.customers   = {DEPOT, DEPOT};
        R.total_demand = 0;
        bool firstRoute = true;

        // Build one route
        while (true) {
            // 1) Gather all feasible regret‐2 insertions
            vector<Insertion> cand;
            for (int j : seq) {
                if (used[j]) continue;
                if (R.total_demand + customers[j].demand > Q) continue;
                int pos; double c1, c2;
                if (!feasible_insertion(R.customers, j, pos, c1, c2)) 
                    continue;
                cand.push_back({j, (int)routes.size(), pos, c1, c2, c2 - c1});
            }

            // 2) If none, and this is the very first route, do one random feasible insertion
            if (cand.empty() && firstRoute && R.customers.size() == 2) {
                vector<pair<int,int>> feas;
                for (int j : seq) {
                    if (used[j]) continue;
                    for (int p = 1; p < (int)R.customers.size(); ++p) {
                        double d1, d2;
                        if (feasible_insertion(R.customers, j, p, d1, d2)) {
                            feas.emplace_back(j, p);
                        }
                    }
                }
                if (!feas.empty()) {
                    auto pick = feas[ std::rand() % feas.size() ];
                    R.customers.insert(R.customers.begin() + pick.second, pick.first);
                    R.total_demand += customers[pick.first].demand;
                    used[pick.first] = true;
                    firstRoute = false;
                    continue;  // resume regret‐2 around this seed
                }
                // else: truly no feasible insertions at all → fall through to break
            }

            // 3) If still empty, we're done with this route
            if (cand.empty()) break;

            // 4) Pick the customer with maximum regret
            auto it = max_element(
                cand.begin(), cand.end(),
                [](auto &A, auto &B){ return A.regret < B.regret; }
            );
            R.customers.insert(R.customers.begin() + it->pos, it->cust);
            R.total_demand += customers[it->cust].demand;
            used[it->cust] = true;
            firstRoute = false;
        }

        // If no real customers were inserted, stop altogether
        if (R.customers.size() <= 2) break;

        // Commit this route and check if all are served
        routes.push_back(R);
        if (all_of(used.begin(), used.end(), [](bool u){ return u; }))
            break;
    }

    return routes;
}


vector<Route> decode_klimited_regret2(const Sequence &seq, bool use_random = false) {
    int nCust = customers.size();
    vector<bool> used(nCust, false);
    used[DEPOT] = true;
    vector<Route> routes;

    int Kmin = 5;
    int Kmax = 50;
    double alpha = 3.0;

    int rawK = static_cast<int>(sqrt(nCust) * alpha);
    int K = std::min(std::max(rawK, Kmin), Kmax);

    vector<pair<double,int>> LBs;
    LBs.reserve(nCust);

    while (true) {
        Route R;
        R.customers = {DEPOT, DEPOT};
        R.total_demand = 0;
        vector<double> arrival(R.customers.size(), 0.0);
        while (true) {
            LBs.clear();
            for (int j = 1; j < nCust; ++j) {
                if (used[j] || R.total_demand + customers[j].demand > Q)
                    continue;
                double bestLB = 1e18;
                for (int pos = 1; pos < (int)R.customers.size(); ++pos) {
                    int u = R.customers[pos-1], v = R.customers[pos];
                    double lb = dist[u][j] + dist[j][v] - dist[u][v];
                    if (lb < bestLB) bestLB = lb;
                }
                LBs.emplace_back(bestLB, j);
            }
            if (LBs.empty()) break;
            int cap = min(K, (int)LBs.size());
            partial_sort(LBs.begin(), LBs.begin() + cap,
                         LBs.end(),
                         [](auto &a, auto &b){ return a.first < b.first; });

            struct Insertion { int cust, pos; double c1, c2, regret; };
            vector<Insertion> cand;
            cand.reserve(cap);

            for (int t = 0; t < cap; ++t) {
                int j = LBs[t].second;
                int bestPos; double c1, c2;
                if (!feasible_insertion(R.customers, j, bestPos, c1, c2))
                    continue;
                cand.push_back({j, bestPos, c1, c2, c2 - c1});
            }
            if (cand.empty()) break;
            auto it = use_random
                ? cand.begin() + rand() % cand.size()
                : max_element(cand.begin(), cand.end(),
                  [](auto &A, auto &B){ return A.regret < B.regret; });

            R.customers.insert(R.customers.begin() + it->pos, it->cust);
            R.total_demand += customers[it->cust].demand;
            used[it->cust] = true;
        }

        if (R.customers.size() <= 2) break;
        routes.push_back(R);
        if (all_of(used.begin(), used.end(), [](bool u){ return u; }))
            break;
    }
    return routes;
}


vector<Route> decode(const Sequence &seq, bool use_random = false) {
    int nCust = customers.size();
    vector<bool> used(nCust, false);
    used[DEPOT] = true;
    vector<Route> routes;

    int Kmin = 5;
    int Kmax = 50;
    double alpha = 3.0;

    int rawK = static_cast<int>(sqrt(nCust) * alpha);
    int K = std::min(std::max(rawK, Kmin), Kmax);

    vector<pair<double,int>> LBs;
    LBs.reserve(nCust);

    while (true) {
        Route R;
        R.customers = {DEPOT, DEPOT};
        R.total_demand = 0;
        vector<double> arrival(R.customers.size(), 0.0);
        while (true) {
            LBs.clear();
            for (int j = 1; j < nCust; ++j) {
                if (used[j] || R.total_demand + customers[j].demand > Q)
                    continue;
                double bestLB = 1e18;
                for (int pos = 1; pos < (int)R.customers.size(); ++pos) {
                    int u = R.customers[pos-1], v = R.customers[pos];
                    double lb = dist[u][j] + dist[j][v] - dist[u][v];
                    if (lb < bestLB) bestLB = lb;
                }
                LBs.emplace_back(bestLB, j);
            }
            if (LBs.empty()) break;
            int cap = min(K, (int)LBs.size());
            partial_sort(LBs.begin(), LBs.begin() + cap,
                         LBs.end(),
                         [](auto &a, auto &b){ return a.first < b.first; });

            struct Insertion { int cust, pos; double c1, c2, regret; };
            vector<Insertion> cand;
            cand.reserve(cap);

            for (int t = 0; t < cap; ++t) {
                int j = LBs[t].second;
                int bestPos; double c1, c2;
                if (!feasible_insertion(R.customers, j, bestPos, c1, c2))
                    continue;
                cand.push_back({j, bestPos, c1, c2, c2 - c1});
            }
            if (cand.empty()) break;
            auto it = use_random
                ? cand.begin() + rand() % cand.size()
                : max_element(cand.begin(), cand.end(),
                  [](auto &A, auto &B){ return A.regret < B.regret; });

            R.customers.insert(R.customers.begin() + it->pos, it->cust);
            R.total_demand += customers[it->cust].demand;
            used[it->cust] = true;
        }

        if (R.customers.size() <= 2) break;
        routes.push_back(R);
        if (all_of(used.begin(), used.end(), [](bool u){ return u; }))
            break;
    }
    return routes;
}

struct SwapOp {int i, j; };
vector<SwapOp> diff(const Sequence &a, const Sequence &b) {
    Sequence t = a;
    vector<SwapOp> ops;
    for (int i = 0; i < t.size(); ++i) {
        if (t[i] != b[i]) {
            int j = find(t.begin() + i + 1, t.end(), b[i]) - t.begin();
            ops.push_back({i, j});
            swap(t[i], t[j]);
        }
    }
    return ops;
}
void apply_swaps(Sequence &s, const vector<SwapOp> &ops) {
    for (auto &o : ops) swap(s[o.i], s[o.j]);
}

SolutionScore evaluate(const Sequence &seq, int iter) {
    bool use_random = (iter % 5 == 0);
    auto routes = USE_FULL_REGRET2 ? decode_full_regret2(seq) : decode_klimited_regret2(seq, use_random);
    double distSum = 0;
    for (auto &r : routes) {
        for (int k = 1; k < r.customers.size(); ++k)
            distSum += travel_time[r.customers[k-1]][r.customers[k]];
    }
    int v = routes.size();
    double pen = 1e4 * v + distSum;
    return {v, distSum, pen};
}

Sequence guided_reinsert(const Sequence &seedSeq) {
    auto routes = decode(seedSeq);
    if (routes.size() <= 1) return seedSeq;

    // 1. Choose a route randomly to destroy
    int ridx = rand() % routes.size();
    const Route &toRemove = routes[ridx];

    // 2. Remove customers from sequence
    unordered_set<int> removed(toRemove.customers.begin(), toRemove.customers.end());
    removed.erase(DEPOT); // never remove depot

    Sequence work;
    work.push_back(DEPOT);
    for (int c : seedSeq) {
        if (removed.count(c) == 0)
            work.push_back(c);
    }
    work.push_back(DEPOT);

    // 3. Try to reinsert removed customers using regret-2
    for (int c : toRemove.customers) {
        if (c == DEPOT) continue;
        int bestPos;
        double c1, c2;
        if (feasible_insertion(work, c, bestPos, c1, c2)) {
            work.insert(work.begin() + bestPos, c);
        }
    }

    return work;
}
// Single‐point order‐based crossover for permutations
Sequence crossover(const Sequence &A, const Sequence &B, mt19937 &rng) {
    int n = A.size();
    uniform_int_distribution<int> cutDist(1, n-2);
    int cut = cutDist(rng);

    Sequence child(n, -1);
    // 1) copy A[0..cut-1]
    for (int i = 0; i < cut; ++i)
        child[i] = A[i];

    // 2) fill rest from B in order
    int idx = cut;
    for (int i = 0; i < n; ++i) {
        int gene = B[i];
        if (find(child.begin(), child.end(), gene) == child.end()) {
            child[idx++] = gene;
        }
    }
    return child;
}



int main(int argc, char *argv[]) {
    ofstream log("pso.log");
    log << left << setw(8) << "iter" << setw(10) << "vehicles"
        << setw(15) << "distance" << setw(15) << "penalized"
        << setw(12) << "evaluations" << '\n';
    if (argc < 4) { cerr << "Usage: " << argv[0] << " <input.txt> <time> <maxEvals>\n"; return 1; }
    string in = argv[1];
    int T = stoi(argv[2]);
    int maxEvals = stoi(argv[3]);
    read_instance(in);
    build_matrices();
    cout << "Operators enabled:";
    if (USE_MUTATION) cout << " Mutation";
    if (USE_REINSERTION) cout << " Reinsertion";
    if (USE_CROSSOVER) cout << " Crossover";
    if (USE_HYPERMUTATION) cout << " Hypermutation";
    cout << "\n";

    int nCust = customers.size();
    infoMatrix.assign(nCust, vector<int>(nCust, 0));
    auto record_adjacencies = [&](const Sequence &seq) {
        for (int i = 1; i < seq.size(); ++i) {
            int u = seq[i-1], v = seq[i];
            infoMatrix[u][v] += 1;
            infoMatrix[v][u] += 1;
        }
    };
    int n = customers.size();
    Sequence base(n-1);
    iota(base.begin(), base.end(), 1);
    mt19937 rng((unsigned)Clock::now().time_since_epoch().count());
    auto uni = [&](double p){ return uniform_real_distribution<>(0,1)(rng) < p; };
    struct Particle { 
        Sequence s, pb; 
        SolutionScore ps; 
        vector<SwapOp> v; 
        int no_improvement = 0;
        vector<Route> decoded_routes;
    };
    vector<Particle> swarm(P);
    Particle gbest; gbest.ps = {INT_MAX, 1e18, 1e18};
    for (int i = 0; i < P; ++i) {
        swarm[i].s = base;
        shuffle(swarm[i].s.begin(), swarm[i].s.end(), rng);
        swarm[i].pb = swarm[i].s;
        swarm[i].ps = evaluate(swarm[i].s, 0);
        if (swarm[i].ps.penalized < gbest.ps.penalized) gbest = swarm[i];
    }
    long long evalCount = 0;
    auto start = Clock::now(); 
    int iter = 0;
    while ((T == 0 || chrono::duration_cast<chrono::seconds>(Clock::now() - start).count() < T )&& (maxEvals == 0 || evalCount < maxEvals)) {
        ++iter;
        if (USE_MUTATION && iter % MUT_FREQ == 0) {
            int nMut = max(1, int(P * MUT_RATE));
            vector<int> ids(P);
            iota(ids.begin(), ids.end(), 0);
            shuffle(ids.begin(), ids.end(), rng);
            for (int k = 0; k < nMut; ++k) {
                int idx = ids[k];
                auto &part = swarm[idx];
                shuffle(part.s.begin(), part.s.end(), rng);
                part.v.clear();
                part.ps = evaluate(part.s, iter);
                part.pb = part.s;
                if (part.ps.penalized < gbest.ps.penalized) {
                    gbest = part;
                }
            }
        }
        double progress = (double)(evalCount) / maxEvals;
        double w = 0.9 - 0.4 * progress;      // inertia: 0.9 → 0.5
        // double c1 = 2.5 - 1.5 * progress;     // personal: 2.5 → 1.0
        // double c2 = 0.5 + 1.5 * progress;     // social:   0.5 → 2.0

        for (auto &p : swarm) {
            if (T > 0 && chrono::duration_cast<chrono::seconds>(Clock::now() - start).count() >= T){
                break;
            }
            double c1 = 1.5, c2 = 1.5;
            vector<SwapOp> new_velocity;

            for (auto &op : p.v)
                if (uni(w)) new_velocity.push_back(op); // inertia

            for (auto &op : diff(p.s, p.pb))
                if (uni(c1/2.0)) new_velocity.push_back(op); // cognitive

            for (auto &op : diff(p.s, gbest.pb))
                if (uni(c2/2.0)) new_velocity.push_back(op); // social

            p.v = std::move(new_velocity);
            if (p.no_improvement > 30) {
                log << "Velocity reset for particle at iter " << iter << '\n';
                p.v.clear();  // Reset velocity after 10 stale iterations
                p.no_improvement = 0;
            }
            apply_swaps(p.s, p.v);
            auto sc = evaluate(p.s, iter);
            evalCount++;
            if (sc.penalized < p.ps.penalized) {
                p.ps = sc;
                p.pb = p.s;
                p.no_improvement = 0; // Reset improvement counter
            } else {
                p.no_improvement++;   // Count stagnation
}
            if (sc.penalized < gbest.ps.penalized) {
                gbest = p;
                record_adjacencies(gbest.pb);
                stagnation_iters = 0;   // reset counter
                gbest.decoded_routes = USE_FULL_REGRET2 ? decode_full_regret2(gbest.pb) : decode_klimited_regret2(gbest.pb, false);

            }
        }
        if (USE_REINSERTION && iter % RI_FREQ == 0) {
            Sequence cand = guided_reinsert(gbest.pb);
            auto sc2 = evaluate(cand, iter);
            log << " RI attempted at iter " << iter 
                << " veh=" << sc2.vehicles 
                << " dist=" << sc2.distance 
                << " (gbest=" << gbest.ps.vehicles << ")\n";
            if (sc2.penalized < gbest.ps.penalized) {
                gbest.pb = cand;
                gbest.ps = sc2;
                record_adjacencies(cand);
            }
        }

        if (USE_CROSSOVER && iter % CROSS_FREQ == 0) {
            // pick two distinct parents
            uniform_int_distribution<int> uniP(0, P-1);
            int i = uniP(rng), j;
            do { j = uniP(rng); } while (j == i);

            auto &parentA = swarm[i].pb;
            auto &parentB = swarm[j].pb;

            // create child
            Sequence childSeq = crossover(parentA, parentB, rng);
            auto childScore = evaluate(childSeq, iter);
            evalCount++;

            // find the worst particle by penalized cost
            int worstIdx = 0;
            double worstPen = swarm[0].ps.penalized;
            for (int k = 1; k < P; ++k) {
                if (swarm[k].ps.penalized > worstPen) {
                    worstPen = swarm[k].ps.penalized;
                    worstIdx = k;
                }
            }

            log << "Crossover at iter " << iter
                << ": child veh=" << childScore.vehicles
                << " (replacing idx=" << worstIdx << ")\n";

            // replace worst if child is better
            if (childScore.penalized < swarm[worstIdx].ps.penalized) {
                swarm[worstIdx].s  = childSeq;
                swarm[worstIdx].ps = childScore;
                swarm[worstIdx].pb = childSeq;
                // update global best if needed
                if (childScore.penalized < gbest.ps.penalized) {
                    gbest               = swarm[worstIdx];
                    record_adjacencies(gbest.pb);
                    log << " New gbest via crossover! veh=" << gbest.ps.vehicles << "\n";
                }
            }
        }

        //  Hyper‑mutation trigger
        if (USE_HYPERMUTATION && ++stagnation_iters >= HYPER_FREQ) {
            // create a fully shuffled copy of gbest
            Sequence mutant = gbest.pb;
            shuffle(mutant.begin(), mutant.end(), rng);

            // evaluate it
            auto mutantScore = evaluate(mutant, iter);
            evalCount++;

            log << "Hyper‑mutation at iter " << iter
                << ": mutant veh=" << mutantScore.vehicles
                << " (gbest=" << gbest.ps.vehicles << ")\n";

            // accept if better
            if (mutantScore.penalized < gbest.ps.penalized) {
                gbest.pb = mutant;
                gbest.ps = mutantScore;
                record_adjacencies(mutant);
                log << " New gbest via hyper‑mutation! veh=" 
                    << gbest.ps.vehicles << "\n";
            }
            stagnation_iters = 0;
        }


        log << left << setw(8) << iter
            << setw(10) << gbest.ps.vehicles
            << setw(15) << gbest.ps.distance
            << setw(15) << gbest.ps.penalized
            << setw(12) << evalCount << '\n';
    }
auto &bestRoutes = gbest.decoded_routes;
int V = bestRoutes.size();
double D = gbest.ps.distance;


    // Open output file and check
    ofstream o(in.substr(0, in.find_last_of('.')) + "_output.txt");
    if (!o) {
        cerr << "Error opening output file for writing\n";
        return 1;
    }

    // Write routes with nicer spacing and formatting
    for (int i = 0; i < (int)bestRoutes.size(); ++i) {
        o << "Route " << i + 1 << ": ";
        for (int j = 1; j + 1 < (int)bestRoutes[i].customers.size(); ++j) {
            o << bestRoutes[i].customers[j] << " ";
        }
        o << "(demand = " << bestRoutes[i].total_demand << ")\n";
    }

    // Write vehicle and distance info with fixed precision
    o << "Vehicles: " << V << "\n";
    o << "Distance: " << fixed << setprecision(2) << D << "\n";

o.close();
    cout << "PSO Best Vehicles=" << V << " Distance=" << D << "\n";
    cout << validate_solution(bestRoutes);
    log.close();
    return 0;
}
