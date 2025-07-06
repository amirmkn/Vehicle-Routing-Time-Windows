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
#include "validation.h"

using namespace std;

// Problem parameters
typedef vector<int> Sequence;
int DEPOT = 0;
double Q;  // capacity
int M;     // max vehicles
vector<Customer> customers;
vector<vector<double>> dist, travel_time;

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

// Read instance
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

// Build distance/time matrices
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

struct Insertion { int cust, route, pos; double cost1, cost2, regret; };

// Compute feasible insertion cost; best and second-best, fully simulating time-windows
bool feasible_insertion(const vector<int>& route, int cust, int& bestPos, double& bestCost, double& secondCost) {
    bestCost = secondCost = 1e18;
    bestPos = -1;
    int rsize = route.size();
    // try each insertion position
    for (int pos = 1; pos < rsize; ++pos) {
        // build temp route with insertion
        vector<int> tmp = route;
        tmp.insert(tmp.begin() + pos, cust);
        double time = 0.0;
        bool ok = true;
        // simulate along tmp
        for (int i = 1; i < tmp.size(); ++i) {
            int u = tmp[i-1], v = tmp[i];
            double travel = travel_time[u][v];
            double service = (i==1 ? 0.0 : customers[u].service_time);
            time = max(time + service + travel,
                       static_cast<double>(customers[v].earliest));
            if (time > customers[v].latest) { ok = false; break; }
        }
        if (!ok) continue;
        // compute extra cost as cost of inserting cust between route[pos-1] and route[pos]
        int u = route[pos-1], v = route[pos];
        double base = travel_time[u][v];
        double extra = travel_time[u][cust] + travel_time[cust][v] - base;
        // record best and second-best
        if (extra < bestCost) { secondCost = bestCost; bestCost = extra; bestPos = pos; }
        else if (extra < secondCost) { secondCost = extra; }
    }
    return bestPos >= 0;
}
// Regret-2 insertion decoder
vector<Route> decode(const Sequence &seq) {
    int nCust = customers.size();
    vector<bool> used(nCust, false);
    used[DEPOT] = true;
    vector<Route> routes;

    // route building
    while (true) {
        Route R;
        R.customers = {DEPOT, DEPOT};
        R.total_demand = 0;
        // insert until no more
        while (true) {
            vector<Insertion> cand;
            for (int j : seq) {
                if (used[j]) continue;
                if (R.total_demand + customers[j].demand > Q) continue;
                int pos;
                double c1, c2;
                if (!feasible_insertion(R.customers, j, pos, c1, c2)) continue;
                cand.push_back({j, (int)routes.size(), pos, c1, c2, c2 - c1});
            }
            if (cand.empty()) break;
            // pick highest regret
            auto it = max_element(cand.begin(), cand.end(), [](auto &a, auto &b){ return a.regret < b.regret; });
            // perform insertion
            R.customers.insert(R.customers.begin() + it->pos, it->cust);
            R.total_demand += customers[it->cust].demand;
            used[it->cust] = true;
        }
        // if only depot inserted -> done
        if (R.customers.size() <= 2) break;
        routes.push_back(R);
        // all assigned?
        if (all_of(used.begin(), used.end(), [](bool u){ return u; })) break;
    }
    return routes;
}

// PSO velocity as swap operations
struct SwapOp { int i, j; };
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

SolutionScore evaluate(const Sequence &seq, int iter, int maxIt) {
    auto routes = decode(seq);
    double distSum = 0;
    for (auto &r : routes) {
        for (int k = 1; k < r.customers.size(); ++k)
            distSum += travel_time[r.customers[k-1]][r.customers[k]];
    }
    int v = routes.size();
    double pen = 1e4 * v + distSum;
    return {v, distSum, pen};
}

int main(int argc, char *argv[]) {
    if (argc < 4) { cerr << "Usage: " << argv[0] << " <in> <time> <particles>\n"; return 1; }
    string in = argv[1]; int T = stoi(argv[2]); int P = stoi(argv[3]); if (P < 1) P = 30;
    read_instance(in);
    build_matrices();
    int n = customers.size();
    Sequence base(n-1);
    iota(base.begin(), base.end(), 1);

    mt19937 rng((unsigned)Clock::now().time_since_epoch().count());
    auto uni = [&](double p){ return uniform_real_distribution<>(0,1)(rng) < p; };
    struct Particle { Sequence s, pb; SolutionScore ps; vector<SwapOp> v; };
    vector<Particle> swarm(P);
    Particle gbest; gbest.ps = {INT_MAX, 1e18, 1e18};

    // init
    for (int i = 0; i < P; ++i) {
        swarm[i].s = base;
        shuffle(swarm[i].s.begin(), swarm[i].s.end(), rng);
        swarm[i].pb = swarm[i].s;
        swarm[i].ps = evaluate(swarm[i].s, 0, T);
        if (swarm[i].ps.penalized < gbest.ps.penalized) gbest = swarm[i];
    }
    auto start = Clock::now(); int iter = 0;
    // main loop
    while (chrono::duration_cast<chrono::seconds>(Clock::now() - start).count() < T) {
        ++iter;
        double w = 0.9 - 0.5 * (double)iter / T;
        for (auto &p : swarm) {
            vector<SwapOp> nv;
            for (auto &op : p.v) if (uni(w)) nv.push_back(op);
            for (auto &op : diff(p.s, p.pb)) if (uni(1.2)) nv.push_back(op);
            for (auto &op : diff(p.s, gbest.pb)) if (uni(1.2)) nv.push_back(op);
            p.v = std::move(nv);
            apply_swaps(p.s, p.v);
            auto sc = evaluate(p.s, iter, T);
            if (sc.penalized < p.ps.penalized) { p.ps = sc; p.pb = p.s; }
            if (sc.penalized < gbest.ps.penalized) gbest = p;
        }
    }
    auto bestRoutes = decode(gbest.pb);
    int V = gbest.ps.vehicles;
    double D = gbest.ps.distance;

    ofstream o(in.substr(0,in.find_last_of('.'))+"_pso.txt");
    for (int i = 0; i < bestRoutes.size(); ++i) {
        o << "Route " << i+1 << ": ";
        for (int j = 1; j + 1 < bestRoutes[i].customers.size(); ++j) o << bestRoutes[i].customers[j] << " ";
        o << "(d=" << bestRoutes[i].total_demand << ")\n";
    }
    o << "Vehicles:" << V << "\nDistance:" << D << "\n";
    cout << "PSO Best Vehicles=" << V << " Distance=" << D << "\n";
    cout << validate_solution(bestRoutes);
    return 0;
}
