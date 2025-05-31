#include "validation.h"
#include <sstream>
#include <algorithm>

// These external variables and functions are defined in your main program.
extern vector<Customer> customers;
extern int DEPOT;
extern double Q;
extern vector<vector<double>> dist;
extern vector<vector<double>> travel_time;

// Helper function to check feasibility for a single route.
static bool is_route_feasible(const Route &route) {
    double time = 0.0;
    int load = 0;
    if (route.customers.empty() || route.customers.front() != DEPOT || route.customers.back() != DEPOT)
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

// Validation function definition.
string validate_solution(const Solution &sol) {
    stringstream ss;

    // Check feasibility for each route.
    bool feasible = true;
    for (const auto &route : sol) {
        if (route.customers.size() > 2 && !is_route_feasible(route)) {
            feasible = false;
            break;
        }
    }

    // Check that all customers (except the depot) are visited exactly once.
    vector<int> visit_count(customers.size(), 0);
    for (const auto &route : sol) {
        for (int cust : route.customers) {
            visit_count[cust]++;
        }
    }
    bool all_served = true;
    for (size_t i = 1; i < customers.size(); ++i) {
        if (visit_count[i] != 1) {
            all_served = false;
            break;
        }
    }

    // Check demand on each route.
    bool demand_ok = true;
    for (const auto &route : sol) {
        if (route.total_demand > Q) {
            demand_ok = false;
            break;
        }
    }

    // Check time window constraints.
    bool time_window_ok = true;
    for (const auto &route : sol) {
        double time = 0.0;
        for (size_t i = 0; i < route.customers.size() - 1; ++i) {
            const Customer &current = customers[route.customers[i]];
            const Customer &next = customers[route.customers[i + 1]];
            time = max(time, static_cast<double>(current.earliest));
            time += current.service_time;
            time += dist[current.id][next.id];
            if (time > next.latest) {
                time_window_ok = false;
                break;
            }
        }
        if (!time_window_ok)
            break;
    }

    // Compose validation messages.
    if (feasible)
        ss << "✅ Feasibility check passed.\n";
    else
        ss << "❌ Route feasibility check failed.\n";

    if (all_served)
        ss << "✅ All customers visited exactly once.\n";
    else
        ss << "❌ Some customers were missed or visited more than once.\n";
    if (!all_served) {
        ss << "  Customer visit counts:\n";
        for (size_t i = 1; i < customers.size(); ++i) {
            if (visit_count[i] != 1) {
                ss << "    Customer " << i << ": visited " << visit_count[i] << " times.\n";
            }
        }
    }

    if (demand_ok)
        ss << "✅ Route demands within vehicle capacity.\n";
    else
        ss << "❌ One or more routes exceed vehicle capacity!\n";

    if (time_window_ok)
        ss << "✅ Time window constraints satisfied.\n";
    else
        ss << "❌ Time window violation detected!\n";

    if (feasible && all_served && demand_ok && time_window_ok)
        ss << "🎉 Final solution is VALID.\n";
    else
        ss << "🚨 Final solution is INVALID.\n";

    return ss.str();
}
