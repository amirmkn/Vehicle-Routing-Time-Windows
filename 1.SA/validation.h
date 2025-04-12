#ifndef VALIDATION_H
#define VALIDATION_H

#include <string>
#include <vector>
using namespace std;

// Forward declarations; these types must match your definitions in the main code.
struct Customer {
    int id;
    int x, y;
    int demand;
    int earliest;
    int latest;
    int service_time;
};

struct Route {
    vector<int> customers;
    int total_demand = 0;
    
    // Overload operator== for Route inside the struct (or outside, as an inline function)
    friend bool operator==(const Route &lhs, const Route &rhs) {
        return lhs.customers == rhs.customers &&
               lhs.total_demand == rhs.total_demand;
    }
};

using Solution = vector<Route>;

// Declaration of the validation function.
string validate_solution(const Solution &sol);

#endif // VALIDATION_H
