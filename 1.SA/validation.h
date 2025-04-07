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
};
using Solution = vector<Route>;

// Declaration of the validation function.
string validate_solution(const Solution &sol);

#endif // VALIDATION_H
