# Vehicle Routing Problem with Time Windows (VRPTW)

A collection of metaheuristic algorithms implemented in C++ for solving the Vehicle Routing Problem with Time Windows (VRPTW).

This repository contains multiple optimization approaches developed and evaluated on Solomon-style benchmark instances, ranging from small (25 customers) to large-scale (1000 customers) problems.

---

## Overview

The Vehicle Routing Problem with Time Windows (VRPTW) is a combinatorial optimization problem in which a fleet of vehicles must serve a set of customers while satisfying:

- Vehicle capacity constraints
- Customer demand requirements
- Time window constraints
- Route feasibility constraints

The objective is typically to:

1. Minimize the number of vehicles used.
2. Minimize the total travel distance.

---

## Implemented Algorithms

### 1. Simulated Annealing (SA)

Location:

```text
1.SA/
```

Features:

- Greedy and randomized initialization
- Multiple neighborhood operators
- Temperature-based acceptance criterion
- Time-based and evaluation-based stopping conditions
- Route validation support

---

### 2. Tabu Search (TS)

Location:

```text
2.TS/
```

Features:

- Adaptive neighborhood exploration
- Tabu list memory structure
- Local search intensification
- Feasibility-aware move selection

---

### 3. GRASP

Location:

```text
3.GRASP/
```

GRASP (Greedy Randomized Adaptive Search Procedure) combines:

- Randomized greedy construction
- Local search improvement
- Multi-start optimization

Features:

- Restricted Candidate List (RCL)
- Iterative solution refinement
- Logging support

---

### 4. Genetic Algorithm (GA)

Location:

```text
4.GA/
```

Features:

- Chromosome-based route encoding
- Population initialization
- Fitness evaluation
- Selection mechanisms
- Crossover operators
- Mutation operators
- Constraint handling

---

### 5. Ant Colony Optimization (ACO)

Location:

```text
5.ACO/
```

Features:

- Pheromone-based learning
- Probabilistic route construction
- Global best reinforcement
- Vehicle and distance optimization

---

### 6. Particle Swarm Optimization (PSO)

Location:

```text
6.PSO/
```

Features:

- Particle-based solution representation
- Swarm intelligence optimization
- Mutation and reinsertion strategies
- Hybrid local search enhancements

---

## Repository Structure

```text
Vehicle-Routing-Time-Windows/
│
├── 1.SA/
│   ├── sa_v7.cpp
│   ├── validation.cpp
│   └── instances/
│
├── 2.TS/
│   ├── ts_v2.cpp
│   ├── validation.cpp
│
├── 3.GRASP/
│   ├── gr_v2.cpp
│   ├── validation.cpp
│
├── 4.GA/
│   ├── ga.cpp
│   ├── validation.cpp
│   └── Makefile
│
├── 5.ACO/
│   ├── aco.cpp
│   └── validation.cpp
│
├── 6.PSO/
│   ├── pso.cpp
│   └── validation.cpp
│
├── ACO_Instances/
├── GRASP_Instances/
└── Benchmark Instances
```

---

## Benchmark Instances

The repository includes Solomon-style benchmark instances with varying problem sizes:

### Available Sizes

- 25 customers
- 50 customers
- 100 customers
- 200 customers
- 400 customers
- 600 customers
- 800 customers
- 1000 customers

### Instance Categories

- C (Clustered)
- R (Random)
- RC (Mixed Random-Clustered)

Examples:

```text
25-rh-13.txt
50-ce-10.txt
100-rce-33.txt
400-rch-50.txt
800-rh-61.txt
```

---

## Input Format

Each instance contains:

- Customer coordinates
- Demand
- Ready time
- Due date
- Service time
- Vehicle capacity information

The depot is represented as customer `0`.

---

## Objective Function

Solutions are evaluated according to:

### Primary Objective

Minimize:

```text
Number of Vehicles
```

### Secondary Objective

Minimize:

```text
Total Travel Distance
```

---

## Compilation

### GCC / Clang

Example:

```bash
g++ -O3 sa_v7.cpp validation.cpp -o sa
```

For other algorithms:

```bash
g++ -O3 ts_v2.cpp validation.cpp -o ts
g++ -O3 gr_v2.cpp validation.cpp -o grasp
g++ -O3 ga.cpp validation.cpp -o ga
g++ -O3 aco.cpp validation.cpp -o aco
g++ -O3 pso.cpp validation.cpp -o pso
```

---

## Running

General format:

```bash
./algorithm instance_file max_time max_evaluations
```

Example:

```bash
./sa instances/100-rh-17.txt 300 100000
```

where:

- `300` = maximum execution time (seconds)
- `100000` = maximum objective evaluations

A value of:

```text
0
```

can be used to disable a stopping criterion.

Examples:

```bash
./ga instance.txt 0 100000
```

Run until evaluation limit.

```bash
./ga instance.txt 300 0
```

Run until time limit.

---

## Validation Framework

All algorithms use a common validation module:

```text
validation.cpp
validation.h
```

The validator checks:

- Capacity constraints
- Time-window feasibility
- Customer visitation
- Route correctness
- Depot consistency

---

## Output

Each algorithm generates:

- Best solution found
- Number of vehicles used
- Total traveled distance
- Route details
- Feasibility status

Some implementations additionally produce:

```text
global_log.txt
local_search_log.txt
error_log.txt
```

for debugging and analysis.

---

## Research Goals

This repository was developed to:

- Compare classical metaheuristics on VRPTW
- Evaluate scalability across benchmark sizes
- Study solution quality versus execution time
- Investigate hybrid improvement strategies

---

## Technologies

- C++17
- STL
- Metaheuristic Optimization
- Operations Research
- Vehicle Routing

---

## Future Improvements

Potential extensions include:

- Adaptive Large Neighborhood Search (ALNS)
- Hybrid GA + Local Search
- Parallel implementations
- Multi-objective optimization
- Dynamic VRPTW
- Electric Vehicle Routing variants

---

## License

This project is released under the MIT License.
