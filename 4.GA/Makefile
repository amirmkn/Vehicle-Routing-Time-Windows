# Makefile for VRPTW Genetic Algorithm

CXX       := g++
CXXFLAGS  := -std=c++17
LDFLAGS   :=

# Source files
SRC       := ga.cpp validation.cpp
OBJ       := $(SRC:.cpp=.o)
EXEC      := vrptw_ga

# Default target
all: $(EXEC)

# Link
$(EXEC): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJ) $(EXEC)

# Run with an example input file, 1h time limit, 10000 evaluations
run: $(EXEC)
	@echo "Running GA on all instances with a 20s limit and 10k evals..."
	@for f in ../instances/*.txt; do \
	  echo "-> $$f"; \
	  ./$(EXEC) $$f 20 0; \
	done

.PHONY: all clean run
