# Compiler and flags
CXX      := g++
# CXXFLAGS := -O2 -std=c++11

# Source files and target executable name
SRC      := sa_v7.cpp
TARGET   := vrp_solver

# Instance files: all .txt files in the current directory
INSTANCE_FILES := $(wildcard instances/*.txt)

# Simulation parameters: adjust these values as needed
MAX_EXEC_SECONDS := 20
MAX_EVALUATIONS  := 0

.PHONY: all compile run clean

# 'all' target compiles and then processes every instance file
all: compile run

# Compile the source code
compile: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

# Run the executable for each instance file found
run:
	@for file in $(INSTANCE_FILES); do \
		echo "--------------------------------------------"; \
		echo "Processing instance file: $$file"; \
		./$(TARGET) $$file $(MAX_EXEC_SECONDS) $(MAX_EVALUATIONS); \
	done

# Remove the executable and output files if needed
clean:
	rm -f $(TARGET) log.txt *_output.txt
