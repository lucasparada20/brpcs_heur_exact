# Makefile for building exec_heur with CPLEX

CXX = g++
CXXFLAGS ?= -std=c++11 -Wall

# CPLEX directory
CPLEX_DIR = /home/lucasparada20/CPLEX_Studio2211

# Include and lib paths
CPLEX_INC = -I$(CPLEX_DIR)/cplex/include -I$(CPLEX_DIR)/concert/include
CPLEX_LIB_DIR = -L$(CPLEX_DIR)/cplex/lib/x86-64_linux/static_pic \
                -L$(CPLEX_DIR)/concert/lib/x86-64_linux/static_pic
CPLEX_LIBS = -lilocplex -lconcert -lcplex -lm -lpthread -ldl

# Sources and objects
SOURCES = AlnsBRPCS.cpp main_test_CN.cpp RouteFeasibilityBRPCS.cpp \
          Parameters.cpp LoadBRPCS.cpp CostFunctionBRPCS.cpp \
          AlnsOutils.cpp InsRmvMethodBRPCS.cpp SteepestDescentInsertionBRPCS.cpp \
          SequentialInsertionBRPCS.cpp RelatednessRemoveBRPCS.cpp \
          RemoveRandomBRPCS.cpp BestSolutionList.cpp primes.c

OBJECTS = $(SOURCES:.cpp=.o)
OBJECTS := $(OBJECTS:.c=.o)

# Target executable
TARGET = exec_heur

# Default rule
all: check_dirs $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) $(CPLEX_LIB_DIR) $(CPLEX_LIBS)

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(CPLEX_INC) -c $< -o $@

%.o: %.c
	$(CXX) $(CXXFLAGS) $(CPLEX_INC) -c $< -o $@

# Check that required directories exist
check_dirs:
	@for dir in $(CPLEX_DIR)/cplex/include $(CPLEX_DIR)/concert/include \
	            $(CPLEX_DIR)/cplex/lib/x86-64_linux/static_pic \
	            $(CPLEX_DIR)/concert/lib/x86-64_linux/static_pic; do \
	    if [ ! -d $$dir ]; then \
	        echo "Error: Required CPLEX path not found: $$dir"; \
	        exit 1; \
	    fi; \
	done

# Debug rule
debug: clean
	$(MAKE) all CXXFLAGS="-std=c++11 -Wall -O0 -ggdb3 -fsanitize=address -fsanitize=undefined"

# Usage message
usage:
	@echo "./exec_heur instance_file=../instances/newyork4.txt"

# Clean rule
clean:
	rm -f $(TARGET) *.o

