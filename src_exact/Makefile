# Makefile for building exec_exact with CPLEX

CXX = g++
CXXFLAGS = -std=c++11 -O3 -Wall

# CPLEX directory
CPLEX_DIR = /home/lucasparada20/CPLEX_Studio2211

# Include and lib paths
CPLEX_INC = -I$(CPLEX_DIR)/cplex/include -I$(CPLEX_DIR)/concert/include
CPLEX_LIB_DIR = -L$(CPLEX_DIR)/cplex/lib/x86-64_linux/static_pic \
                -L$(CPLEX_DIR)/concert/lib/x86-64_linux/static_pic
CPLEX_LIBS = -lilocplex -lconcert -lcplex -lm -lpthread -ldl

# Sources and objects
SOURCES = ../src_heur/Parameters.cpp \
          ../src_heur/LoadBRPCS.cpp \
          ../src_heur/RouteFeasibilityBRPCS.cpp \
          ../src_heur/CostFunctionBRPCS.cpp \
          ../src_heur/SteepestDescentInsertionBRPCS.cpp \
          ../src_heur/InsRmvMethodBRPCS.cpp \
          ../src_heur/AlnsOutils.cpp \
          main_ex_brpcs.cpp \
          ExactBrpcs.cpp \
          ExactBrpcsGraph.cpp \
          Network.cpp \
          ExactBrpcsCallBacks.cpp \
          ExactBrpcsSepUser.cpp

OBJECTS = $(SOURCES:.cpp=.o)

# Target executable
TARGET = exec_exact

# Default rule
all: check_dirs $(TARGET)

# Rule to build the executable
$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) $(CPLEX_LIB_DIR) $(CPLEX_LIBS)

# Compile source files
%.o: %.cpp
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
	$(MAKE) all CXXFLAGS="-std=c++11 -Wall -ggdb3 -fsanitize=address -fsanitize=undefined"

# Usage message
usage:
	@echo "./exec_exact"

# Clean rule
clean:
	rm -f $(TARGET) *.o
