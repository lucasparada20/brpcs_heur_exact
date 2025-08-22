cmake_minimum_required(VERSION 3.2)
project(ExecHeur)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # if no -std=c++11, generate an error rather than silently falling back to an older standard.

# Hard code the path to the CPLEX directory
set(CPLEX_DIR "/some/path/to/CPLEX")
message("Using CPLEX_DIR = ${CPLEX_DIR}")

# Include directories for CPLEX
set(CPLEX_INCLUDE_DIRS 
    ${CPLEX_DIR}/cplex/include
    ${CPLEX_DIR}/concert/include
)

# Link directories for CPLEX
set(CPLEX_LIBRARY_DIRS 
    ${CPLEX_DIR}/cplex/lib/x86-64_linux/static_pic
    ${CPLEX_DIR}/concert/lib/x86-64_linux/static_pic
)

# Flags for CPLEX
set(CPLEX_FLAGS ilocplex concert cplex m pthread dl)

message("CPLEX_INCLUDE_DIRS = ${CPLEX_INCLUDE_DIRS}")
message("CPLEX_LIBRARY_DIRS = ${CPLEX_LIBRARY_DIRS}")
message("CPLEX_FLAGS = ${CPLEX_FLAGS}")

# Create a shared interface target for CPLEX 
add_library(CPLEX INTERFACE)
# A simple trick to avoid multiple repetition of cplex headers and flags for compiling and linking the sources and target
# Any target linking with the library CPLEX will automatically inherit all the following settings:
target_include_directories(CPLEX INTERFACE ${CPLEX_INCLUDE_DIRS})
target_link_directories(CPLEX INTERFACE ${CPLEX_LIBRARY_DIRS})
target_link_libraries(CPLEX INTERFACE ${CPLEX_FLAGS})


# Check that all required paths and libraries exist
foreach(dir IN LISTS CPLEX_INCLUDE_DIRS CPLEX_LIBRARY_DIRS)
    if(NOT EXISTS ${dir})
        message(FATAL_ERROR "Required CPLEX path not found: ${dir}")
    endif()
endforeach()


# Source files
# All source files see the cplex headers, but doesn't affect compile time unless too many source files see these headers
#main_test_CN.cpp
#main_heur_brpcs.cpp
#main_test_RT.cpp
set(SOURCES
	AlnsBRPCS.cpp
    main_test_RT.cpp
    RouteFeasibilityBRPCS.cpp
    Parameters.cpp
    LoadBRPCS.cpp
    CostFunctionBRPCS.cpp
    InsRmvMethodBRPCS.cpp
	AlnsOutils.cpp
	SteepestDescentInsertionBRPCS.cpp
	SequentialInsertionBRPCS.cpp
	RegretInsertionBRPCS.cpp
	RelatednessRemoveBRPCS.cpp
	RemoveRandomBRPCS.cpp
	BestSolutionList.cpp
	primes.c
)

# Target: exec_heur
add_executable(exec_heur ${SOURCES} ${LIB_SOURCES})
target_compile_options(exec_heur PRIVATE -Wall) # -Wall is additional compiler flag: Wreoder, Wunused, etc
target_link_libraries(exec_heur PRIVATE CPLEX)

# Usage message when typing make in the build directory
add_custom_target(usage
    COMMAND ${CMAKE_COMMAND} -E echo "./exec_heur instance_file=../instances/newyork4.txt"
)


