#ifndef ROUTE_FEASIBILITY_BRPCS
#define ROUTE_FEASIBILITY_BRPCS

#include <cstring> // For strcmp

#include "ProblemDefinition.h"
#include "PathHashTable.h"
#include <ilcplex/ilocplex.h>
#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include "constants.h"

class RouteFeasibility {
public:	
	RouteFeasibility(Prob * _prob) : prob(_prob), _show(false), visited_paths_CN(0), visited_paths_RT(0) {
		_paths_map_continue = new PathHashTable<int>(3000000);
		_paths_map_restocking = new PathHashTable<double>(3000000);
	};
	
	~RouteFeasibility(){
		delete _paths_map_continue;
		delete _paths_map_restocking;
	}
	
	// Cost computation
	double CalculateRestockingTrips(std::vector<Node*>& nodes, int Q, int delta);
	double CalculateRestockingTripsNonLinear(std::vector<Node*>& nodes, int Q, int delta, IloEnv env);
	double CalculateRestockingTripsBigM(std::vector<Node*>& nodes, int Q, int delta, IloEnv env);
	double CalculateRestockingTripsLM(std::vector<Node*>& nodes, int Q, int delta, IloEnv env);
	double CalculateRestockingTripsSW(std::vector<Node*>& nodes, int Q, int delta, IloEnv env);
	
	int CalculateContinueToNextMIP(std::vector<Node*>& nodes, int Q, int delta);
	int CalculateContinueToNextMIP(std::vector<Node*>& nodes, int Q, int delta, IloEnv env);
	int CalculateContinueToNextSW(std::vector<Node*>& nodes, int Q, int delta, IloEnv env);
	
	int CalculateContinueToNextDP(std::vector<Node*>& nodes, int Q, int delta);
	bool CheckSummedInequality(Node* node, int x, int e, int W, int z_k);
	bool SkipForGivenXE(Node* node, int x, int e);
	
	// Lb
	static double CalculateSequenceLb(std::vector<Node*>& nodes, int Q, int delta);
	
	// Helper functions
	void CalculateBigM(const std::vector<Node*> & nodes, int Q, std::vector<double> & M_cn, std::vector<double> & M_r, bool show);
	
	// Feasibility checks
	static bool HasZeroHCUnchargedViolations(std::vector<Node*>& nodes, int Q, bool show);
	static bool HasZeroHCUnchargedViolations(std::vector<Node*>& nodes, int Q, bool show, int init_q, int init_qe);
	static bool HasZeroHCUncharged(std::vector<Node*>& nodes, int Q, bool show);
	static bool HasZeroHCUncharged(std::vector<Node*>& nodes, int Q, bool show, int init_q, int init_qe);
	static bool HasZeroHCBase(std::vector<Node*>& nodes, int Q, bool show);
	static bool HasZeroHCBase(std::vector<Node*>& nodes, int Q, bool show, int init_q, int init_qe);
	static bool EndLoadHybrid(std::vector<Node*>& nodes, int Q, bool show);
 	
	// Switch to debug
	bool _show; bool restocked;
	int nb_restocks;
	
private:
	
	Prob * prob;
    
	IloModel model;
    IloCplex cplex_continue;
	IloCplex cplex_restock;
    IloConstraintArray constraints;
	IloObjective objective;
	IloExpr objExpr;
    
    // Variable arrays
    IloNumVarArray x, e, u;
    IloNumVarArray wX_minus, wX_plus, wE_minus, wE_plus;
    IloNumVarArray y, z;
	
	// Restock arrays
	IloNumVarArray x_dep, e_dep, b, F;
     
	PathHashTable<int> * _paths_map_continue; //set of paths that have been visited
	PathHashTable<double> * _paths_map_restocking; //set of paths that have been visited
	std::vector<int> path_ids;	
	int visited_paths_CN;
	int visited_paths_RT;
};

#endif