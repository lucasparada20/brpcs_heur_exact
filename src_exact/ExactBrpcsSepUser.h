
#ifndef EXACT_BRP_SEP_USER_H
#define EXACT_BRP_SEP_USER_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>
#include <map>

#include "../src_heur/NodeBRPCS.h"
#include "../src_heur/DriverBRPCS.h"
#include "../src_heur/ProblemDefinition.h"
#include "../src_heur/RouteFeasibilityBRPCS.h"
#include "../src_heur/Parameters.h"
#include "ExactBrpcsGraph.h"
#include "ExactBrpcsSepBase.h"

class ExactBrpSepUser : public ExactBrpSepBase 
{
	public:
		
		ExactBrpSepUser(IloEnv env, ExactBrpGraphO * graph, IloNumVarArray x, IloNumVar theta, RouteFeasibility * r);

		void SeparateInt(IloRangeArray array) override;
		void SeparateFrac(IloRangeArray array) override;
		const char* VersionTag() const override { return "Separator: User"; }
		
		bool SeparateUserCapRecursive(IloRangeArray array);
		void SeparateSubTourInequalities(IloRangeArray array);
		void ResearchConnectedComponent(Node* n, int comp);
		void ResearchDepotComponent(Node* n, int comp);
		void SeparateOptCut(IloRangeArray array);
		void CutCurrentSolution(IloRangeArray array);

		//These return the number of added constraints
		int TestAndAddInfeasiblePath(std::vector<Node*> & path, IloRangeArray array);
		int TestAndAddTournament(std::vector<Node*> & path, IloRangeArray array);
		int AddInfeasiblePath(IloRangeArray array, std::vector<Node*> & path);

		double best_sol;
		double best_sol_recourse;
		double best_sol_distance;
		std::vector< std::vector<Node*> > best_solution;
		
		//Counters for cuts
		int nb_opt_cuts;
		int nb_inf_sets;
		int nb_inf_paths;
		int nb_sub_tours;
		int nb_sub_tour_frac;
		int nb_max_route_cuts;
		int nb_infeasible_solution_cuts;
    
		std::vector<int> _component;

	private:		
		
		IloEnv _env;
		ExactBrpGraphO * _graph;
		IloNumVarArray _x;
		IloNumVar _theta;

		Prob * _prob;
		RouteFeasibility * _r;

};

#endif