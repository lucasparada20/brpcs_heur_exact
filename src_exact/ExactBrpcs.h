
#ifndef EXACT_BRP_H
#define EXACT_BRP_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>
#include "../src_heur/NodeBRPCS.h"
#include "../src_heur/DriverBRPCS.h"
#include "../src_heur/ProblemDefinition.h"
#include "../src_heur/Solution.h"
#include "ExactBrpcsGraph.h"
#include "ExactBrpcsCallBacks.h"
#include "ExactBrpcsSepBase.h"
#include "ExactBrpcsSepUser.h"

// To deprecate later on ...
using ExactBrpSep = ExactBrpSepUser;

class ExactBrpO
{
	public:
		ExactBrpO()
		{
			max_time = 300;
			lazy_call = NULL;
		}
		
		void Solve(Prob * prob, RouteFeasibility * r);
		void Init(IloEnv env);
		void SolveProblem(IloEnv env);
		void Clear();
		void SetSolution(Sol & _sol){ s = _sol; }
		void SetMipStart();

		int max_time;
		double time_taken;
		double start_time;
		int status;
		double lb;
		double ub;
		double ub_recourse;
		double ub_distance;
		int nb_inf_sets;
		int nb_inf_paths;
		int nb_sub_tours;
		int nb_sub_tour_frac;
		int nb_l_cuts;
		int nb_p_cuts;
		int nb_opt_cuts;
		int nb_frac_l_cuts;
		int nb_sorted_l_cuts;
		int nb_benders_cuts;
		int drvs;

		double cplex_distances;
		double cplex_recourse;
		double cplex_relative_gap;
		bool solvedAtRoot;

	private:

		Prob * prob;
		ExactBrpGraphO * graph;
		RouteFeasibility * r;
		Sol s;
		
		IloModel model;
		IloObjective obj_func;
		IloCplex cplex;
		IloNumVarArray x;

		IloNumVar z; //Nb of vehicles
		IloNumVar theta; //Recourse
		
	
		ExactBrpSep * _sep;
		ExactBrpLazyCallBackO * lazy_call;
		ExactBrpUserCutCallBackO * user_call;
		
};

#endif
