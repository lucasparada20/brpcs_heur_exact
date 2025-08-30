#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <csignal>
#include <string.h> //strcm
#include <ctime>
#include <fstream>
#include <iostream>

#include "ExactBrpcs.h"

#include "../src_heur/ProblemDefinition.h"
#include "../src_heur/Parameters.h"
#include "../src_heur/LoadBRPCS.h"
#include "../src_heur/RouteFeasibilityBRPCS.h"
#include "../src_heur/CostFunctionBRPCS.h"
#include "../src_heur/Solution.h"
#include "../src_heur/SteepestDescentInsertionBRPCS.h"
#include "../src_heur/InsRmvMethodBRPCS.h"

int main(int argc, char ** argv)
{
	Prob pr;
	LoadBRPCS load;

	Parameters param;
	param.Read(argc,argv);
	
	load.LoadInstance(pr, Parameters::GetInstanceFileName());
	
	RouteFeasibility r(&pr); //All classes that require r utilize a reference to it.
	CostFunctionBRPCS cost_func(&r);
	Sol sol(&pr,&cost_func);
	sol.PutAllNodesToUnassigned();
	
	InsRmvMethodBRPCS method(&r,&pr);
	SteepestDescentInsertionBRPCS steep_seq(method,&r,0.5);	
	
	int sum_u=0;
	for(int i=0;i<pr.GetNodeCount();i++)
			sum_u += pr.GetNode(i)->h_u_i0;
	int L = Parameters::GetBSSType() == SW ? 0 : -1*sum_u;	
	pr.SetL(L);
	printf("Lower bound L:%d\n",pr.GetL());
	
	steep_seq.Insert(sol,true); //Sequential insertion of nodes to build an initial solution and store in sol
	sol.Update();
	sol.Show();
	pr.SetUpperBound(sol.GetCost());
	
	ExactBrpO ex;
	ex.SetSolution(sol);
	ex.max_time = 30;
	ex.Solve(&pr,&r);
	
	double exact_ub_distance = -1.0; double exact_ub_recourse = -1.0; double exact_ub = -1.0; double exact_lb = -1.0; int exact_nb_drivers = -1; int exact_status = -1; int nb_sub_tours = -1; int nb_opt_cuts = 0; int nb_inf_paths = 0; double elapsed_time = -1.0;
	
	if(ex.status == 1 || ex.status == 11)
	{
		exact_status = ex.status;
		exact_ub = ex.ub;
		exact_lb = ex.lb;
		exact_ub_distance = ex.ub_distance;
		exact_ub_recourse = ex.ub_recourse;
		exact_nb_drivers = ex.drvs;
		elapsed_time = ex.time_taken;
		nb_sub_tours = ex.nb_sub_tours + ex.nb_sub_tour_frac;
		nb_opt_cuts = ex.nb_opt_cuts;
		nb_inf_paths = ex.nb_inf_paths;
	}

	printf("%s policy Status:%d Exact:%.3lf ExactLb:%.3lf dist:%.3lf rec:%.3lf drv:%d time:%.3lf\n", 
			Parameters::GetCostPolicy() ? "Restocking Trips" : "Continue-To-Next", 
				exact_status, exact_ub, exact_lb, exact_ub_distance, exact_ub_recourse,exact_nb_drivers, elapsed_time);
	printf("OptCuts:%d SubTours:%d InfPaths:%d\n",
			nb_opt_cuts, nb_sub_tours, nb_inf_paths);
	
	std::string bss_type_str = Parameters::GetBSSType() == CS ? "_CS" : "_SW";
	std::string re_continue_file_name = std::string("results/re_exact_CN_") + std::to_string((int)Parameters::MaxRouteDistance()) + "_" + Parameters::GetCityName() + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";
	std::ofstream re_file_CN(re_continue_file_name);
	if(!re_file_CN.is_open())
	{
		std::cout << re_continue_file_name << std::endl;
		std::cout << "Could not open CN file. Exiting ..." << std::endl; 
		exit(1);
	}
	
	int total_charges = 0; int total_init_reg = 0; int total_init_elec = 0; int total_init_u = 0;
	for(int i=0;i<pr.GetCustomerCount();i++)
	{
		Node * n = pr.GetCustomer(i);
		total_init_reg += n->h_i0; total_init_elec += n->h_e_i0; total_init_u += n->h_u_i0;
		if(n->is_chargeable)
			total_charges++;
	}
	re_file_CN << std::string(Parameters::GetCityName()) << "," << Parameters::GetNbStations() << "," << Parameters::GetUValue() 
				<< "," << "CN" << "," << total_init_reg << "," << total_init_elec << "," << total_init_u << "," 
				<< exact_status << "," << exact_ub << "," << exact_lb << "," << exact_ub_distance << "," << exact_ub_recourse << "," << pr.GetL() << "," << exact_nb_drivers << "," 
				<< nb_opt_cuts << "," << nb_sub_tours << "," << nb_inf_paths;
				
	re_file_CN << std::fixed << std::setprecision(2) << elapsed_time << "\n";
	printf("ReCN file written to:%s\n",re_continue_file_name.c_str());
	re_file_CN.close();			

	return 0;
}


