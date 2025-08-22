#include <stdio.h>
#include <stdlib.h>
#include <string.h> //strcmp
#include <time.h>
#include <csignal>
#include <iomanip>
#include <ctime>
#include <fstream>
#include <iostream>
#include <set>
#include <map>

//General headers
#include "Parameters.h"
#include "ProblemDefinition.h"
#include "Solution.h"

//==== Alns headers ====//
#include "SteepestDescentInsertionBRPCS.h"
#include "RemoveRandomBRPCS.h"
#include "InsRmvMethodBRPCS.h"
#include "MoveBRPCS.h"
#include "AlnsBRPCS.h"
#include "SequentialInsertionBRPCS.h"
#include "RelatednessRemoveBRPCS.h"
#include "RegretInsertionBRPCS.h"

//======================//

//Project specific headers

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include "LoadBRPCS.h"
#include "CostFunctionBRPCS.h"
#include "RouteFeasibilityBRPCS.h"

// -------------------- //

#include <ilcplex/ilocplex.h> //For the tests ...

int main(int argc, char ** argv)
{

	srand(0); 

	Prob pr;
	LoadBRPCS load;

	Parameters param;
	param.Read(argc,argv);
	
	load.LoadInstance(pr, Parameters::GetInstanceFileName());
	
	//Feasibility checks ...
	int Q = pr.GetDriver(0)->capacity; int nb_charging = 0;
	for(int i=0; i<pr.GetCustomerCount(); i++)
	{
		Node * node = pr.GetCustomer(i);
		
		if(node->is_chargeable && node->h_u_i0 > 0)
		{
			std::cout << "Chargeable station CANNOT have uncharged bikes ... DP and LP will have different cost ... exiting" << std::endl;
			node->Show(); exit(1);
		}
		
		//Need to check for q > 0 and q
		if( (node->q > 0 && node->q - node->h_i0 > Q) || (node->q < 0 && node->q + node->maxWm < -Q)
			|| (node->q_e > 0 && node->q_e - node->h_e_i0 > Q) || (node->q_e < 0 && node->q_e + node->maxWm < -Q))
			{
				std::cout << "Node CANNOT have larger demand than vehicle cap ... exiting" << std::endl;
				node->Show(); exit(1);
			}
		if(node->is_chargeable) nb_charging++;
		/*if(node->q > pr.GetDriver(0)->capacity || node->q_e > pr.GetDriver(0)->capacity)
		{
			std::cout << "Node CANNOT have larger demand than vehicle cap ... exiting" << std::endl;
			node->Show(); exit(1);
		}*/
		
	}
	
	std::cout << "Nodes (includes the depots):" << pr.GetNodeCount() << " Charging:" << nb_charging << std::endl;
	//for(int i=0; i<pr.GetNodeCount();  i++)
	//	pr.GetNode(i)->Show();
	std::cout << "Customers:" << pr.GetCustomerCount() << std::endl;
	std::cout << "Drivers:" << pr.GetDriverCount() << std::endl;
	//for(int i=0; i<pr.GetCustomerCount();  i++)
	//	pr.GetCustomer(i)->Show();

	clock_t begin = clock();
	
	RouteFeasibility r(&pr); //All classes that require r utilize a reference to it.
	
	CostFunctionBRPCS cost_func(&r);
	
	//Test : DP vs MIP for SW
	/*srand(19880704); 
	int k = 40;  // Set k stations statically
	std::vector<Node*> vec1;
	RouteFeasibility r1(&pr);

	double cumTime1 = 0.0, cumTime2 = 0.0;  // cumulative totals

	for (int comb = 0; comb < 100000; comb++)
	{
		vec1.clear();
		vec1.reserve(2 * k);
		
		while ((int)vec1.size() < k)
		{
			int index = rand() % pr.GetCustomerCount();
			vec1.push_back(pr.GetCustomer(index));            
		}
		
		vec1.insert(vec1.begin(), pr.GetNode(pr.GetCustomerCount()));
		vec1.push_back(pr.GetNode(pr.GetCustomerCount() + 1));
		
		double time1, time2;
		
		clock_t begin = clock();
		IloEnv env;
		int costMIP = r1.CalculateContinueToNextSW(vec1,Q,1,env);
		env.end();
		clock_t end1 = clock();
		time1 = (double)(end1 - begin) / CLOCKS_PER_SEC;
		cumTime1 += time1;

		begin = clock();
		int costDP = r1.CostTwoDemands(vec1, Q, 1);
		clock_t end2 = clock();
		time2 = (double)(end2 - begin) / CLOCKS_PER_SEC;
		cumTime2 += time2;

		if (comb % 1000 == 0 && comb > 0)
			printf("iter:%d timeMIP:%.2lf avgTimeMip:%.2lf cumTimeMip:%.2lf "
				   "timeDP:%.2lf avgTimeDP:%.2lf cumTimeDP:%.2lf\n",
				   comb,
				   time1, cumTime1 / (comb + 1), cumTime1,
				   time2, cumTime2 / (comb + 1), cumTime2);

		if (costDP != costMIP)
		{
			printf("Phil Collins (1989) printing to debug .... CostCN:%d CostDP:%d\n", costMIP, costDP);
			printf("Stations:\n");
			for (Node* n : vec1)
				if (n->type == NODE_TYPE_CUSTOMER)
					n->Show();
			exit(1);
		} 
	}*/


	Sol sol(&pr,&cost_func);
	sol.PutAllNodesToUnassigned();
	
	InsRmvMethodBRPCS method(&r,&pr);
	SteepestDescentInsertionBRPCS steep_seq(method,&r,0.5);
	SeqInsertBRPCS seq(method,&r);
	RegretInsertBRPCS regret_2(&pr,method,&r); //regret_k initialized with 2
	RegretInsertBRPCS regret_3(&pr,method,&r); regret_3.SetK(3);
	RegretInsertBRPCS regret_4(&pr,method,&r); regret_4.SetK(4);
	//RegretInsertBRPCS regret_n(&pr,method,&r);	regret_n.SetK(pr.GetDriverCount());
	
	
	RemoveRandomBRPCS random_remove;
	RelatednessRemoveBRPCS related_remove(pr.GetDistances());

	ALNS alns;
	
	printf("Construction heuristic for Alns will be:%s\n",Parameters::GetConstructionHeuristic());
	
	if(strcmp(Parameters::GetConstructionHeuristic(),"SEQ")==0)
		alns.AddInsertOperator(&seq);
	else if(strcmp(Parameters::GetConstructionHeuristic(),"REG2")==0)
		alns.AddInsertOperator(&regret_2);
	else if(strcmp(Parameters::GetConstructionHeuristic(),"REG3")==0)
		alns.AddInsertOperator(&regret_3);
	else if(strcmp(Parameters::GetConstructionHeuristic(),"REG4")==0)
		alns.AddInsertOperator(&regret_4);
	//else if(strcmp(Parameters::GetConstructionHeuristic(),"REGn")==0)
	//	alns.AddInsertOperator(&regret_n);
	else if(strcmp(Parameters::GetConstructionHeuristic(),"ALL")==0)
	{
		alns.AddInsertOperator(&seq); alns.AddInsertOperator(&regret_2); alns.AddInsertOperator(&regret_3);
		//alns.AddInsertOperator(&regret_4); //alns.AddInsertOperator(&regret_n);
	} else {
		printf("No construction heuristic given. Phil Collins (1989). Exiting ...\n"); exit(1);
	}
	
	alns.AddRemoveOperator(&random_remove);
	alns.AddRemoveOperator(&related_remove);

	//Initially, cost_policy set to false in Parameters (Continue-To-Next)	
	steep_seq.Insert(sol,true);	//steep_seq is faster but weaker than greedy insertion
	printf("Initial solution cost:%.2lf drivers:%d unassigneds:%d\n",sol.GetCost(),sol.GetUsedDriverCount(),sol.GetUnassignedCount());
	//Alns parameters
	alns.SetAcceptationGap(1.0);
	alns.SetTemperatureIterInit(0);
	alns.SetTemperature(0.9995);
	alns.SetIterationCount(50000);//Remember to set a lot of iterations
	
	// Done in Parameters at the beginning, unless you want to restock
	Parameters::SetCostPolicy(CN); // Use Continue-To-Next trips policy
	printf("Cost policy:%s\n",Parameters::GetCostPolicy() == RT ? "Restock" : "Continue-To-Next");
	alns.SetCostPolicy( Parameters::GetCostPolicy() ); // Temporarily not used

	alns.Optimize(sol);
	sol.Update();
	
	int nb_unassigned = 0;
	for(int i=0;i<pr.GetCustomerCount();i++)
	{
		Node * n = pr.GetCustomer(i);
		if(sol.IsUnassigned(n))
			nb_unassigned++;
	}
	sol.Show();
	printf("Manual count of nb_unassigned:%d\n",nb_unassigned);
	
	double heur_ub_distance = sol.GetTotalDistances();
	double heur_ub_recourse = sol.GetTotalRecourse();
	double heur_ub = heur_ub_distance + heur_ub_recourse;
	int heur_nb_drivers = sol.GetUsedDriverCount();
	double elapsed_time = (double)(clock() - begin)/CLOCKS_PER_SEC;
	printf("%s policy Heur:%.3lf dist:%.3lf rec:%.3lf drv:%d time:%.3lf\n", 
			Parameters::GetCostPolicy() ? "Restocking Trips" : "Continue-To-Next", heur_ub, heur_ub_distance, heur_ub_recourse,heur_nb_drivers, elapsed_time);
	
	std::string cons_heur_str;
	if(strcmp(Parameters::GetConstructionHeuristic(),"SEQ")==0)
		cons_heur_str = "SEQ";
	if(strcmp(Parameters::GetConstructionHeuristic(),"REG2")==0)
		cons_heur_str = "REG2";
	if(strcmp(Parameters::GetConstructionHeuristic(),"REG3")==0)
		cons_heur_str = "REG3";
	if(strcmp(Parameters::GetConstructionHeuristic(),"REG4")==0)
		cons_heur_str = "REG4";
	if(strcmp(Parameters::GetConstructionHeuristic(),"REGn")==0)
		cons_heur_str = "REGn";
	if(strcmp(Parameters::GetConstructionHeuristic(),"ALL")==0)
		cons_heur_str = "ALL";
	
	std::string bss_type_str = Parameters::GetBSSType() == CS ? "_CS" : "_SW";
	std::string re_continue_file_name = std::string("results/re_CN_") + cons_heur_str + "_" + std::to_string((int)Parameters::MaxRouteDistance()) + "_" + Parameters::GetCityName() + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";
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
	re_file_CN << std::string(Parameters::GetCityName()) << "," << Parameters::GetNbStations() << "," << Parameters::GetUValue() << "," << "CN" << "," << total_init_reg << "," << total_init_elec << "," << total_init_u << "," << heur_ub << "," << heur_ub_distance << "," << heur_ub_recourse << "," << heur_nb_drivers << ",";
	re_file_CN << std::fixed << std::setprecision(2) << elapsed_time << "\n";
	printf("ReCN file written to:%s\n",re_continue_file_name.c_str());
	re_file_CN.close();
	
	//route file
	std::string solution_file_name_str = std::string("results/test_solution_CN_") + cons_heur_str + "_" + std::to_string((int)Parameters::MaxRouteDistance()) + "_" + Parameters::GetCityName() + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";	
	std::ofstream solutionFile(solution_file_name_str);
	
	if(!solutionFile.is_open())
	{
		printf("Could not open solutionFile file:%s\n",solution_file_name_str); 
		exit(1);
	}
	
	solutionFile << pr.GetCustomerCount() << "," << sol.GetDriverCount() << "\n";	
	int routeCounter=0;
	for(int i=0;i<sol.GetDriverCount();i++) // Printing all routes to avoid I/O bugs later when loading
	{
		Driver * d = sol.GetDriver(i);
		//if(sol.RoutesLength[d->id]==0 && strcmp(Parameters::GetCityName(),"paris") != 0) continue;

		//distances.push_back(d->curDistance);
		
		Node * curr = sol.GetNode( d->StartNodeID );
		solutionFile << routeCounter << "," << sol.RoutesLength[d->id] << "," << d->sum_q << "," << d->sum_q_e << "," << d->curDistance << "\n";
		while( curr != NULL)
		{
			solutionFile << curr->id << "-";
			//printf("%d-",curr->id);
			curr = sol.Next[ curr->id ];
		}
		//printf(" length:%d\n",sol.GetRouteLength(i));
		solutionFile << "\n";
		
		routeCounter++;
	}
	printf("Solution file written to:%s\n",solution_file_name_str.c_str());
	solutionFile.close();		

	return 0;
}