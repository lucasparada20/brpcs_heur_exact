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

	Sol sol(&pr,&cost_func);
	sol.PutAllNodesToUnassigned();

	//Debugging improved restocking policy
	/*srand(19900316); 
	Parameters::SetMaxRouteDistance(9999.0);
	int k = 100;  // Set k stations statically
	std::vector<Node*> vec1;
	RouteFeasibility r1(&pr); r1._show = false;
	int nb_restocks = 0; int nb_feasible=0;
	for (int comb = 0; comb < 10000000; comb++)
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

		//double base_dist = 0.0;
		//for (int i = 1; i < t; ++i)
		//	base_dist += pr.GetDist(vec1[i], vec1[i - 1]);		
		//printf("route distance:%.1lf MaxDist:%.1lf\n",base_dist,Parameters::MaxRouteDistance());		
		
		IloEnv env;
		double costRSTK = r1.CalculateRestockingTrips(vec1,Q,1);
		env.end();
		nb_restocks += r1.nb_restocks;
		clock_t end1 = clock();
		
		int costCN = r1.CalculateContinueToNextMIP(vec1,Q,1);
		
		double total_dist = 0.0;
		for(int i=1;i<vec1.size();i++)
				total_dist += pr.GetDist(vec1[i-1],vec1[i]);
		
		if (comb % 1000 == 0 && comb > 0)
			printf("Comb:%d Total_restocks:%d avg_restocks:%.1lf costRSTK:%.1lf costCN:%d\n",
					comb,nb_restocks,nb_restocks/(double)comb,costRSTK,costCN);
					
		if(costCN > (int)(std::floor(costRSTK + 0.1)) && !r1.nb_restocks)
		{
			printf("costCN:%d costRSTK:%.2lf restocked?:%d total_dist:%.1lf\n",costCN, costRSTK, r1.restocked, total_dist);
			for(size_t i=0;i<vec1.size();i++)
			printf("n:%zu id:%d (q,q_e,u,charges?) = (%d,%d,%d,%s)\n",
				i,vec1[i]->id,vec1[i]->q,vec1[i]->q_e,vec1[i]->h_u_i0,vec1[i]->is_chargeable ? "True" : "False");
			int t = vec1.size() - 2;
			
			
			for (int i = 1; i < t; ++i) 
			{
				double delta_dist = pr.GetDist(vec1[i], vec1[0]) + pr.GetDist(vec1[0], vec1[i + 1]) - pr.GetDist(vec1[i], vec1[i + 1]);
				printf("restock dist %d_%d: %.2lf\n",
					vec1[i]->id,vec1[i+1]->id,delta_dist);
			}
			r1._show = true;
			IloEnv env1;
			costCN = r1.CalculateContinueToNextMIP(vec1,Q,1,env1);
			env1.end();
			
			IloEnv env2;
			costRSTK = r1.CalculateRestockingTripsLM(vec1,Q,1,env2);
			env2.end();
			exit(1);
		}
					
		if (costCN < (int)(std::floor(costRSTK + 0.1)))
		{
			printf("costCN:%d costRSTK:%.2lf restocked?:%d total_dist:%.1lf\n",costCN, costRSTK, r1.restocked, total_dist);
			for(size_t i=0;i<vec1.size();i++)
			printf("n:%zu id:%d (q,q_e,u,charges?) = (%d,%d,%d,%s)\n",
				i,vec1[i]->id,vec1[i]->q,vec1[i]->q_e,vec1[i]->h_u_i0,vec1[i]->is_chargeable ? "True" : "False");
			int t = vec1.size() - 2;
			for (int i = 1; i < t; ++i) 
			{
				double delta_dist = pr.GetDist(vec1[i], vec1[0]) + pr.GetDist(vec1[0], vec1[i + 1]) - pr.GetDist(vec1[i], vec1[i + 1]);
				printf("restock dist %d_%d: %.2lf\n",
					vec1[i]->id,vec1[i+1]->id,delta_dist);
			}
			printf("Continue-To-Next cost for same path: %d RSTKcost: %.2lf\n", costCN,costRSTK);
			r1._show = true;
			
			IloEnv env1;
			costCN = r1.CalculateContinueToNextMIP(vec1,Q,1,env1);
			env1.end();			
			
			
			IloEnv env2;
			costRSTK = r1.CalculateRestockingTripsLM(vec1,Q,1,env2);
			env2.end();
			exit(1);
		}
			
	}*/
	
	
	InsRmvMethodBRPCS method(&r,&pr);
	SteepestDescentInsertionBRPCS steep_seq(method,&r,0.5);
	SeqInsertBRPCS seq(method,&r);
	RegretInsertBRPCS regret_2(&pr,method,&r); //regret_k initialized with 2
	RegretInsertBRPCS regret_3(&pr,method,&r); regret_3.SetK(3);
	RegretInsertBRPCS regret_4(&pr,method,&r); regret_4.SetK(4);
	RegretInsertBRPCS regret_n(&pr,method,&r); regret_n.SetK(pr.GetDriverCount());

	if(Parameters::GetInitialSolutionFileName()[0] != '\0')
	{
		load.LoadSolution(pr,sol,Parameters::GetInitialSolutionFileName());
		printf("Loaded the following solution:\n");
		Parameters::SetCostPolicy(RT); // Use Restocking Trips policy
		sol.Update();
		sol.Show();
		
		double cost_RT = 0.0; double cost_CN = 0.0;
		Parameters::SetCostPolicy(CN); // Use Restocking Trips policy
		cost_CN = sol.GetCost();
		
		Parameters::SetCostPolicy(RT); // Use Restocking Trips policy
		cost_RT = sol.GetCost();
		
		printf("Cost RT:%.2lf CN:%.2lf\n",cost_RT,cost_CN);
			
		//Testing the paths of the solution ...
		cost_RT = 0.0; cost_CN = 0.0; double dist = 0.0;
		for(int i=0;i<sol.GetDriverCount();i++)
		{
			Driver * d = sol.GetDriver(i);
			std::vector<Node*> path;
			sol.GetPath(d, path);
			
			if(path.size() <= 2) continue;
			
			double path_dist = 0.0;
			for(int k=1;k<path.size();k++)
				path_dist+=pr.GetDist(path[k-1],path[k]);
			IloEnv env;
			//double costRSTK = r.CalculateRestockingTripsLM(path,Q,1,env);
			double costRSTK = r.CalculateRestockingTrips(path,Q,1);
			env.end();
			IloEnv env1;
			int costCN = r.CalculateContinueToNextMIP(path,Q,1);
			env1.end();
			
			printf("D:%d/%d costMIP:%d costRSTK:%.2lf dist:%.2lf\n",
						i, sol.GetDriverCount(), costCN, costRSTK, path_dist);
			cost_CN += (double)costCN;
			cost_RT += costRSTK;
			dist += path_dist;
		}
		printf("Final costRT:%.2lf costCN:%.2lf dist:%.2lf\n",cost_RT,cost_CN,dist);	
		
	} else { //Parameters::GetInitialSolutionFileName()[0] == '\0'
		steep_seq.Insert(sol,true);	//steep_seq is faster but weaker than greedy insertion
		printf("Initial solution cost:%.2lf drivers:%d unassigneds:%d\n",sol.GetCost(),sol.GetUsedDriverCount(),sol.GetUnassignedCount());
	}	
	//Testing
	//steep_seq.Insert(sol,true);	//steep_seq is faster but weaker than greedy insertion
	//sol.Update();
	//printf("Initial solution cost:%.2lf drivers:%d unassigneds:%d\n",sol.GetCost(),sol.GetUsedDriverCount(),sol.GetUnassignedCount());
	
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
	else if(strcmp(Parameters::GetConstructionHeuristic(),"REGn")==0)
		alns.AddInsertOperator(&regret_n);
	else if(strcmp(Parameters::GetConstructionHeuristic(),"ALL")==0)
	{
		alns.AddInsertOperator(&seq); alns.AddInsertOperator(&regret_2); alns.AddInsertOperator(&regret_3);
		//alns.AddInsertOperator(&regret_4); //alns.AddInsertOperator(&regret_n);
	} else {
		printf("No construction heuristic given. Phil Collins (1989). Exiting ...\n"); exit(1);
	}
	
	alns.AddRemoveOperator(&random_remove);
	alns.AddRemoveOperator(&related_remove);


	alns.SetAcceptationGap(0.001);
	alns.SetTemperatureIterInit(0);
	alns.SetTemperature(0.9980); //For RT
	alns.SetIterationCount(100);//Remember to set a lot of iterations
	
	// Done in Parameters at the beginning, unless you want to restock
	Parameters::SetCostPolicy(RT); // Use Restocking Trips policy
	//Parameters::SetCostPolicy(CN);
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
	
	/*int global_nb_restock = 0; int global_nb_missed_bikes = 0; int global_picked_uncharged_bikes = 0; int global_delivered_uncharged_bikes = 0;
	double global_restock_distance = 0.0;
	for(int i=0;i<sol.GetDriverCount();i++)
	{
		std::vector<Node*> path; path.reserve( pr.GetCustomerCount() );
		Driver * d = sol.GetDriver(i);
		sol.GetPath(d, path);
		if(path.size() <= 2) continue;
		printf("path:\n");
		for(int j=0;j<path.size();j++)
			printf("%d-",path[j]->id);
		printf("\n");
		
		
		//RouteFeasibility r1(&pr); r1._show = true;
		IloEnv env;
		r1.CalculateRestockingTripsLM(path, sol.GetDriver(i)->capacity, 1, env);
		env.end();
		global_nb_restock += r1.total_nb_restock;
		global_nb_missed_bikes += r1.total_nb_missed_bikes;
		global_picked_uncharged_bikes += r1.total_picked_uncharged_bikes;
		global_delivered_uncharged_bikes += r1.total_delivered_uncharged_bikes;
		global_restock_distance += r1.total_restock_distance;
	}
	printf("Global values:\n");
	printf("nb_restocks:%d restock_distance:%.1lf\n",global_nb_restock,global_restock_distance);
	printf("missed_bikes:%d uncharged_delivered(z[i]):%d uncharged_picked(y[i]):%d\n",
		global_nb_missed_bikes, global_delivered_uncharged_bikes, global_picked_uncharged_bikes);*/	
	
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
	std::string re_continue_file_name = std::string("results/re_RT_") + cons_heur_str + "_" + std::to_string((int)Parameters::MaxRouteDistance()) + "_" + Parameters::GetCityName() + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";
	std::ofstream re_file_RT(re_continue_file_name);
	if(!re_file_RT.is_open())
	{
		std::cout << re_continue_file_name << std::endl;
		std::cout << "Could not open RT file. Exiting ..." << std::endl; 
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
	re_file_RT << std::string(Parameters::GetCityName()) << "," << Parameters::GetNbStations() << "," << Parameters::GetUValue() << "," << "RT" << "," << total_init_reg << "," << total_init_elec << "," << total_init_u << "," << heur_ub << "," << heur_ub_distance << "," << heur_ub_recourse << "," << heur_nb_drivers << ",";
	re_file_RT << std::fixed << std::setprecision(2) << elapsed_time << "\n";
	printf("ReRT file written to:%s\n",re_continue_file_name.c_str());
	re_file_RT.close();
	
	//route file
	std::string solution_file_name_str = std::string("results/solution_RT_") + cons_heur_str + "_" + std::to_string((int)Parameters::MaxRouteDistance()) + "_" + Parameters::GetCityName() + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";	
	std::ofstream solutionFile(solution_file_name_str);
	
	if(!solutionFile.is_open())
	{
		printf("Could not open solutionFile file:%s\n",solution_file_name_str); 
		exit(1);
	}	
	solutionFile << pr.GetCustomerCount() << "," << sol.GetUsedDriverCount() << "\n";	
	int routeCounter=0;
	for(int i=0;i<sol.GetDriverCount();i++)
	{
		Driver * d = sol.GetDriver(i);
		if(sol.RoutesLength[d->id]==0) continue;

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