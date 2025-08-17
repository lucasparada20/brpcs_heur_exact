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
	
	InsRmvMethodBRPCS method(&r,&pr);
	SteepestDescentInsertionBRPCS steep_seq(method,&r,0.5);
	SeqInsertBRPCS seq(method,&r);
	RegretInsertBRPCS regret_2(&pr,method,&r,0.5); //regret_k initialized with 2
	RegretInsertBRPCS regret_n(&pr,method,&r,0.5);	
	regret_n.SetK(5);
	
	RemoveRandomBRPCS random_remove;
	RelatednessRemoveBRPCS related_remove(pr.GetDistances());

	ALNS alns;
	alns.AddInsertOperator(&seq);
	alns.AddInsertOperator(&regret_2);
	alns.AddRemoveOperator(&random_remove);
	alns.AddRemoveOperator(&related_remove);

	//Initially, cost_policy set to false in Parameters (Continue-To-Next)	
	steep_seq.Insert(sol,true);	//steep_seq is faster but weaker than greedy insertion

	printf("Initial solution cost:%.1lf unassigneds:%d\n",sol.GetCost(),sol.GetUnassignedCount()); 
	//Alns parameters
	alns.SetAcceptationGap(1.1);
	alns.SetTemperatureIterInit(0);
	//alns.SetTemperature(0.9995); // For CN
	//alns.SetTemperature(0.9980); //For RT
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
	
	std::string bss_type_str = Parameters::GetBSSType() == CS ? "_CS" : "_SW";
	std::string re_continue_file_name = std::string("results/re_CN_") + Parameters::GetCityName() + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";
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
	re_file_CN << std::fixed << std::setprecision(2) << elapsed_time << ";\n";
	printf("ReCN file written to:%s\n",re_continue_file_name.c_str());
	re_file_CN.close();


	/* ====================== */
	/* ==== RESTOCK RUN ===== */
	
	begin = clock();
	
	ALNS alns2;
	alns2.AddInsertOperator( &seq );
	alns2.AddRemoveOperator(&random_remove);
	alns2.AddRemoveOperator(&related_remove);

	//Alns parameters
	alns2.SetAcceptationGap(1.1);
	alns2.SetTemperatureIterInit(0);
	alns.SetTemperature(0.9980); //For RT
	alns2.SetIterationCount(100);//Remember to set a lot of iterations
	
	Parameters::SetCostPolicy(RT); // Use restocking trips policy
	printf("Cost policy:%s\n",Parameters::GetCostPolicy() == RT ? "Restock" : "Continue-To-Next");
	alns2.SetCostPolicy( Parameters::GetCostPolicy() ); 
	
	begin = clock();
	alns2.Optimize(sol);
	sol.Update();
	
	nb_unassigned = 0;
	for(int i=0;i<pr.GetCustomerCount();i++)
	{
		Node * n = pr.GetCustomer(i);
		if(sol.IsUnassigned(n))
			nb_unassigned++;
	}
	sol.Show();
	printf("Manual count of nb_unassigned:%d\n",nb_unassigned);
	
	heur_ub_distance = sol.GetTotalDistances();
	heur_ub_recourse = sol.GetTotalRecourse();
	heur_ub = heur_ub_distance + heur_ub_recourse;
	heur_nb_drivers = sol.GetUsedDriverCount();
	elapsed_time = (double)(clock() - begin)/CLOCKS_PER_SEC;
	printf("%s policy Heur:%.3lf dist:%.3lf rec:%.3lf drv:%d time:%.3lf\n", 
			Parameters::GetCostPolicy() ? "Restocking Trips" : "Continue-To-Next", heur_ub, heur_ub_distance, heur_ub_recourse,heur_nb_drivers, elapsed_time);	

	std::string re_restock_file_name = std::string("results/re_RSTK_") + std::string(Parameters::GetCityName()) + "_" + std::to_string(Parameters::GetUValue()) + bss_type_str + ".txt";
	std::ofstream re_file_RSTK(re_restock_file_name);
	if(!re_file_RSTK.is_open())
	{
		std::cout << re_restock_file_name << std::endl;
		std::cout << "Could not open re RSTK file. Exiting ..." << std::endl; 
		exit(1);
	}
	total_charges = 0; total_init_reg = 0; total_init_elec = 0; total_init_u = 0;
	for(int i=0;i<pr.GetCustomerCount();i++)
	{
		Node * n = pr.GetCustomer(i);
		total_init_reg += n->h_i0; total_init_elec += n->h_e_i0; total_init_u += n->h_u_i0;
		if(n->is_chargeable)
			total_charges++;
	}
	re_file_RSTK << std::string(Parameters::GetCityName()) << "," << Parameters::GetNbStations() << "," << Parameters::GetUValue() << "," "RSTK" << "," << total_init_reg << "," << total_init_elec << "," << total_init_u << "," << heur_ub << "," << heur_ub_distance << "," << heur_ub_recourse << "," << heur_nb_drivers << ",";
	re_file_RSTK << std::fixed << std::setprecision(2) << elapsed_time << ";\n";
	printf("ReRSTK file written to:%s\n",re_restock_file_name.c_str());
	re_file_RSTK.close();	
	
	return 0;
}