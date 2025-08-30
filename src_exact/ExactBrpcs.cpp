#include "ExactBrpcs.h"
#include "Constants.h"
#include "../src_heur/Parameters.h"
#include <set>

void ExactBrpO::Solve(Prob * pprob, RouteFeasibility * _r)
{
	start_time = clock();
	prob = pprob;
	r = _r;
	graph = new ExactBrpGraphO(prob,r);

	IloEnv env;
	Init(env);
	SolveProblem(env);
	
	Clear(); //Releases resource from cplex callback objects and user defined objects graph and sep
	
	env.end();
}

void ExactBrpO::SolveProblem(IloEnv env)
{
	try
	{
		bool re = cplex.solve();
		clock_t end_time = clock();
		time_taken = (double)(end_time - start_time) / CLOCKS_PER_SEC;
		
		int cplex_status = (int)cplex.getCplexStatus();
		double sol = re?cplex.getObjValue():9999999.0;
		
		printf("re:%d sol:%.3lf status:%d nbnodes:%d time:%.3lf status:%d\n", (int)re, sol, cplex_status, (int)cplex.getNnodes(), time_taken,cplex_status);
		
		if( re && (cplex_status == 11 || cplex_status == 1) )
		{
			status = cplex_status;
			ub = sol;
			ub_distance = sol - cplex.getValue(theta);
			ub_recourse = cplex.getValue(theta);
			lb = cplex.getBestObjValue();
			drvs = (int)cplex.getValue(z);
			//cannot call cplex.GetValue() if no solution was found ...
			if(cplex_status == 11)
				printf("Timed out ObjLb:%.1lf Ub:%.1lf Rec:%.1lf Drvs:%d\n"
					,lb,ub,ub_recourse,drvs);
			else 
				printf("Optimal ObjLb:%.1lf Ub:%.1lf Rec:%.1lf Drvs:%d\n"
					,lb,ub,ub_recourse,drvs);
		} 
		
		if(_sep->best_solution.size() >= 1)
		{
			//build the solution
			for (int i = 0; i < graph->GetArcCount(); i++)
			{
				graph->GetArc(i)->value = (double)cplex.getValue(x[i]);
				//printf("Arc:%d value:%.2lf\n",i,cplex.getValue(x[i]));
			}
			graph->AssignPositiveValues();
			graph->MakePaths();
			//graph->ShowPosValueArcs();
			printf("Solution IsInteger:%d Paths:%d\n",graph->IsInteger(),graph->GetPathCount()); 
			graph->ShowPaths();
			
			double sum_rec = 0;
			for (int i = 0; i < graph->GetPathCount(); i++)
			{
				std::vector<Node*>& path = graph->GetPath(i);
				
				int rec = r->CalculateContinueToNextMIP(path, prob->GetDriver(0)->capacity, 1);
				bool is_feas = rec < 9990 ? true : false;
				printf("Path:%d IsFeas:%d Rec:%d\n",i,is_feas,rec);
				sum_rec += rec;
			}
			std::cout << "Theta: " << cplex.getValue(theta) << " sumRec: " << sum_rec << std::endl;
		}

		nb_inf_sets = _sep->nb_inf_sets;
		nb_inf_paths = _sep->nb_inf_paths;
		nb_sub_tours = _sep->nb_sub_tours;
		nb_sub_tour_frac = _sep->nb_sub_tour_frac;
		nb_opt_cuts = _sep->nb_opt_cuts;
		
		printf("nb_sub_tours:%d\n", _sep->nb_sub_tours);
		printf("nb_sub_tours_frac:%d\n", _sep->nb_sub_tour_frac);
		printf("nb_inf_paths:%d\n",_sep->nb_inf_paths);
		printf("nb_inf_sets:%d\n",_sep->nb_inf_sets);
		printf("nb_max_route_cuts:%d\n",_sep->nb_max_route_cuts);
		printf("nb_opt_cuts:%d\n",_sep->nb_opt_cuts);

	} catch (IloException &ex) {
	   std::cerr << ex << std::endl;
	}

}

void ExactBrpO::Init(IloEnv env)
{
	int Q = prob->GetDriver(0)->capacity;
	model = IloModel(env);

	std::vector<Node*> stations;
	stations.resize(graph->GetNodeCount(), NULL);
	for (int i = 0; i < graph->GetNodeCount(); i++)
		stations[i] = graph->GetNode(i);

	x = IloNumVarArray(env, graph->GetArcCount(), 0, 1, ILOINT);
	z = IloNumVar(env, prob->GetDriverCountLB(), prob->GetDriverCount(), ILOINT);
	theta = IloNumVar(env,prob->GetL(),IloInfinity,ILOFLOAT);
	
	theta.setName("t");
	z.setName("z");
	
	//std::map<std::pair<int, int>, int> arc_seen;
	for (int i = 0; i < graph->GetArcCount(); i++)
	{
		ExBrpArcO* ar = graph->GetArc(i);
		
		char name[40];
		sprintf(name, "x%d_%d", ar->from->no, ar->to->no);
		x[i].setName(name);
		if (ar->from->no != 0)
			x[i].setBounds(0, 1);
	}


	IloExpr obj1(env);
	for (int i = 0; i < graph->GetArcCount(); i++)
		obj1 += graph->GetArc(i)->cost * x[i];
	obj1 += theta;
	obj_func = IloMinimize(env, obj1);
	model.add(obj_func);
	obj1.end();


	//\sum_{ i \in C} x_{0i} = 2K
	{
		IloExpr expr(env);
		for (int i = 0; i < graph->GetArcsInOfCount(0); i++)
			expr += x[graph->GetArcInOf(0, i)->index];
		model.add(expr == z);
		expr.end();
	}

	{
		IloExpr expr(env);
		for (int i = 0; i < graph->GetArcsOutOfCount(0); i++)
			expr += x[graph->GetArcOutOf(0, i)->index];
		model.add(expr == z);
		expr.end();
	}

	//station's in and out degree constrains
	{
		for (int i = 1; i < graph->GetNodeCount(); i++)
		{
			IloExpr expr(env);
			for (int j = 0; j < graph->GetArcsInOfCount(i); j++)
				expr += x[graph->GetArcInOf(i, j)->index];
			model.add(expr == 1);
			expr.end();
		}

		for (int i = 1; i < graph->GetNodeCount(); i++)
		{
			IloExpr expr(env);
			for (int j = 0; j < graph->GetArcsOutOfCount(i); j++)
				expr += x[graph->GetArcOutOf(i, j)->index];
			model.add(expr == 1);
			expr.end();
		}
	}

	//x_{ij} + x_{ji} \leq 1
	//Necessary for preventing 2-cycles
	{
		for (int i = 1; i < graph->GetNodeCount(); i++)
			for (int j = i + 1; j < graph->GetNodeCount(); j++)
				if (i != j)
				{
					ExBrpArcO* ar = graph->GetArc(i, j);
					ExBrpArcO* arr = graph->GetArc(j, i);
					if(ar == NULL || arr == NULL) continue;

					IloExpr expr(env);
					expr += x[ar->index];
					expr += x[arr->index];
					model.add(expr <= 1);
					expr.end();
				}
	}

	
	// To separate the cuts 
	_sep = new ExactBrpSep(env, graph, x, theta, r);
	printf("%s\n",_sep->VersionTag());
	
	lazy_call = new (env) ExactBrpLazyCallBackO(env, graph, x, theta, _sep);
	user_call = new (env) ExactBrpUserCutCallBackO(env, graph, x, theta, _sep);
	
	_sep->best_sol = 9999999999; _sep->best_sol_distance = 9999999999;	

	cplex = IloCplex(model);
	SetMipStart();
	//cplex.exportModel("brp_first_stage.lp");
	cplex.setParam(IloCplex::Param::TimeLimit,max_time);
	cplex.setParam(IloCplex::Param::Threads, 1);
	cplex.setWarning(env.getNullStream());
	cplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, prob->GetUpperBound() + EPSILON + 0.01);
	
	lazy_call->add_constraints = user_call->add_constraints = false;
	cplex.setParam(IloCplex::Param::MIP::Limits::Nodes,9999999999);
	cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0);
	
	// ======= Limiting Root Node Time ========== //
	cplex.setParam(IloCplex::Param::MIP::Strategy::HeuristicFreq, -1);
	cplex.setParam(IloCplex::Param::MIP::Strategy::VariableSelect, 4);
	cplex.setParam(IloCplex::Param::Emphasis::MIP, 1);
	//cplex.setParam(IloCplex::WorkMem, 1024); // For local machine but deprecated
	// ========================================== //	
	
	cplex.use(lazy_call);
	cplex.use(user_call);
}

void ExactBrpO::SetMipStart()
{
	std::vector<double> start_arcs(graph->GetArcCount(),0.0);
	for(int i=0;i<prob->GetDriverCount();i++)
	{
		Driver * d = s.GetDriver(i);
		Node * cur = s.GetNode( d->StartNodeID );
		if(s.RoutesLength[i] == 0) continue; //Unassigned customers
		while(cur != NULL)
		{
			int start_no = cur->no;
			cur = s.Next[ cur->id ];
			int next_no = cur->no;
			ExBrpArcO* arc = graph->GetArc(start_no,next_no);

			start_arcs[arc->index] = 1.0;
			//printf("start arc---> index:%d from:%d to:%d value:%.1lf\n",arc->index,arc->from->no,arc->to->no,start_arcs[arc->index]);

			if(cur->type == NODE_TYPE_END_DEPOT) break;
		}
	}

	//x_ij
	IloNumVarArray startVar(cplex.getEnv());
	IloNumArray startVal(cplex.getEnv());
	for(int i = 0; i < graph->GetArcCount(); i++)
	{
		ExBrpArcO* arc = graph->GetArc(i);
		startVar.add(x[arc->index]);
		startVal.add(start_arcs[i]);
		//startVal.add(start_arcs[i]+0.01);
	}
	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartRepair);
	//cplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 10);
	startVal.end();
	startVar.end();

}

void ExactBrpO::Clear()
{
	delete graph;
	delete _sep;
	if (lazy_call != NULL)
	{
		cplex.remove(lazy_call);
		delete lazy_call;

	}
	if (user_call != NULL)
	{
		cplex.remove(user_call);
		delete user_call;
	}
}

