#include "RouteFeasibilityBRPCS.h"

//Needs an instance of RouteFeasibility and reuses variables
double RouteFeasibility::CalculateRestockingTrips(std::vector<Node*>& nodes, int Q, int delta)
{
	path_ids.clear(); path_ids.reserve( nodes.size() + 2 );
	for(size_t i=0;i<nodes.size();i++)
		if(nodes[i]->type == NODE_TYPE_CUSTOMER)
			path_ids.emplace_back( nodes[i]->id );
	
	double re = _paths_map_restocking->GetData(path_ids);
	if(re > 0)
	{
		visited_paths++;
		if(visited_paths % 10000 == 0 && visited_paths)
			printf("Encountered visited paths:%d\n",visited_paths);
		return (re-1000.0);
	}
	
	IloEnv env;
	// the cost is a double here and the hash map was initialized with doubles
	//double cost = CalculateRestockingTripsNonLinear(nodes,Q,delta,env);
	//double cost = CalculateRestockingTripsBigM(nodes,Q,delta,env);
	double cost = CalculateRestockingTripsLM(nodes,Q,delta,env);	
	env.end(); 
	
	_paths_map_restocking->Assign(path_ids,cost+1000.0); 
	
	return cost;
}

//Needs an instance of RouteFeasibility and reuses variables
int RouteFeasibility::CalculateContinueToNextMIP(std::vector<Node*>& nodes, int Q, int delta)
{
	path_ids.clear(); path_ids.reserve( nodes.size() + 2 );
	for(size_t i=0;i<nodes.size();i++)
		if(nodes[i]->type == NODE_TYPE_CUSTOMER)
			path_ids.emplace_back( nodes[i]->id );
	
	int re = _paths_map_continue->GetData(path_ids);
	if(re > 0)
	{
		visited_paths++;
		if(visited_paths % 10000 == 0 && visited_paths)
			printf("Encountered visited paths:%d\n",visited_paths);
		return (re-1000);
	}

	
	IloEnv env;
	int cost = CalculateContinueToNextMIP(nodes,Q,delta,env);
	env.end(); 
	//int cost = CalculateContinueToNextDP(nodes,Q,delta);
	
	_paths_map_continue->Assign(path_ids,cost+1000); //Just assign the integer recourse
	
	return cost;
}

//To compute the cost
double RouteFeasibility::CalculateRestockingTripsNonLinear(std::vector<Node*>& nodes, int Q, int delta, IloEnv env)
{
	int t =  nodes.size() - 2; // t is without counting depots
	if(t < 1) return 0.0;
	
    model = IloModel (env);
    constraints = IloConstraintArray(env);
	
	std::vector<double> M_cn, M_r;
	CalculateBigM(nodes, Q, M_cn, M_r, false);
		
    // Variables with size t+1 for flows
    x = IloNumVarArray (env, t + 1, 0, Q, ILOINT);
    e = IloNumVarArray (env, t + 1, 0, Q, ILOINT);
    u = IloNumVarArray (env, t + 1, 0, Q, ILOINT);
	constraints.add(u[0] == 0); //No uncharged from the depot
	
	x_dep = IloNumVarArray (env, t-1, 0, Q, ILOINT);
	e_dep = IloNumVarArray (env, t-1, 0, Q, ILOINT);

    // Variables with size t for weights and decisions
    wX_minus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wX_plus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wE_minus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wE_plus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    y = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    z = IloNumVarArray(env, t, 0, Q, ILOINT);
	
	// Variables to choose between cost policies
	b = IloNumVarArray(env, t-1, 0, 1, ILOINT);
	//Resources released later with env.end() ...
	IloExpr hard_b_constraint(env);
	for(int i=0;i<t-1;i++)
		hard_b_constraint += b[i];
	//constraints.add( hard_b_constraint <= 1 );	
	
	// Variables for the objective
	F = IloNumVarArray(env, t, -IloInfinity, IloInfinity, ILOFLOAT);

    // Objective function
    IloExpr objective(env);
	for (int i=0;i<t;i++)
		objective += F[i];
	model.add(IloMinimize(env, objective));
    objective.end();
	
	//Cost restricting constraints
	Node * first = nodes[1];
	IloConstraint continue_first;
	if(first->is_chargeable)
		continue_first = 0 == -F[0] + wX_plus[0] + wX_minus[0] + wE_plus[0] + wE_minus[0] - z[0];	
	else 
		continue_first = 0 == -F[0] + wX_plus[0] + wX_minus[0] + wE_plus[0] + wE_minus[0] + first->h_u_i0 - y[0] + z[0];
	constraints.add(continue_first);
	
	//Continue-to-next cost restricting
	for (int i=1;i<t;i++)
	{
		IloConstraint continue_lb; IloConstraint continue_ub;	
		Node * n = nodes[i+1]; // weird ...
		if(n->is_chargeable)
		{
			continue_lb = 0 >= -F[i] + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] - z[i] - M_cn[i-1]*b[i-1];
			continue_ub = 0 <= -F[i] + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] - z[i] + M_cn[i-1]*b[i-1];
		} else {
			continue_lb = 0 >= -F[i] + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + n->h_u_i0 - y[i] + z[i] - M_cn[i-1]*b[i-1];
			continue_ub = 0 <= -F[i] + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + n->h_u_i0 - y[i] + z[i] + M_cn[i-1]*b[i-1];
		
		}		
		constraints.add(continue_lb);
		constraints.add(continue_ub);
	}
	//Restock cost restricting
	for (int i=1;i<t;i++)
	{
		Node * curr = nodes[i];
		Node * dep = nodes[0];
		Node * next = nodes[i+1];
		
		double r = prob->GetDist(curr,dep) + prob->GetDist(dep,next) - prob->GetDist(curr,next);
	
		IloConstraint restock_lb = 0 >= -F[i] + r + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + next->h_u_i0 - y[i] - M_r[i-1]*(1-b[i-1]);
		IloConstraint restock_ub = 0 <= -F[i] + r + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + next->h_u_i0 - y[i] + M_r[i-1]*(1-b[i-1]);
		
		constraints.add(restock_lb);
		constraints.add(restock_ub);
		
		//Constraints for remaining distance
		double sum_dist = 0.0;
		for(int j=1;j<i;j++)
			sum_dist += prob->GetDist(nodes[j-1],nodes[j]);
		if(r+sum_dist > Parameters::MaxRouteDistance())
			constraints.add( b[i-1] == 0 );
	}
	
	//IfThen constraints
    constraints.add(-x[0] + x[1] + wX_plus[0] - wX_minus[0] == first->q);
    constraints.add(-e[0] + e[1] + wE_plus[0] - wE_minus[0] == first->q_e);
    constraints.add(-u[0] + u[1] + z[0] - y[0] == 0);	
	
	//Continue-to-next Flow constraints
	for (int i = 1; i < t; i++) 
	{
		Node* n = nodes[i];

		IloConstraint cn  = (b[i - 1] == 0);
		
		// Flow balance of regular CN
		IloConstraint CnFlowR = -x[i - 1] + x[i] + wX_plus[i - 1] - wX_minus[i - 1] == n->q;
		IloConstraint ifCnR = IloIfThen(env, cn, CnFlowR);
		constraints.add(ifCnR);

		// Flow balance of electric CN
		IloConstraint CnFlowE = -e[i - 1] + e[i] + wE_plus[i - 1] - wE_minus[i - 1] == n->q_e;
		IloConstraint ifCnE = IloIfThen(env, cn, CnFlowE);
		constraints.add(ifCnE);

		// Flow balance of uncharged CN
		IloConstraint CnFlowU = u[i] + z[i - 1] - y[i - 1] - u[i - 1] == 0;
		IloConstraint ifCnU = IloIfThen(env, cn, CnFlowU);
		constraints.add(ifCnU);

		if(i == t-1)
		{
			Node* lastNode = nodes[nodes.size()-2];
			constraints.add(IloIfThen(env,cn,-x[t - 1] + x[t] + wX_plus[t - 1] - wX_minus[t - 1] == lastNode->q));
			constraints.add(IloIfThen(env,cn,-e[t - 1] + e[t] + wE_plus[t - 1] - wE_minus[t - 1] == lastNode->q_e));
			constraints.add(IloIfThen(env,cn,u[t] + z[t - 1] - y[t - 1] - u[t - 1] == 0));
		}		

	}
	//Node* lastNode = nodes[nodes.size()-2];
    //constraints.add(-x[t - 1] + x[t] + wX_plus[t - 1] - wX_minus[t - 1] == lastNode->q);
    //constraints.add(-e[t - 1] + e[t] + wE_plus[t - 1] - wE_minus[t - 1] == lastNode->q_e);
	//constraints.add(u[t] + z[t - 1] - y[t - 1] - u[t - 1] == 0);

	//Restock Flow constraints
	for (int i = 1; i < t; i++)
	{
		IloConstraint rst = (b[i - 1] == 1);
		
		Node * n = nodes[i+1]; //Restock flows affect the 2nd visit and above ...
	
		// Flow balance of regular Restock
		IloConstraint RFlowR = -x_dep[i-1] + x[i] + wX_plus[i] - wX_minus[i] == n->q;
		IloConstraint ifRR = IloIfThen(env, rst, RFlowR);
		constraints.add(ifRR);
		
		// Flow balance of electric Restock
		IloConstraint RFlowE = -e_dep[i-1] + e[i] + wE_plus[i] - wE_minus[i] == n->q_e;
		IloConstraint ifRE = IloIfThen(env, rst, RFlowE);
		constraints.add(ifRE);

		// Flow balance of uncharged Restock
		IloConstraint RFlowU = u[i+1]-y[i] == 0;
		IloConstraint ifRU = IloIfThen(env, rst, RFlowU);
		constraints.add(ifRU);		
	}
	
    // Capacity constraints
	for (int i = 0; i < t; i++)
	{
		Node* n = nodes[i + 1];

		// z[i] - u[i] <= 0
		IloConstraint zu_constraint = z[i] - u[i] <= 0;
		constraints.add(zu_constraint);

		// wX_minus[i] + wE_minus[i] + z[i] <= maxWm
		IloConstraint maxWm_constraint = wX_minus[i] + wE_minus[i] + z[i] <= n->maxWm;
		constraints.add(maxWm_constraint);

		// wX_plus[i] <= h_i0
		IloConstraint h_i0_constraint = wX_plus[i] <= n->h_i0;
		constraints.add(h_i0_constraint);

		// wE_plus[i] <= h_e_i0
		IloConstraint h_e_i0_constraint = wE_plus[i] <= n->h_e_i0;
		constraints.add(h_e_i0_constraint);

		// y[i] <= h_u_i0
		IloConstraint h_u_i0_constraint = y[i] <= n->h_u_i0;
		constraints.add(h_u_i0_constraint);
	}
	
	// Vehicle capacity constraints : Continue-to-next and Restock
	constraints.add(x[0] + e[0] + u[0] <= Q);
	for (int i = 1; i < t; i++)
	{
		IloConstraint capCn = (x[i] + e[i] + u[i] <= Q);
		constraints.add(capCn);
		
		IloConstraint capRst = x_dep[i-1] + e_dep[i-1] <= Q;			
		constraints.add(capRst);
	}
	//After the final visit:	
	constraints.add(x[t] + e[t] + u[t] <= Q);	
 
    model.add(constraints);

    // Solve the model
	cplex_restock = IloCplex(env);
    cplex_restock.extract(model);
	cplex_restock.setParam(IloCplex::Param::Threads, 1);
	cplex_restock.setOut(env.getNullStream());
	cplex_restock.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0);
	//double UB = CalculateContinueToNextMIP(nodes, Q, delta);
	//cplex_restock.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, UB + 0.01);	
		
    if (!cplex_restock.solve()) 
        return 999999.0; 

    return cplex_restock.getObjValue();
}

//To compute the cost
double RouteFeasibility::CalculateRestockingTripsBigM(std::vector<Node*>& nodes, int Q, int delta, IloEnv env)
{
	int t =  nodes.size() - 2;
	if(t < 1) return 0.0;
	
    model = IloModel(env);
    constraints = IloConstraintArray(env);
	
	std::vector<double> M_cn, M_r;
	CalculateBigM(nodes, Q, M_cn, M_r, false);

    //x = IloNumVarArray(env, t + 1, 0, Q, ILOINT);
    //e = IloNumVarArray(env, t + 1, 0, Q, ILOINT);
    //u = IloNumVarArray(env, t + 1, 0, Q, ILOINT);
    x = IloNumVarArray(env, t + 1, 0, Q, ILOFLOAT);
    e = IloNumVarArray(env, t + 1, 0, Q, ILOFLOAT);
    u = IloNumVarArray(env, t + 1, 0, Q, ILOFLOAT);	
	constraints.add(u[0] == 0);

	x_dep = IloNumVarArray(env, t-1, 0, Q, ILOFLOAT);
	e_dep = IloNumVarArray(env, t-1, 0, Q, ILOFLOAT);
	//x_dep = IloNumVarArray(env, t-1, 0, Q, ILOINT);
	//e_dep = IloNumVarArray(env, t-1, 0, Q, ILOINT);

    wX_minus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wX_plus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wE_minus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wE_plus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    y = IloNumVarArray(env, t, 0, Q, ILOINT);
    z = IloNumVarArray(env, t, 0, Q, ILOINT);

	b = IloNumVarArray(env, t-1, 0, 1, ILOINT);
	IloExpr hard_b_constraint(env);
	for(int i=0;i<t-1;i++)
		hard_b_constraint += b[i];
	//constraints.add( hard_b_constraint <= 6 );	

	F = IloNumVarArray(env, t, -IloInfinity, IloInfinity, ILOFLOAT);


	/*for (int i = 0; i <= t; ++i) {
		x[i].setName(("x_" + std::to_string(i)).c_str());
		e[i].setName(("flowE_" + std::to_string(i)).c_str());
		u[i].setName(("u_" + std::to_string(i)).c_str());
	}


	for (int i = 0; i < t-1; ++i) {
		x_dep[i].setName(("x_dep_" + std::to_string(i)).c_str());
		e_dep[i].setName(("flowE_dep_" + std::to_string(i)).c_str());
	}


	for (int i = 0; i < t; ++i) {
		wX_minus[i].setName(("wX_minus_" + std::to_string(i)).c_str());
		wX_plus[i].setName(("wX_plus_" + std::to_string(i)).c_str());
		wE_minus[i].setName(("wE_minus_" + std::to_string(i)).c_str());
		wE_plus[i].setName(("wE_plus_" + std::to_string(i)).c_str());
		y[i].setName(("y_" + std::to_string(i)).c_str());
		z[i].setName(("z_" + std::to_string(i)).c_str());
	}


	for (int i = 0; i < t-1; ++i)
		b[i].setName(("b_" + std::to_string(i)).c_str());


	for (int i = 0; i < t; ++i)
		F[i].setName(("F_" + std::to_string(i)).c_str());*/


	IloExpr objective(env);
	for (int i = 0; i < t; i++)
		objective += F[i];
	model.add(IloMinimize(env, objective));
	objective.end();

	Node* first = nodes[1];
	IloConstraint continue_first;
	if(first->is_chargeable)
		continue_first = -F[0] + wX_plus[0] + wX_minus[0] + wE_plus[0] + wE_minus[0] - z[0] == 0;
	else
		continue_first = -F[0] + wX_plus[0] + wX_minus[0] + wE_plus[0] + wE_minus[0] + first->h_u_i0 - y[0] + z[0] == 0;
	constraints.add(continue_first);

	for (int i = 1; i < t; i++)
	{
		Node* n = nodes[i + 1];

		IloExpr expr = -F[i] + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i];
		if (!n->is_chargeable)
			expr += n->h_u_i0 - y[i] + z[i];
		else expr += - z[i];

		constraints.add(expr >= -M_cn[i-1] * b[i-1]);
		constraints.add(expr <=  M_cn[i-1] * b[i-1]);
		expr.end();
	}

	for (int i = 1; i < t; i++)
	{
		Node * curr = nodes[i]; Node * dep = nodes[0]; Node * next = nodes[i+1];
		double r = prob->GetDist(curr, dep) + prob->GetDist(dep, next) - prob->GetDist(curr, next);

		IloExpr expr = -F[i] + r + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + next->h_u_i0 - y[i];
		constraints.add(expr >= -M_r[i-1] * (1 - b[i-1]));
		constraints.add(expr <=  M_r[i-1] * (1 - b[i-1]));
		expr.end();

		double sum_dist = 0.0;
		for (int j = 1; j < i; j++)
			sum_dist += prob->GetDist(nodes[j-1], nodes[j]);
		if (r + sum_dist > Parameters::MaxRouteDistance())
			constraints.add(b[i-1] == 0);
	}

	// Initial flows
	constraints.add(-x[0] + x[1] + wX_plus[0] - wX_minus[0] == first->q);
	constraints.add(-e[0] + e[1] + wE_plus[0] - wE_minus[0] == first->q_e);
	constraints.add(-u[0] + u[1] + z[0] - y[0] == 0);

	// Big-M flows for Continue and Restock
	for (int i = 1; i < t; i++)
	{
		Node* n = nodes[i];
		int bi = i - 1;

		// Continue
		constraints.add(-x[bi] + x[i] + wX_plus[bi] - wX_minus[bi] - n->q <= Q * b[bi]);
		constraints.add(-x[bi] + x[i] + wX_plus[bi] - wX_minus[bi] - n->q >= -Q * b[bi]);

		constraints.add(-e[bi] + e[i] + wE_plus[bi] - wE_minus[bi] - n->q_e <= Q * b[bi]);
		constraints.add(-e[bi] + e[i] + wE_plus[bi] - wE_minus[bi] - n->q_e >= -Q * b[bi]);

		constraints.add(u[i] + z[bi] - y[bi] - u[bi] <= Q * b[bi]);
		constraints.add(u[i] + z[bi] - y[bi] - u[bi] >= -Q * b[bi]);

		if(i == t - 1)
		{
			Node* lastNode = nodes[nodes.size() - 2];
			constraints.add(-x[t - 1] + x[t] + wX_plus[t - 1] - wX_minus[t - 1] - lastNode->q <= Q * b[bi]);
			constraints.add(-x[t - 1] + x[t] + wX_plus[t - 1] - wX_minus[t - 1] - lastNode->q >= -Q * b[bi]);
			constraints.add(-e[t - 1] + e[t] + wE_plus[t - 1] - wE_minus[t - 1] - lastNode->q_e <= Q * b[bi]);
			constraints.add(-e[t - 1] + e[t] + wE_plus[t - 1] - wE_minus[t - 1] - lastNode->q_e >= -Q * b[bi]);
			constraints.add(u[t] + z[t - 1] - y[t - 1] - u[t - 1] <= Q * b[bi]);
			constraints.add(u[t] + z[t - 1] - y[t - 1] - u[t - 1] >= -Q * b[bi]);
		}

		// Restock
		Node* next = nodes[i+1];

		constraints.add(-x_dep[bi] + x[i] + wX_plus[i] - wX_minus[i] - next->q <= Q * (1 - b[bi]));
		constraints.add(-x_dep[bi] + x[i] + wX_plus[i] - wX_minus[i] - next->q >= -Q * (1 - b[bi]));

		constraints.add(-e_dep[bi] + e[i] + wE_plus[i] - wE_minus[i] - next->q_e <= Q * (1 - b[bi]));
		constraints.add(-e_dep[bi] + e[i] + wE_plus[i] - wE_minus[i] - next->q_e >= -Q * (1 - b[bi]));

		constraints.add(u[i+1] - y[i] <= Q * (1 - b[bi]));
		constraints.add(u[i+1] - y[i] >= -Q * (1 - b[bi]));
	}

	// Missed request constraints
	for (int i = 0; i < t; i++)
	{
		Node* n = nodes[i + 1];
		constraints.add(z[i] <= u[i]);
		constraints.add(wX_minus[i] + wE_minus[i] + z[i] <= n->maxWm);
		constraints.add(wX_plus[i] <= n->h_i0);
		constraints.add(wE_plus[i] <= n->h_e_i0);
		constraints.add(y[i] <= n->h_u_i0);
	}

	// Capacity
	constraints.add(x[0] + e[0] + u[0] <= Q);
	for (int i = 1; i < t; i++)
	{
		constraints.add(x[i] + e[i] + u[i] <= Q);
		constraints.add(x_dep[i-1] + e_dep[i-1] <= Q );
	}
	constraints.add(x[t] + e[t] + u[t] <= Q);

	model.add(constraints);

	cplex_restock = IloCplex(env);
    cplex_restock.extract(model);
	cplex_restock.setParam(IloCplex::Param::Threads, 1);
	cplex_restock.setOut(env.getNullStream());
	cplex_restock.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0);
	cplex_restock.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Dual); //or Dual
	cplex_restock.setParam(IloCplex::NodeAlg, IloCplex::Dual); //or Dual	
	//double UB = CalculateContinueToNextMIP(nodes, Q, delta);
	//cplex_restock.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, UB + 0.01);	

	if (!cplex_restock.solve())
		return 999999.0;
	
	/*if(_show)
	{
		restocked = false; nb_restocks = 0;
		for (int i = 0; i < t-1; i++)
			if((int)(cplex.getValue(b[i]) + 0.1)) 
			{
				restocked = true; nb_restocks++;
			}	
			
		cplex.exportModel("bigM.lp");
		printf("BigM cplex optimal values ... Restock cost:%.1lf restocked:%d nb_restocks:%d\n",(double)cplex.getObjValue(),restocked,nb_restocks);
		printf("M_cn:\n");
		for(size_t i =0;i<M_cn.size();i++)
			printf("%.1lf-",M_cn[i]);
		printf("\n");
		printf("M_r:\n");
		for(size_t i =0;i<M_r.size();i++)
			printf("%.1lf-",M_r[i]);
		printf("\n");		
		for (int i = 0; i < t-1; i++)
			std::cout << "b[" << i << "] = " << cplex.getValue(b[i]) << ", ";
		printf("\n");
		for (int i = 0; i < t; i++)
			std::cout << "F[" << i << "] = " << std::setprecision(2) << cplex.getValue(F[i]) << ", ";
		printf("\n");
		for (int i = 0; i < t-1; i++)
		{
			std::cout << "xDep[" << i << "] = " << (int)cplex.getValue(x_dep[i]) << ", ";
			std::cout << "eDep[" << i << "] = " << (int)cplex.getValue(e_dep[i]) << std::endl;			
		}
		
		for (int i = 0; i <= t; ++i) {
			std::cout << "x[" << i << "] = " << (int)cplex.getValue(x[i]) << ", ";
			std::cout << "e[" << i << "] = " << (int)cplex.getValue(e[i]) << ", ";
			std::cout << "u[" << i << "] = " << (int)cplex.getValue(u[i]) << std::endl;
		}
		
		// t variables
		for (int i = 0; i < t; ++i) {
			std::cout << "wX_minus[" << i << "] = " << (int)cplex.getValue(wX_minus[i]) << ", ";
			std::cout << "wX_plus[" << i << "] = " << (int)cplex.getValue(wX_plus[i]) << ", ";

			std::cout << "wE_minus[" << i << "] = " << (int)cplex.getValue(wE_minus[i]) << ", ";
			std::cout << "wE_plus[" << i << "] = " << (int)cplex.getValue(wE_plus[i]) << ", ";
			
			std::cout << "y[" << i << "] = " << (int)cplex.getValue(y[i]) << ", ";
			std::cout << "z[" << i << "] = " << (int)cplex.getValue(z[i]) << std::endl;
		}
		if(restocked)
		{
			printf("Restocked! check for silent bugs ....\n"); //getchar();
		}	
	}*/
	
	return cplex_restock.getObjValue();	
}

//To compute the cost
double RouteFeasibility::CalculateRestockingTripsLM(std::vector<Node*>& nodes, int Q, int delta, IloEnv env)
{
	int t = nodes.size() - 2;
	if (t < 1) return 0.0;

	model = IloModel(env);
	constraints = IloConstraintArray(env);

	std::vector<double> M_cn, M_r;
	CalculateBigM(nodes, Q, M_cn, M_r, false);

	IloNumVarArray x(env, t + 1, 0, Q, ILOFLOAT);
	IloNumVarArray e(env, t + 1, 0, Q, ILOFLOAT);
	IloNumVarArray u(env, t + 1, 0, Q, ILOFLOAT);
	constraints.add(u[0] == 0);

	IloNumVarArray lX(env, t - 1, 0, Q, ILOFLOAT);
	IloNumVarArray lE(env, t - 1, 0, Q, ILOFLOAT);
	IloNumVarArray mX(env, t - 1, 0, Q, ILOFLOAT);
	IloNumVarArray mE(env, t - 1, 0, Q, ILOFLOAT);
	IloNumVarArray mU(env, t - 1, 0, Q, ILOFLOAT);

	IloNumVarArray wX_minus(env, t, 0, IloInfinity, ILOINT);
	IloNumVarArray wX_plus(env, t, 0, IloInfinity, ILOINT);
	IloNumVarArray wE_minus(env, t, 0, IloInfinity, ILOINT);
	IloNumVarArray wE_plus(env, t, 0, IloInfinity, ILOINT);
	IloNumVarArray y(env, t, 0, Q, ILOINT);
	IloNumVarArray z(env, t, 0, Q, ILOINT);
	IloNumVarArray b(env, t - 1, 0, 1, ILOINT);
	IloNumVarArray F(env, t, -IloInfinity, IloInfinity, ILOFLOAT);

	/*for (int i = 0; i <= t; ++i) {
		x[i].setName(("x_" + std::to_string(i)).c_str());
		e[i].setName(("flowE_" + std::to_string(i)).c_str());
		u[i].setName(("u_" + std::to_string(i)).c_str());
	}
	for (int i = 0; i < t - 1; ++i) {
		lX[i].setName(("lX_" + std::to_string(i)).c_str());
		lE[i].setName(("lE_" + std::to_string(i)).c_str());
		mX[i].setName(("mX_" + std::to_string(i)).c_str());
		mE[i].setName(("mE_" + std::to_string(i)).c_str());
		mU[i].setName(("mU_" + std::to_string(i)).c_str());
		b[i].setName(("b_" + std::to_string(i)).c_str());
	}
	for (int i = 0; i < t; ++i) {
		wX_minus[i].setName(("wXminus_" + std::to_string(i)).c_str());
		wX_plus[i].setName(("wXplus_" + std::to_string(i)).c_str());
		wE_minus[i].setName(("wEminus_" + std::to_string(i)).c_str());
		wE_plus[i].setName(("wEplus_" + std::to_string(i)).c_str());
		y[i].setName(("y_" + std::to_string(i)).c_str());
		z[i].setName(("z_" + std::to_string(i)).c_str());
		F[i].setName(("F_" + std::to_string(i)).c_str());
	}*/

	IloExpr objective(env);
	for (int i = 0; i < t; i++)
		objective += F[i];
	model.add(IloMinimize(env, objective));
	objective.end();

	Node* first = nodes[1];
	IloConstraint continue_first;
	if(first->is_chargeable)
		continue_first = -F[0] + wX_plus[0] + wX_minus[0] + wE_plus[0] + wE_minus[0] - z[0] == 0;
	else
		continue_first = -F[0] + wX_plus[0] + wX_minus[0] + wE_plus[0] + wE_minus[0] + first->h_u_i0 - y[0] + z[0] == 0;
	constraints.add(continue_first);

	/*for (int i = 1; i < t; i++)
	{
		Node* n = nodes[i + 1];

		IloExpr expr = -F[i] + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i];
		if (!n->is_chargeable)
			expr += n->h_u_i0 - y[i] + z[i];
		else expr += - z[i];

		constraints.add(expr >= -M_cn[i-1] * b[i-1]);
		constraints.add(expr <=  M_cn[i-1] * b[i-1]);
		expr.end();
	}

	for (int i = 1; i < t; i++)
	{
		Node * curr = nodes[i]; Node * dep = nodes[0]; Node * next = nodes[i+1];
		double r = prob->GetDist(curr, dep) + prob->GetDist(dep, next) - prob->GetDist(curr, next);

		IloExpr expr = -F[i] + r + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + next->h_u_i0 - y[i];
		constraints.add(expr >= -M_r[i-1] * (1 - b[i-1]));
		constraints.add(expr <=  M_r[i-1] * (1 - b[i-1]));
		expr.end();
	}*/
	
	for (int i = 1; i < t; i++)
	{
		Node* curr = nodes[i];
		Node* dep = nodes[0];
		Node* next = nodes[i+1];
		double r = prob->GetDist(curr, dep) + prob->GetDist(dep, next) - prob->GetDist(curr, next);

		IloExpr expr_C(env);
		expr_C = wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i];
		if (!next->is_chargeable)
			expr_C += next->h_u_i0 - y[i] + z[i];
		else
			expr_C += -z[i];

		IloExpr expr_R(env);
		expr_R = r + wX_plus[i] + wX_minus[i] + wE_plus[i] + wE_minus[i] + next->h_u_i0 - y[i];

		// Reformulated cost constraint using delta = expr_R - expr_C
		//IloExpr delta(env);
		//delta = expr_R - expr_C;

		constraints.add(F[i] == (1-b[i-1])*expr_C + b[i-1] * expr_R);

		expr_C.end();
		expr_R.end();
		//delta.end();
	}

	
	for (int i = 1; i < t; i++)
	{
		Node * curr = nodes[i]; Node * dep = nodes[0]; Node * next = nodes[i+1];
		double r = prob->GetDist(curr, dep) + prob->GetDist(dep, next) - prob->GetDist(curr, next);
		double sum_dist = 0.0;
		for (int j = 1; j < i; j++)
			sum_dist += prob->GetDist(nodes[j-1], nodes[j]);
		if (r + sum_dist > Parameters::MaxRouteDistance())
			constraints.add(b[i-1] == 0);		
	}
	
	// Initial flows
	constraints.add(-x[0] + x[1] + wX_plus[0] - wX_minus[0] == first->q);
	constraints.add(-e[0] + e[1] + wE_plus[0] - wE_minus[0] == first->q_e);
	constraints.add(-u[0] + u[1] + z[0] - y[0] == 0);
	//wX_minus[1], wX_plus[1] are from the second node!
	for (int i = 2; i < t; ++i) {
		Node* node = nodes[i];
		
		//x[2] = x[1] + nodes[2]->q + wX_minus[1] - wX_plus[1] + lX[0]
		constraints.add( x[i] == x[i-1] + nodes[i]->q + wX_minus[i-1] - wX_plus[i-1] + lX[i-2] );
		//x[1] = x[0] + nodes[1]->q + wX_minus[0] - wX_plus[0] - mX[0]
		constraints.add( x[i-1] == x[i-2] + nodes[i-1]->q + wX_minus[i-2] - wX_plus[i-2] - mX[i-2] );
		
		//e[2] = e[1] + nodes[2]->q_e + wE_minus[1] - wE_plus[1] + lE[0]
		constraints.add( e[i] == e[i-1] + nodes[i]->q_e + wE_minus[i-1] - wE_plus[i-1] + lE[i-2] );
		//e[1] = e[0] + nodes[1]->q_e + wE_minus[0] - wE_plus[0] - mE[0]
		constraints.add( e[i-1] == e[i-2] + nodes[i-1]->q_e + wE_minus[i-2] - wE_plus[i-2] - mE[i-2] );
		
		//u[2] = u[1] - z[1] + y[1] + lU[0], lU[0] == 0 
		constraints.add( u[i] == u[i-1] - z[i-1] + y[i-1] );
		//u[1] = u[0] - z[0] + y[0] - mU[0]
		constraints.add( u[i-1] == u[i-2] - z[i-2] + y[i-2] - mU[i-2] ); 
	}

	// Flows at the last node i = t
	{
		Node* lastNode = nodes[t];
		int i = t;

		constraints.add(-x[i - 1] + x[i] + wX_plus[i - 1] - wX_minus[i - 1] == lastNode->q);
		constraints.add(-e[i - 1] + e[i] + wE_plus[i - 1] - wE_minus[i - 1] == lastNode->q_e);
		constraints.add(u[i] + z[i - 1] - y[i - 1] - u[i - 1] == 0);
	}


	// Logic constraints
	for (int i = 0; i < t - 1; ++i) {
		constraints.add(lX[i] <= Q * b[i]);
		constraints.add(lE[i] <= Q * b[i]);
		constraints.add(mX[i] <= Q * b[i]);
		constraints.add(mE[i] <= Q * b[i]);

		// Uncharged logic
		//constraints.add( mU[i] == u[i]*(b[i]) );
		constraints.add(mU[i] >= u[i] - Q * (1 - b[i]));
		constraints.add(mU[i] <= u[i]);
	}

	// Missed requests and station limits
	for (int i = 0; i < t; ++i) {
		Node* n = nodes[i + 1];
		constraints.add(z[i] <= u[i]);
		constraints.add(wX_minus[i] + wE_minus[i] + z[i] <= n->maxWm);
		constraints.add(wX_plus[i] <= n->h_i0);
		constraints.add(wE_plus[i] <= n->h_e_i0);
		constraints.add(y[i] <= n->h_u_i0);
	}

	// Capacity constraints
	for (int i = 0; i <= t; ++i)
		constraints.add(x[i] + e[i] + u[i] <= Q);
	for (int i = 0; i < t - 1; ++i)
		constraints.add(lX[i] + lE[i] <= Q);

	model.add(constraints);
	
	cplex_restock = IloCplex(model);
	cplex_restock.setParam(IloCplex::Param::Threads, 1);
	cplex_restock.setOut(env.getNullStream());
	//If you have non-linearities, you need the barrier algorithm
	//cplex_restock.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Dual); //or Dual
	//cplex_restock.setParam(IloCplex::NodeAlg, IloCplex::Dual); //or Dual


	if (!cplex_restock.solve())
		return 999999.0;
	
	//To dbg
	//If you're debugging in main(), you need to turn this on! Specifically, the nb_restocks
	restocked = false; nb_restocks = 0;
	for (int i = 0; i < t - 1; i++)
		if ((int)(cplex_restock.getValue(b[i]) + 0.1))
		{
			restocked = true; nb_restocks++;
		}
	
	if (_show)
	{
		cplex_restock.exportModel("restockLM.lp");
		printf("LM cplex optimal values ... Restock cost: %.1lf restocked: %d nb_restocks: %d\n",
			(double)cplex_restock.getObjValue(), restocked, nb_restocks);
		
		printf("restock distances:\n");
		for (int i = 1; i < t; i++)
		{
			Node * curr = nodes[i]; Node * dep = nodes[0]; Node * next = nodes[i+1];
			double r = prob->GetDist(curr, dep) + prob->GetDist(dep, next) - prob->GetDist(curr, next);
			printf("%.1lf-",r);
		}
		printf("\n");
		printf("M_cn:\n");
		for (size_t i = 0; i < M_cn.size(); i++)
			printf("%.1lf-", M_cn[i]);
		printf("\n");

		printf("M_r:\n");
		for (size_t i = 0; i < M_r.size(); i++)
			printf("%.1lf-", M_r[i]);
		printf("\n");

		for (int i = 0; i < t - 1; i++)
			std::cout << "b[" << i << "] = " << cplex_restock.getValue(b[i]) << ", ";
		printf("\n");

		for (int i = 0; i < t; i++)
			std::cout << "F[" << i << "] = " << std::setprecision(2) << cplex_restock.getValue(F[i]) << ", ";
		printf("\n");

		for (int i = 0; i < t - 1; i++) {
			std::cout << "lX[" << i << "] = " << (int)cplex_restock.getValue(lX[i]) << ", ";
			std::cout << "lE[" << i << "] = " << (int)cplex_restock.getValue(lE[i]) << ", ";
			std::cout << "mX[" << i << "] = " << (int)cplex_restock.getValue(mX[i]) << ", ";
			std::cout << "mE[" << i << "] = " << (int)cplex_restock.getValue(mE[i]) << ", ";
			std::cout << "mU[" << i << "] = " << (int)cplex_restock.getValue(mU[i]) << std::endl;
		}

		for (int i = 0; i <= t; ++i) {
			std::cout << "x[" << i << "] = " << (int)cplex_restock.getValue(x[i]) << ", ";
			std::cout << "e[" << i << "] = " << (int)cplex_restock.getValue(e[i]) << ", ";
			std::cout << "u[" << i << "] = " << (int)cplex_restock.getValue(u[i]) << std::endl;
		}

		for (int i = 0; i < t; ++i) {
			std::cout << "wX_minus[" << i << "] = " << (int)cplex_restock.getValue(wX_minus[i]) << ", ";
			std::cout << "wX_plus[" << i << "] = " << (int)cplex_restock.getValue(wX_plus[i]) << ", ";
			std::cout << "wE_minus[" << i << "] = " << (int)cplex_restock.getValue(wE_minus[i]) << ", ";
			std::cout << "wE_plus[" << i << "] = " << (int)cplex_restock.getValue(wE_plus[i]) << ", ";
			std::cout << "y[" << i << "] = " << (int)cplex_restock.getValue(y[i]) << ", ";
			std::cout << "z[" << i << "] = " << (int)cplex_restock.getValue(z[i]) << std::endl;
		}

		if (restocked)
		{
			printf("Restocked! check for silent bugs ....\n");
			// getchar();
		}
	}

	return cplex_restock.getObjValue();	
}

//To compute the cost
int RouteFeasibility::CalculateContinueToNextMIP(std::vector<Node*>& nodes, int Q, int delta, IloEnv env) 
{
    int t =  nodes.size() - 2; // t is without counting depots
	if(t < 1) return 0.0;
	
    model = IloModel(env);
	
    // Variables with size t+1 for flows
    x = IloNumVarArray(env, t + 1, 0, Q, ILOINT);
    e = IloNumVarArray(env, t + 1, 0, Q, ILOINT);
    u = IloNumVarArray(env, t + 1, 0, Q, ILOINT);

    // Variables with size t for weights and decisions
    wX_minus = IloNumVarArray (env, t, 0, IloInfinity, ILOINT);
    wX_plus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wE_minus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    wE_plus = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    y = IloNumVarArray(env, t, 0, IloInfinity, ILOINT);
    z = IloNumVarArray(env, t, 0, Q, ILOINT);

    // Objective function
    IloExpr objExpr(env);
    for (int i = 0; i < t; ++i) {
        Node* node = nodes[i+1];
        objExpr += wX_minus[i] + wX_plus[i] + wE_minus[i] + wE_plus[i];
        if (node->is_chargeable)
            objExpr -= z[i]; // Subtract for chargeable stations
        else
            objExpr += (node->h_u_i0 - y[i] + z[i]);
    }
    model.add(IloMinimize(env, objExpr));
	objExpr.end();

    // Add constraints
	for (int i = 1; i < t; i++) {
		Node* node = nodes[i];
		
		model.add( -x[i - 1] + x[i] + wX_plus[i - 1] - wX_minus[i - 1] == node->q );
		model.add( -e[i - 1] + e[i] + wE_plus[i - 1] - wE_minus[i - 1] == node->q_e );
		model.add( u[i] + z[i - 1] - y[i - 1] - u[i - 1] == 0 );
	}	
 
    // Flows at the last node
	// printf("t:%d nodes.size():%d\n",t,(int)nodes.size());
    Node* lastNode = nodes[nodes.size()-2];
    model.add(-x[t - 1] + x[t] + wX_plus[t - 1] - wX_minus[t - 1] == lastNode->q);
    model.add(-e[t - 1] + e[t] + wE_plus[t - 1] - wE_minus[t - 1] == lastNode->q_e);
    model.add(u[t] + z[t - 1] - y[t - 1] - u[t - 1] == 0);

    // Capacity constraints
    for (int i = 0; i < t; i++) {
        Node* node = nodes[i+1];
        model.add(x[i] + e[i] + u[i] <= Q);
        model.add(z[i] - u[i] <= 0);
        model.add(wX_minus[i] + wE_minus[i] + z[i] <= node->maxWm);
        model.add(wX_plus[i] <= node->h_i0);
        model.add(wE_plus[i] <= node->h_e_i0);
        model.add(y[i] <= node->h_u_i0);
    }
    model.add(x[t] + e[t] + u[t] <= Q);
    model.add(u[0] == 0);

    // Solve the LP
    cplex_continue = IloCplex(model);
    cplex_continue.setOut(env.getNullStream());
	cplex_continue.setParam(IloCplex::Param::Threads, 1);
    //cplex.exportModel("route1.lp");
	//printf("Printing the lp\n");
    if (!cplex_continue.solve())
        return 999999; 


    return (int)cplex_continue.getObjValue();
}


void RouteFeasibility::CalculateBigM(const std::vector<Node*> & nodes, int Q, std::vector<double> & M_cn, std::vector<double> & M_r, bool show)
{
	int t =  nodes.size() - 2; // t is without counting depots
	
	std::vector<double> L_r(t-1,0); std::vector<double> L_cn(t-1);
	std::vector<double> U_r(t-1); std::vector<double> U_cn(t-1);
	
	if(t>1)
		for(int i=1;i<t;i++)
		{
			Node * curr = nodes[i];
			Node * dep = nodes[0];
			Node * next = nodes[i+1];
			double r = prob->GetDist(curr,dep) + prob->GetDist(dep,next) - prob->GetDist(curr,next); 
			//U_r[i-1] = r + curr->maxWm + curr->h_i0 + curr->h_e_i0 + curr->h_u_i0;
			U_r[i-1] = r + next->maxWm + next->h_i0 + next->h_e_i0 + next->h_u_i0;
			if(show) printf("i%d Node:%d char:%d h_i:%d h_e_i:%d h_u_i:%d Wm:%d L_cn:%.1lf U_R:%.1lf\n",i,next->id,next->is_chargeable,next->h_i0,next->h_e_i0,next->h_u_i0,next->maxWm,L_r[i-1],U_r[i-1]);
		}
	
	if(t>1)
		for(int i=1;i<t;i++)
		{
			//The first station in the path does not need bounds ...
			Node * curr = nodes[i+1]; //The second station in the path. First curr is nodes[2] ...
			
			if(curr->is_chargeable)
			{
				U_cn[i-1] = curr->maxWm + curr->h_i0 + curr->h_e_i0;
				L_cn[i-1] = -curr->maxWm;
			} else {
				U_cn[i-1] = curr->maxWm + curr->h_i0 + curr->h_e_i0 + curr->h_u_i0;
				L_cn[i-1] = 0;
			}
			if(show) printf("i:%d Node:%d char:%d h_i:%d h_e_i:%d h_u_i:%d Wm:%d L_cn:%.1lf U_CN:%.1lf\n",i,curr->id,curr->is_chargeable,curr->h_i0,curr->h_e_i0,curr->h_u_i0,curr->maxWm,L_cn[i-1],U_cn[i-1]);
		}
	
	M_r.clear(); M_cn.clear();
	M_r.reserve(t);
	M_cn.reserve(t);
	for(int i=0;i<t-1;i++)
	{
		double restock = U_r[i] - L_r[i];
		double cn = U_cn[i] - L_cn[i];
		M_r.emplace_back( restock );
		M_cn.emplace_back( cn );
	}
	
	if(show)
	{
		printf("CalculateBigM t:%d\n",t);
		printf("Bounds CN:\n");
		for(size_t i =0;i<U_cn.size();i++)
			printf("[%.1lf,%.1lf]-",L_cn[i],U_cn[i]);
		printf("\n");
		printf("Bounds R:\n");
		for(size_t i =0;i<U_r.size();i++)
			printf("[%.1lf,%.1lf]-",L_r[i],U_r[i]);
		printf("\n");
		printf("M_cn:\n");
		for(size_t i =0;i<M_cn.size();i++)
			printf("%.1lf-",M_cn[i]);
		printf("\n");
		printf("M_r:\n");
		for(size_t i =0;i<M_r.size();i++)
			printf("%.1lf-",M_r[i]);
		printf("\n");
	}	
}

bool RouteFeasibility::EndLoadHybrid(std::vector<Node*>& nodes, int Q, bool show) // End load feasibility (Regular + Electric)
{
    int lambda_hybrid = 0; 
    int min_lambda_hybrid = 0;
    int mu_hybrid = 0; 
    int max_mu_hybrid = 0;

    for (const Node* node : nodes) 
    {
        if (node->type != NODE_TYPE_CUSTOMER) continue; // Ignore depots

        // Combined demand using both regular and electric components
        int combined_demand = node->q + node->q_e;
        int slack_hybrid = combined_demand - node->h_i0 - node->h_e_i0; // Using h_e_i0 as electric counterpart
        int surplus_hybrid = combined_demand + node->maxWm;

        lambda_hybrid += slack_hybrid; 
        min_lambda_hybrid = std::min(lambda_hybrid, min_lambda_hybrid);
        mu_hybrid += surplus_hybrid; 
        max_mu_hybrid = std::max(mu_hybrid, max_mu_hybrid);

        int lb_hybrid = lambda_hybrid - min_lambda_hybrid;
        int ub_hybrid = mu_hybrid + Q - max_mu_hybrid;

        if (show)
        {
            printf("EndLoadHybrid Lb>Ub?%d no:%d lb:%d ub:%d dmd:%d wp:%d wm:%d lambda:%d mu:%d lambda_hybrid:%d mu_hybrid:%d min_lambda_hybrid:%d max_mu_hybrid:%d\n",
                   lb_hybrid > ub_hybrid, node->id, lb_hybrid, ub_hybrid, combined_demand, node->h_i0 + node->h_e_i0, node->maxWm, slack_hybrid, surplus_hybrid, 
                   lambda_hybrid, mu_hybrid, min_lambda_hybrid, max_mu_hybrid);
        }

        if (lb_hybrid > ub_hybrid)
            return false;
    }

    return true;
}

bool RouteFeasibility::HasZeroHCUnchargedViolations(std::vector<Node*>& nodes, int Q, bool show)
{
	if(show) printf("Beginning HasZeroRecUnchargedViolations ...\n");
	
	int init_q = 0, init_qe = 0;
	for(size_t i=0;i<nodes.size();i++)
	{
		Node * n = nodes[i];
		if(n->type != NODE_TYPE_CUSTOMER) continue;
		init_q += n->q;
		init_qe += n->q_e;
	}
	
	if(init_q > Q || init_q < -Q || init_qe > Q || init_qe < -Q) return false;
	if(init_q+init_qe > Q || init_q+init_qe < -Q) return false;
	if(init_q > 0) init_q = 0;
	if(init_qe > 0) init_qe = 0;

	init_q = -init_q;
	init_qe = -init_qe;
	
	int cur_q = init_q, cur_qe = init_qe;
	if(show) printf("init_q:%d init_qe:%d\n",init_q,init_qe);

	int cur_u = 0;
	for (size_t i = 0; i < nodes.size(); i++) {
        Node* n = nodes[i];

        if (n->type != NODE_TYPE_CUSTOMER) continue; 
		
        cur_q += n->q;
        cur_qe += n->q_e;
        cur_u += n->h_u_i0;  // Accumulate h_u_i0. If you cannot pick them all, you incur in a violation. This is checked below ...

        int sum = cur_q + cur_qe + cur_u;
        int cur_violation = 0;
        if (sum > Q) cur_violation = sum - Q;
        else if (sum < 0) cur_violation = -sum;

        // Adjust for chargeable nodes
        
		//This seems to work ...
		int reduced_violation = 0;
		
		if (n->is_chargeable && cur_violation) {
			int adjustment = std::min(cur_violation, cur_u); // Only use what we can
			cur_u -= adjustment;  // Reduce cur_u safely
			reduced_violation = cur_violation - adjustment;  // Remaining violation after cur_u adjustment
			cur_violation = std::max(reduced_violation, 0);  // Ensure it's non-negative
		}
		/*if (n->is_chargeable && cur_violation) {
			reduced_violation = std::max(cur_violation - cur_u, 0);
            cur_u -= cur_violation;  
            cur_u = std::max(cur_u, 0);  
            cur_violation = reduced_violation;
        }*/ else if (n->is_chargeable && !cur_violation) {
			reduced_violation = std::min(cur_u, n->maxWm);  
			cur_u = std::max(cur_u - n->maxWm, 0);
		}
		
		//Feasibility checks
		if (reduced_violation > 0) return false;
        if (cur_violation > 0 || cur_q > Q || cur_q < 0 || cur_qe > Q || cur_qe < 0 || cur_u > Q) return false;
		if(sum > Q || sum < 0) return false;

		if (show) {
			printf(
				"Id: %d | sum: %d | cur_q: %d | cur_qe: %d | cur_u: %d | cur_violation: %d | reduced_violation: %d\n"
				"     wpR: %d | wpE: %d | wpU: %d | wm: %d | Chargeable: %d\n",
				n->id, sum, cur_q, cur_qe, cur_u, cur_violation, reduced_violation,
				n->h_i0, n->h_e_i0, n->h_u_i0, n->maxWm, n->is_chargeable ? 1 : 0
			);
		}
    }
	
	if(show) printf("The sequence of stations should have Zero recourse ...\n");

	return true;
}

bool RouteFeasibility::HasZeroHCUnchargedViolations(std::vector<Node*>& nodes, int Q, bool show, int init_q, int init_qe)
{
	if(show) printf("Beginning HasZeroRecUnchargedViolations ...\n");
	
	if(init_q > 0) init_q = 0;
	if(init_qe > 0) init_qe = 0;

	init_q = -init_q;
	init_qe = -init_qe;
	
	int cur_q = init_q, cur_qe = init_qe;
	if(show) printf("init_q:%d init_qe:%d\n",init_q,init_qe);

	int cur_u = 0;
	for (size_t i = 0; i < nodes.size(); i++) {
        Node* n = nodes[i];

        if (n->type != NODE_TYPE_CUSTOMER) continue; 
		
        cur_q += n->q;
        cur_qe += n->q_e;
        cur_u += n->h_u_i0;  // Accumulate h_u_i0. If you cannot pick them all, you incur in a violation. This is checked below ...

        int sum = cur_q + cur_qe + cur_u;
        int cur_violation = 0;
        if (sum > Q) cur_violation = sum - Q;
        else if (sum < 0) cur_violation = -sum;

        // Adjust for chargeable nodes
        
		//This seems to work ...
		int reduced_violation = 0;
		
		if (n->is_chargeable && cur_violation) {
			int adjustment = std::min(cur_violation, cur_u); // Only use what we can
			cur_u -= adjustment;  // Reduce cur_u safely
			reduced_violation = cur_violation - adjustment;  // Remaining violation after cur_u adjustment
			cur_violation = std::max(reduced_violation, 0);  // Ensure it's non-negative
		}
		/*if (n->is_chargeable && cur_violation) {
			reduced_violation = std::max(cur_violation - cur_u, 0);
            cur_u -= cur_violation;  
            cur_u = std::max(cur_u, 0);  
            cur_violation = reduced_violation;
        }*/ else if (n->is_chargeable && !cur_violation) {
			reduced_violation = std::min(cur_u, n->maxWm);  
			cur_u = std::max(cur_u - n->maxWm, 0);
		}
		
		//Feasibility checks
		if (reduced_violation > 0) return false;
        if (cur_violation > 0 || cur_q > Q || cur_q < 0 || cur_qe > Q || cur_qe < 0 || cur_u > Q) return false;
		if(sum > Q || sum < 0) return false;

		if (show) {
			printf(
				"Id: %d | sum: %d | cur_q: %d | cur_qe: %d | cur_u: %d | cur_violation: %d | reduced_violation: %d\n"
				"     wpR: %d | wpE: %d | wpU: %d | wm: %d | Chargeable: %d\n",
				n->id, sum, cur_q, cur_qe, cur_u, cur_violation, reduced_violation,
				n->h_i0, n->h_e_i0, n->h_u_i0, n->maxWm, n->is_chargeable ? 1 : 0
			);
		}
    }
	
	if(show) printf("The sequence of stations should have Zero recourse ...\n");

	return true;
}


bool RouteFeasibility::HasZeroHCUncharged(std::vector<Node*>& nodes, int Q, bool show)
{
	if(show) printf("Beginning HasZeroRecUncharged ...\n");
	
	int init_q = 0, init_qe = 0;
	for(size_t i=0;i<nodes.size();i++)
	{
		Node * n = nodes[i];
		if(n->type != NODE_TYPE_CUSTOMER) continue;
		init_q += n->q;
		init_qe += n->q_e;
	}
	
	if(init_q > Q || init_q < -Q || init_qe > Q || init_qe < -Q) return false;
	if(init_q+init_qe > Q || init_q+init_qe < -Q) return false;
	if(init_q > 0) init_q = 0;
	if(init_qe > 0) init_qe = 0;

	init_q = -init_q;
	init_qe = -init_qe;
	
	int cur_q = init_q, cur_qe = init_qe;
	if(show) printf("init_q:%d init_qe:%d\n",init_q,init_qe);

	//Simple check for uncharged
	int cur_u = 0; 	
	for(size_t i=0;i<nodes.size();i++)
	{
		Node * n = nodes[i];
		if(n->type != NODE_TYPE_CUSTOMER) continue;
		cur_q += n->q;
		cur_qe += n->q_e;
		cur_u += n->h_u_i0;
		if(n->is_chargeable && cur_u) return false; //You will drop the bikes and potentially make the objective negative
		if(cur_q > Q || cur_q < 0 || cur_qe > Q || cur_qe < 0) return false;
		if(cur_q+cur_qe+cur_u > Q || cur_q+cur_qe+cur_u < 0) return false;
		
		if(show)
			printf("Id:%d curqR:%d curqE:%d curU:%d wpR:%d wpE:%d wpU:%d wm:%d Char:%d\n",n->id,cur_q,cur_qe,cur_u,n->h_i0,n->h_e_i0,n->h_u_i0,n->maxWm,n->is_chargeable);
	}
	
	if(show) printf("The sequence of stations should have Zero recourse ...\n");

	return true;
}

bool RouteFeasibility::HasZeroHCUncharged(std::vector<Node*>& nodes, int Q, bool show, int init_q, int init_qe)
{
	if(show) printf("Beginning HasZeroRecUncharged ...\n");
	
	if(init_q > 0) init_q = 0;
	if(init_qe > 0) init_qe = 0;

	init_q = -init_q;
	init_qe = -init_qe;
	
	int cur_q = init_q, cur_qe = init_qe;
	if(show) printf("init_q:%d init_qe:%d\n",init_q,init_qe);

	//Simple check for uncharged
	int cur_u = 0; 	
	for(size_t i=0;i<nodes.size();i++)
	{
		Node * n = nodes[i];
		if(n->type != NODE_TYPE_CUSTOMER) continue;
		cur_q += n->q;
		cur_qe += n->q_e;
		cur_u += n->h_u_i0;
		if(n->is_chargeable && cur_u) return false; //You will drop the bikes and potentially make the objective negative
		if(cur_q > Q || cur_q < 0 || cur_qe > Q || cur_qe < 0) return false;
		if(cur_q+cur_qe+cur_u > Q || cur_q+cur_qe+cur_u < 0) return false;
		
		if(show)
			printf("Id:%d curqR:%d curqE:%d curU:%d wpR:%d wpE:%d wpU:%d wm:%d Char:%d\n",n->id,cur_q,cur_qe,cur_u,n->h_i0,n->h_e_i0,n->h_u_i0,n->maxWm,n->is_chargeable);
	}
	
	if(show) printf("The sequence of stations should have Zero recourse ...\n");

	return true;
}

double RouteFeasibility::CalculateSequenceLb(std::vector<Node*>& nodes, int Q, int delta)
{
    double max_L = 99999.0;
	int n = nodes.size();
	// Generate all contiguous subsequences 
    for (int start = 0; start < n; start++) {
        for (int end = start; end < n; end++) {
            std::vector<Node*> subseq;
			subseq.reserve(n);
            for (int i = start; i <= end; i++) {
                subseq.push_back(nodes[i]);
            }
			
			int sum_u = 0; int sum_q = 0; int sum_q_e = 0;
			for(auto & node : subseq)
			{
				sum_u += node->h_u_i0; sum_q += node->q; sum_q_e += sum_q_e; 
			}
			
			//double L = std::abs(sum_q) + std::abs(sum_q_e) + sum_u;
			//L -= Q;
			
			double L = std::max( 0, std::abs(sum_q + sum_q_e) - Q) - sum_u;
			
			max_L = std::min( max_L, L );
			
			//std::cout << "Current Subsequence: ";
            //for (auto & node : subseq) {
            //    std::cout << "(" << node->q << ", " << node->q_e << ", " << node->h_u_i0 << ", " << Q << ") ";
            //}
            //std::cout << "| L: " << L << " | max_L: " << max_L << std::endl;
        }
    }
	return max_L;
}

int RouteFeasibility::CalculateContinueToNextDP(std::vector<Node*>& nodes, int Q, int delta)
{
	int t =  nodes.size() - 2; // t is without counting depots
	if(t < 1) return 0.0;
	
    double inf = 99999;

    // DP table: dp[k][x][e][u] holds the minimum cost at node k with combined load (x, e, u)
    std::vector<std::vector<std::vector<std::vector<double>>>> dp(
        t + 2, std::vector<std::vector<std::vector<double>>>(
            Q + 1, std::vector<std::vector<double>>(
                Q + 1, std::vector<double>(
                    Q + 1, inf
                )
            )
        )
    );

    // Initialization of the final state
    for (int x = 0; x <= Q; ++x)
	{
		for (int e = 0; e <= Q - x; ++e)
            for (int u = 0; u <= Q - x - e; ++u)
				dp[t + 1][x][e][u] = 0;
	}
        
    // Compute Total1, Total2
	/*int Total1 =  0, Total2 =0;
    for (int x = 0; x <= Q; x++) {
        for (int e = 0; e <= Q - x; e++) {
            Total1++;
        }
    }

    for (int x = 0; x <= Q; x++) {
        for (int e = 0; e <= Q - x; e++) {
            for (int u = 0; u <= Q - x - e; u++) {
                for (int z_k = 0; z_k <= u; z_k++) {
                    Total2++;
                }
            }
        }
    }*/

	double global_min_cost = inf;
	int skipped1 = 0, skipped2 = 0, skipped3 = 0;

	for (int k = t; k >= 1; --k) {	
		Node* node = nodes[k]; // For vectors with depots
		//printf("Beginning loop at node:\n");
		//node->Show();

        for (int x = 0; x <= Q; x++) {
            for (int e = 0; e <= Q - x; e++) {
				
				bool skip = SkipForGivenXE(node, x, e);
				if (skip) {
					skipped1++; 
					continue;
				}

                for (int u = 0; u <= Q - x - e; u++) {
                    double min_cost = inf;
					
					bool skip_summed = CheckSummedInequality(node, x, e, node->maxWm, u);
					if (skip_summed) {
						skipped2++;  
						continue;
					}
									
                    // Evaluate feasible combinations
                    for (int z_k = 0; z_k <= u; z_k++) {
						/*bool skip_summed2 = CheckSummedInequality(node, x, e, node->maxWm, z_k);
						if (skip_summed2) {
							skipped3++;  
							continue;
						}*/
						
						for (int w_minus_x = 0; w_minus_x <= node->maxWm - z_k; w_minus_x++) {
							for (int w_minus_e = 0; w_minus_e <= node->maxWm - z_k - w_minus_x; w_minus_e++) {						
                                for (int w_plus_x = 0; w_plus_x <= node->h_i0; w_plus_x++) {
                                    for (int w_plus_e = 0; w_plus_e <= node->h_e_i0; w_plus_e++) {
                                        int new_x = x + node->q + w_minus_x - w_plus_x;
                                        int new_e = e + node->q_e + w_minus_e - w_plus_e;
                                        int new_u = u - z_k;
										if (new_x < 0 || new_e < 0 || new_u < 0 || new_x + new_e + new_u > Q || w_minus_x + w_minus_e + z_k > node->maxWm)
                                            continue;

                                        if (node->is_chargeable) {
                                            double total_cost_chargeable = w_minus_x + w_minus_e + w_plus_x + w_plus_e - z_k + dp[k + 1][new_x][new_e][new_u];

                                            if (total_cost_chargeable < min_cost) {
                                                min_cost = total_cost_chargeable;
                                                dp[k][x][e][u] = min_cost;
                                            }
                                        } else { // Non-chargeable case
                                            for (int y_k = 0; y_k <= node->h_u_i0; y_k++) {
												new_u = u - z_k + y_k;
                                                if (new_u < 0 || new_x + new_e + new_u > Q) continue;

                                                double total_cost_nonchargeable = w_minus_x + w_minus_e + w_plus_x + w_plus_e + node->h_u_i0 - y_k + z_k + dp[k + 1][new_x][new_e][new_u];

                                                if (total_cost_nonchargeable < min_cost) {
                                                    min_cost = total_cost_nonchargeable;
                                                    dp[k][x][e][u] = min_cost;
                                                }
                                            }
                                        }
										// Track minimum cost for dp[1][x][e][0]
										if (k == 1 && u == 0 && min_cost < global_min_cost) {
											global_min_cost = min_cost;
										}
										
                                    }
                                }
                            }
                        }
                    }

					
				}
				//if(min_cost > 9999.0)
				//if(min_cost < 9999.0 && skip)
				//if(skip)
				//{
					//printf("Position:%d x:%d e:%d\n",k,x,e); 
					//node->Show();
					//getchar();
				//}
            }
        }
    }
	//std::cout << "States skipped1:" << skipped1 << " skipped2:" << skipped2 /*<< " skipped3:" << skipped3 */ << std::endl;
	//printf("\nStates skipped1:%d/%d skipped2:%d/%d\n",skipped1,Total1,skipped2,Total2);
	return global_min_cost;

}


bool RouteFeasibility::CheckSummedInequality(Node* node, int x, int e, int W, int z_k) {

    //int rhs = (node->h_i0 + node->h_e_i0) - std::max(W - z_k, 0) - (node->q + node->q_e);

    //if (x + e < rhs) {
        //printf("INFEASIBLE SUM: x+e=%d, RHS=%d, x=%d, e=%d, W=%d, z_k=%d\n", 
        //       x+e, rhs, x, e, W, z_k);
    //   return false;
    //}

    int bound1X = x + node->q + std::max(W - z_k, 0);
    int bound2X = x + node->q + std::max(W - z_k, 0) - node->h_i0;

    int bound1E = e + node->q_e + std::max(W - z_k, 0);
    int bound2E = e + node->q_e + std::max(W - z_k, 0) - node->h_e_i0;

    bool feasible_x = (bound1X >= 0);  // Ensures w^{x-} exists
    bool feasible_e = (bound1E >= 0);  // Ensures w^{e-} exists

    if (!feasible_x || !feasible_e) {
        //printf("INFEASIBLE INDIVIDUAL: x=%d, e=%d, z=%d, q=%d, qE=%d, boundX=[%d, %d], boundE=[%d, %d], W=%d\n", 
        //       x, e, z_k,node->q,node->q_e, bound1X, bound2X, bound1E, bound2E, W);
        return true;
    }

    return false;
}

//Computes bounds on wE_minus and wX_minus
bool RouteFeasibility::SkipForGivenXE(Node* node, int x, int e) {
	
	bool skip = false;
	int bound1X = -node->q - x;
	int bound2X = -node->q - x + node->h_i0;
	
	int bound1E = -node->q_e - e;
	int bound2E = -node->q_e - e + node->h_e_i0;
	
	if ((bound1X > node->maxWm && bound2X > node->maxWm) || (bound1E > node->maxWm && bound2E > node->maxWm)) {
		skip = true;
	}

	//if (skip /*|| (node->maxWm < 5 && (node->q < 0 || node->q_e <0))*/) {
	//	printf("Skipping x=%d, e=%d, q=%d, q_e=%d\n", x, e, node->q, node->q_e);
	//	printf("X bounds: [%d, %d], maxWm: %d\n", bound1X, bound2X, node->maxWm);
	//	printf("E bounds: [%d, %d], maxWm: %d\n", bound1E, bound2E, node->maxWm);
	//}

	return skip;  
}