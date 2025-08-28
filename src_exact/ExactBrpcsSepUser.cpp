#include "ExactBrpcsSepUser.h"

ExactBrpSepUser::ExactBrpSepUser(IloEnv env, ExactBrpGraphO* graph, IloNumVarArray x, IloNumVar theta, RouteFeasibility * r) : _env(env), _graph(graph), _x(x), _theta(theta), _prob(graph->GetProblem()), _r(r)
{
	nb_inf_sets = 0;
	nb_inf_paths = 0;
	nb_sub_tours = 0;
	nb_sub_tour_frac = 0;
	nb_opt_cuts = 0;
	nb_infeasible_solution_cuts = 0;
	nb_max_route_cuts = 0;
	best_sol = best_sol_recourse = best_sol_distance = 9999999999;

	_component.resize(_graph->GetNodeCount(), -1);
}

//Separates inequalities from fractional solutions
void ExactBrpSepUser::SeparateFrac(IloRangeArray array)
{	
	
	for(int i = 0; i < _graph->GetNodeCount(); i++)//stations + depot
		_component[i] = -1;

	int comp = -1;
	for(int i=1;i<_graph->GetNodeCount();i++)
		if(_component[_graph->GetNode(i)->no] == -1)
			ResearchConnectedComponent(_graph->GetNode(i), ++comp);
	comp++;

	std::vector< std::vector<Node*> > components(comp);
	for(int i=0;i<comp;i++)
		components[i].reserve( _graph->GetNodeCount() );
	
	for(int i=1;i<_graph->GetNodeCount();i++)
	{
		Node * n = _graph->GetNode(i);
		components[ _component[n->no] ].push_back(n);
	}
	
	for(int i=0;i<comp;i++)
	{
		if(components[i].size()<=2) continue;//Added to the root
		
		double sumv = 0;
		int nb=components[i].size();

		
		IloExpr expr(array.getEnv());
		for(int m = 0; m < nb; m++)
			for(int n = 0; n < nb; n++)
				if (m != n)
				{
					ExBrpArcO* arc = _graph->GetArc(components[i][m]->no, components[i][n]->no);
					if (arc == NULL) continue;
					sumv += arc->value;
					expr += _x[arc->index];
				}
				
		if(sumv > nb - 1 + 0.2) //OG
		{
			//printf("lhs:%.1lf rhsCheck:%.1lf rhs:%d nb:%d nb_veh:%d\n",sumv,nb - 1 + 0.2,nb - nb_veh,nb,nb_veh);
			array.add(expr <= nb - 1);
			nb_sub_tour_frac++;

		}
		expr.end();
	}		
	
}

//Separates inequalities from integer solutions
void ExactBrpSepUser::SeparateInt(IloRangeArray array)
{
	if(!SeparateUserCapRecursive(array))
	{
		_graph->MakePaths(); 
		for(int i=0;i<_graph->GetPathCount();i++)
		{
			std::vector<Node*> path = _graph->GetPath(i);

			nb_inf_paths += TestAndAddInfeasiblePath(path,array); //Quick checks on infeasible paths
		}
	} 

}

bool ExactBrpSepUser::SeparateUserCapRecursive(IloRangeArray array)
{
	bool added_inq=false;
	
	for(int i = 0; i < _graph->GetNodeCount(); i++)
		_component[i] = -1;

	int comp = -1;
	for(int i = 0; i < _graph->GetNodeCount(); i++)
		if (_component[_graph->GetNode(i)->no] == -1)
			ResearchConnectedComponent(_graph->GetNode(i), ++comp);
	
	for(int co = 1; co <= comp; co++)
	{
		//get the ordered cycle
		std::vector<Node*> tour; tour.reserve( _graph->GetNodeCount() );
		for(int i=1;i<_graph->GetNodeCount();i++)
			if(_component[_graph->GetNode(i)->no] == co)
			{
				if(_graph->GetArcsOutPosCount( _graph->GetNode(i)->no ) == 0) continue;
				ExBrpArcO* ar = _graph->GetArcsOutPos(_graph->GetNode(i)->no, 0);
				tour.push_back(ar->from);

				Node* from = ar->from;
				Node* to = ar->to;
				while (1)
				{
					tour.push_back(to);
					from = to;
					ar = _graph->GetArcsOutPos(to->no, 0);
					to = ar->to;
					if(to->no == _graph->GetNode(i)->no) break;
				}
				break;
			}
		if (tour.size() == 0) continue;
		
		double sumv=0.0;
		IloExpr expr(array.getEnv());
		for(int m = 0; m < tour.size(); m++)
			for(int n = 0; n < tour.size(); n++)
				if (m != n)
				{
					ExBrpArcO* arc = _graph->GetArc(tour[m]->no, tour[n]->no);
					if (arc != NULL) expr += _x[arc->index];
					if (arc != NULL) sumv += arc->value;
				}
		double rhs = (tour.size()) - 1;

		array.add(expr <= rhs);
		nb_sub_tours++;
		expr.clear();
		added_inq=true;

		expr.end();

	}// End connected component
	
	return added_inq;

}

void ExactBrpSepUser::ResearchConnectedComponent(Node* n, int comp)
{
	_component[n->no] = comp;
	for(int i = 0; i < _graph->GetArcsOutPosCount(n->no); i++)
	{
		ExBrpArcO* a = _graph->GetArcsOutPos(n->no, i);
		if (a->to->no!=0 && _component[a->to->no] == -1 && a->value >= 0.45)//more subtours than inf set
		{
			_component[a->to->no] = comp;
			ResearchConnectedComponent(_graph->GetNode(a->to->no),comp);
		}
	}
}

void ExactBrpSepUser::ResearchDepotComponent(Node* n, int comp)
{
	_component[n->no] = comp;
	for(int i = 0; i < _graph->GetArcsOutPosCount(n->no); i++)
	{
		ExBrpArcO* a = _graph->GetArcsOutPos(n->no, i);
		if (_component[a->to->no] == -1 && a->value >= 0.45) 
		//if (_component[a->to->no] == -1 && a->value >= 0.60) //OG
		{
			_component[a->to->no] = comp;
			ResearchDepotComponent(_graph->GetNode(a->to->no),comp);
		}
	}
}

// Test if a the path is infeasible
// In such case, it searches for the smallest infeasible set
// The smallest infeasible path
// Note that the first and last nodes are the depot
int ExactBrpSepUser::TestAndAddInfeasiblePath(std::vector<Node*> & path, IloRangeArray array)
{
	if(path.size()<=3) return 0;

	int nb_inq = 0;
	int Q = _prob->GetDriver(0)->capacity;
	std::vector<Node*> subpath;
	
	double dist = 0.0;
	for(int i=1;i<path.size();i++)
		dist += _prob->GetDist(path[i-1],path[i]);
	
	if(RouteFeasibility::EndLoadHybrid(path,Q,false) || dist < Parameters::MaxRouteDistance())
		return 0;


	for(int k=1;k+2<path.size();k++)
	{
		bool found_inf = false;
		for(int j=1;j+2<path.size() && j+k+1<path.size();j++)
		{
			subpath.clear(); subpath.reserve( (int)path.size() );
			subpath.push_back(path[0]);
			
			double dist = 0.0;
			for(int l=j;l<=j+k && l+1 < path.size();l++)
			{
				subpath.push_back(path[l]); dist += _prob->GetDist(subpath[l-1],subpath[l]);
			}
			subpath.push_back(path.back());
			dist += _prob->GetDist(subpath[subpath.size()-2],subpath[subpath.size()-1]);
			
			if(RouteFeasibility::EndLoadHybrid(subpath,Q,false) || dist < Parameters::MaxRouteDistance()) // A feasible path
				continue;

			IloExpr expr(array.getEnv());
			int nb = 0; double sum_v=0.0;
			for(int l = 1; l < subpath.size(); l++)
			{
				if(subpath[l - 1]->type != NODE_TYPE_CUSTOMER || subpath[l]->type != NODE_TYPE_CUSTOMER) continue;

				ExBrpArcO* arc = _graph->GetArc(subpath[l - 1]->no, subpath[l]->no);
				if(arc != NULL)
				{
					expr += _x[arc->index];
					sum_v += arc->value;
					nb++;
				}
			}
			if(sum_v > nb - 1 + 0.2 )
			{
				array.add(expr <= nb-1);
				nb_inf_paths++;
				nb_inq++;
				expr.end();
				found_inf = true;				
			}

		}//end for j

		if(found_inf) break;
	}//end for k

	return nb_inq;
}


void ExactBrpSepUser::CutCurrentSolution(IloRangeArray array)
{
	//printf("CutCurrentSolution\n");
	//_graph->ShowPaths();
	IloExpr expr(array.getEnv());

	int rhs = 0;
	for(int i = 0; i < _graph->GetPosArcCount(); i++)
	{
		ExBrpArcO* a = _graph->GetPosArc(i);
		if(a->from->no == 0) continue;

		expr += _x[a->index];
		rhs++;
	}
	array.add(expr <= rhs - 2);
	expr.end();
	nb_infeasible_solution_cuts++;
}

int ExactBrpSepUser::AddInfeasiblePath(IloRangeArray array, std::vector<Node*> & path)
{
	IloExpr expr(array.getEnv());
	int nb = 0;
	for(size_t i=1;i<path.size();i++)
	{
		ExBrpArcO* arc = _graph->GetArc(path[i - 1]->no, path[i]->no);
		if(arc != NULL)
		{
			expr += _x[arc->index];
			nb++;
		}
	}
	if(nb)
		array.add( expr <= nb-1 );
	
	expr.end();
	
	return nb ? nb_inf_paths++ : 0;
		
		
}

void ExactBrpSepUser::SeparateOptCut(IloRangeArray array)
{
	int Q = _prob->GetDriver(0)->capacity;
	double cost = 0;
	std::vector<double> costs(_graph->GetPathCount(), 0);
	int infeasible_paths = 0;
	for(int i = 0; i < _graph->GetPathCount(); i++)
	{
		costs[i] = _r->CalculateContinueToNextMIP(_graph->GetPath(i),Q,1);
		cost += costs[i];
		if(costs[i] > 9990)
			infeasible_paths += AddInfeasiblePath(array,_graph->GetPath(i));
	}
	if(infeasible_paths)
		return;
		
	if(_graph->GetCost() + cost < best_sol)
	{
		best_sol_distance = _graph->GetCost();
		best_sol_recourse = cost;
		best_sol = best_sol_distance + best_sol_recourse;
		printf("New solution:%.3lf dist:%.1lf rec:%.3lf OptCuts:%d Subtours:%d InfSolCuts:%d InfPathCuts:%d\n", 
				best_sol, best_sol_distance, best_sol_recourse,nb_opt_cuts,
				nb_sub_tour_frac+nb_sub_tours,nb_infeasible_solution_cuts,nb_inf_paths);
		best_solution.clear();
		for(int i = 0; i < _graph->GetPathCount(); i++)
			best_solution.push_back( _graph->GetPath(i) );
	}

	{
		//add optimality cut
		IloExpr expr(array.getEnv());
		expr += _theta;

		int nb = 0;
		for(int i = 0; i < _graph->GetPathCount(); i++)
		{
			std::vector<Node*> & path = _graph->GetPath(i);
			if(costs[i] <= 0.0001) continue;

			for(int j = 1; j < path.size(); j++)
			{
				ExBrpArcO* arc = _graph->GetArc(path[j - 1]->no, path[j]->no);

				if(arc->from->type != NODE_TYPE_CUSTOMER || arc->to->type != NODE_TYPE_CUSTOMER) continue;
				expr -= cost * _x[arc->index];
				nb++;
			}
		}

		array.add(expr >= cost * (1 - nb));
		//std::cout << "OptCut: " << nb_opt_cuts << " Cost: " << cost << " Cut: " << expr << " >= " << cost * (1 - nb) << std::endl;
		expr.end();
		nb_opt_cuts++;
	}
	//CutCurrentSolution(array);
}

/*void ExactBrpSepUser::SeparateOptCut(IloRangeArray array)
{
	int Q = _prob->GetDriver(0)->capacity;
	double cost = 0;
	std::vector<double> costs(_graph->GetPathCount(), 0);
	bool is_feasible = true;
	for(int i = 0; i < _graph->GetPathCount(); i++)
	{
		costs[i] = _r->CalculateContinueToNextMIP(_graph->GetPath(i),Q,1);
		cost += costs[i];
		if(costs[i] > 9990){
			is_feasible = false; break;
		} 
	}
	
	if(!is_feasible)
	{
		CutCurrentSolution(array);
		return;
	}
		
	
	if(_graph->GetCost() + cost < best_sol)
	{
		best_sol_distance = _graph->GetCost();
		best_sol_recourse = cost;
		best_sol = best_sol_distance + best_sol_recourse;
		printf("New solution:%.3lf dist:%.1lf rec:%.3lf OptCuts:%d Subtours:%d InfSolCuts:%d InfPathCuts:%d\n", 
				best_sol, best_sol_distance, best_sol_recourse,nb_opt_cuts,
				nb_sub_tour_frac+nb_sub_tours,nb_infeasible_solution_cuts,nb_inf_paths);
		best_solution.clear();
		for(int i = 0; i < _graph->GetPathCount(); i++)
			best_solution.push_back( _graph->GetPath(i) );
	}

	{
		//add optimality cut
		IloExpr expr(array.getEnv());
		expr += _theta;

		int nb = 0;
		for(int i = 0; i < _graph->GetPathCount(); i++)
		{
			std::vector<Node*> & path = _graph->GetPath(i);
			if(costs[i] <= 0.0001) continue;

			for(int j = 1; j < path.size(); j++)
			{
				ExBrpArcO* arc = _graph->GetArc(path[j - 1]->no, path[j]->no);

				if(arc->from->type != NODE_TYPE_CUSTOMER || arc->to->type != NODE_TYPE_CUSTOMER) continue;
				expr -= cost * _x[arc->index];
				nb++;
			}
		}

		array.add(expr >= cost * (1 - nb));
		//std::cout << "OptCut: " << nb_opt_cuts << " Cost: " << cost << " Cut: " << expr << " >= " << cost * (1 - nb) << std::endl;
		expr.end();
		nb_opt_cuts++;
	}
}*/

