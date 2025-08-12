#include "InsRmvMethodBRPCS.h"
#include "constants.h"
#include "RouteFeasibilityBRPCS.h"
#include "Parameters.h"
#include <algorithm>

void InsRmvMethodBRPCS::FillInsertionList(Sol & s, std::vector<Node*> & list)
{
	list.clear(); list.reserve( _prob->GetCustomerCount() );
	for(int i=0;i<s.GetUnassignedCount();i++)
		list.push_back(s.GetUnassigned(i));
}
// This method is deprecated.
/*void InsRmvMethodBRPCS::InsertCost(Sol & s, Node * n, Driver * d, Move & mo)
{
	//s.Update() is called right before this function
	//printf("InsertCost n:%d d:%d\n", n->id, d->id);
	int Q = d->capacity;
	
	mo.IsFeasible = false;
	mo.DeltaCost = INFINITE;
	mo.n = n;
	mo.to = d;
		
	path.clear();
	path.reserve( s.RoutesLength[d->id] );

	Node * prev = s.GetNode( d->StartNodeID );
	path.push_back(prev); 
	path.push_back(n);
	while(prev->type != NODE_TYPE_END_DEPOT)
	{
		Node * next = s.Next[ prev->id ];
		path.push_back(next);
		prev = next;
	}
	//if((int)path.size()>3)
	//{
	//	printf("Path: ");
	//	for(size_t i=0;i<path.size();i++)
	//		printf("%d-", path[i]->id);
	//	printf("\n");
	//	getchar();
	//}


	//INSERT COST FOR THE SBRP
	//Looping to search for the insertion move that respects distance constraint, is feasible, and has the better cost. The following 3 feasibility checks are made.
	//Route is of the form: d-1-2-...-prev-InsertNode-Insert+1-Insert+2-...-d
	//Input: In the Update() function, ALL forward and backward feasibility lb, ubs are computed and stored for all nodes in the route.
	//Check 1.- Check if the route is feasible up to prev (d-1-2-...-prev). This check is O(1).
	//Check 2.- Check if the route is BACKWARDS feasible from the next node after the insertion up to the end depot (Compute StartLoadWindow for path Insert+1-....-d). This check is O(1).
	//Check 3.- Check if the insertion is feasible (InsertNode). This check is O(1)
	// ---------------------------------------------------------------------------------------------------------------------------------------------------
	//TOTAL COMPLEXITY: 3 checks made, each one is O(1).

	int combined_demand = n->q + n->q_e;
	int lambda_pos = std::max(-Q, combined_demand - n->h_i0 - n->h_e_i0);
	int mu_pos = std::min(Q, combined_demand + n->maxWm);
	
	int pos = 0;
	prev = s.GetNode( d->StartNodeID );
	while(prev->type != NODE_TYPE_END_DEPOT)
	{
		Node * next = s.Next[ prev->id ];
		//printf("Prev:%d next:%d\n", prev->id, next->id);
		if(prev->type != NODE_TYPE_START_DEPOT)
			iter_swap(path.begin() + pos, path.begin() + pos + 1);
		
		double deltaDist = s.GetDist(prev,n) + s.GetDist(n,next) - s.GetDist(prev,next);
		if(d->curDistance + deltaDist > Parameters::MaxRouteDistance()) 
		{
			prev = next; pos++; continue;
		}
		
		std::vector<Node*> path_until_prev; std::vector<Node*> path_after_insertion;
		path_until_prev.reserve(path.size()); path_after_insertion.reserve(path.size());
		
		for(size_t i=0;i<path.size();i++)
		{
			if(n->no == path[i]->no) continue;
			if((int)i<pos+1) path_until_prev.push_back( path[i] );
			else path_after_insertion.push_back( path[i] );
		}
		
		//1.- Check if the route is feasible up to prev. This check is O(1)
		if( prev->type ==NODE_TYPE_CUSTOMER && prev->is_end_feasible == false)
		{
			prev = next; pos++; continue;
		}
		
		//2.- Check if the route is BACKWARDS feasible from the next node after the insertion (Compute StartLoadWindow for path Insert+1-....-d). This check is O(n).
		//IMPROVED: In the Update() function the lb, ubs were compute for both forward and backward feasibility -> The check becomes O(1).
		if(path_after_insertion[0]->type == NODE_TYPE_CUSTOMER && path_after_insertion[0]->is_start_feasible == false)
		{	
			prev = next; pos++; continue;
		}		
		
		int lambda = prev->lambda + lambda_pos;
		int min_lambda = std::min( lambda, prev->min_lambda );
		int mu = prev->mu + mu_pos;
		int max_mu = std::max( mu, prev->max_mu );

		int lb3 = lambda - min_lambda;
		int ub3 = mu + Q - max_mu;
		int lb4 = next->gamma - next->min_gamma;
		int ub4 = next->zeta + Q - next->max_zeta;
		
		//3.- Check if the insertion is feasible. This check is O(1)
		if(!(lb3 <= ub4 && lb4 <= ub3))
		{
			prev = next; pos++; continue;
		}
		
		double newcost = deltaDist;
		//if( newcost < mo.DeltaCost && RouteFeasibility::EndLoadHybrid(path,Q,false)) //Without constant time check, this line is necessary
		{
			int rec = _r->CalculateContinueToNextMIP(path,Q,1);
			//printf("InsRmvMethod : Called cost from CN ...\n");
			if(rec > 9999.0)
			{
				prev = next; pos++; continue;
			}
			if(newcost + rec < mo.DeltaCost)
			{
				//if((int)path.size()>3)
				//{
				//	printf("Path with better move in InsRmvMethodBRPCS: Dist:%.1lf Rec:%d\n",d->curDistance+deltaDist,rec);
				//	for(size_t i=0;i<path.size();i++)
				//		printf("%d-", path[i]->id);
				//	printf("\n");
				//	getchar();
				//}

				mo.DeltaDistance = deltaDist;
				mo.DeltaCost = newcost + rec;
				mo.IsFeasible = true;
				mo.move.prev = prev;
			}
		}

		prev = next;
		pos++;
	}
		
}*/

void InsRmvMethodBRPCS::ApplyInsertMove(Sol & s, Move & m)
{
	if(m.from != NULL)
	{
		s.Remove(m.n);
		m.from->sum_q -=  m.n->q;
		m.from->sum_q_e -=  m.n->q_e;

	}
	else if(s.IsUnassigned(m.n))
		s.RemoveFromUnassigneds(m.n);

	s.InsertAfter(m.n, m.move.prev);
	//opt2.Optimize(s, m.to);
}

void InsRmvMethodBRPCS::RemoveCost(Sol & s, Node * n, Move & m)
{
	m.IsFeasible = true;
	m.DeltaDistance = 0;
	m.n = n;
	m.to = NULL;
	m.move.prev = NULL;
	m.from = s.GetAssignedTo(n);
	if(m.from != NULL)
	{
		Node * prev = s.Prev[n->id];
		Node * next = s.Next[n->id];
		m.DeltaDistance = 	s.GetDist( prev,next) -
							s.GetDist( prev,n) -
		 					s.GetDist( n,next);
		m.move.prev = prev;
	}

	m.DeltaCost = m.DeltaDistance;
}
