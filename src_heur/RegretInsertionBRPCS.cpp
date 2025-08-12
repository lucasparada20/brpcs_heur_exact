#include "RegretInsertionBRPCS.h"

void RegretInsertBRPCS::Insert(Sol & s, bool show)
{
	_used_k = std::min(s.GetDriverCount(), _k_regret);
	
	//printf("RegretInsertion k:%d Unassigneds:%d Drivers:%d\n",_k_regret,s.GetUnassignedCount(),s.GetDriverCount());
	std::vector<Node*> toRemove(0);
	std::vector<Node*> refused(0);
	s.Update();
	for(int i=0;i<s.GetUnassignedCount();i++)
	{
		bool has_seen_empty_driver = false;
		for(int j = 0 ; j < s.GetDriverCount() ; j++)
		{
			Node * n = s.GetUnassigned(i);
			Driver * d = s.GetDriver(j);
			
			if(s.GetRouteLength( s.GetDriver(j)) == 0 && has_seen_empty_driver) 
			{
				_moves[n->id][d->id].IsFeasible = false;
				continue;
			}
			
			InsertCost(s, n, d, _moves[n->id][d->id]); //The greedy version computes the best cost for _moves[n->id][d->id]
			
			if(s.GetRouteLength( s.GetDriver(j)) == 0)
				has_seen_empty_driver = true;
		}		
	}
	
	int iter = 0;
	while(s.GetUnassignedCount() > 0)
	{
		//printf("Iteration:%d unassigned:%d list:\n",iter,s.GetUnassignedCount());
		//for(int k=0;k<s.GetUnassignedCount();k++)
		//	printf(" %d",s.GetUnassigned(k)->id);
		//printf("\n"); //getchar();
		
		Move best;
		best.IsFeasible = false;
		double maxRegret = -INFINITE;

		for(int i=0;i<s.GetUnassignedCount();i++)
		{
			Move m;
			double regret = GetRegretCost(s, s.GetUnassigned(i), m);		
			//printf("Unassigned:%d regret:%.1lf maxRegret:%.1lf\n",s.GetUnassigned(i)->id,regret,maxRegret);
						
			if(m.IsFeasible && (regret > maxRegret || (regret == maxRegret && m.DeltaCost < best.DeltaCost)))
			{
				maxRegret = regret;
				best = m;
			}
			else if (!m.IsFeasible)
				toRemove.push_back( s.GetUnassigned(i) );
		}

		if(best.IsFeasible)
		{
			best.from = NULL;
			_insrmv.ApplyInsertMove(s,best);
			s.Update(best.to);
		}

		for(size_t i = 0 ; i < toRemove.size() ; i++)
		{
			refused.push_back(toRemove[i]);
			s.RemoveFromUnassigneds(toRemove[i]);
		}
		toRemove.clear();

		for(int i=0;i<s.GetUnassignedCount();i++)
		{
			Node * n = s.GetUnassigned(i);
			if(_moves[n->id][best.to->id].IsFeasible)
				InsertCost(s, n, best.to, _moves[n->id][best.to->id]);
		}
		iter++;
	}//end while

	for(size_t i = 0 ; i < refused.size() ; i++)
		s.AddToUnassigneds( refused[i] );
	
	//printf("Finished regret. Cost:%.2lf Unassigneds:%d List:\n",s.GetCost(),s.GetUnassignedCount());
	//for(int k=0;k<s.GetUnassignedCount();k++)
	//	printf(" %d",s.GetUnassigned(k)->id);
	///printf("\n"); getchar();
}

void RegretInsertBRPCS::InsertCost(Sol & s, Node * n, Driver * d, Move & m)
{	
	int Q = d->capacity;

	path.clear();
	path.reserve( s.RoutesLength[d->id]+3 );
	
	Node * prev = s.GetNode( d->StartNodeID );
	path.push_back(prev); 
	path.push_back(n);
	while(prev->type != NODE_TYPE_END_DEPOT)
	{
		Node * next = s.Next[ prev->id ];
		path.push_back(next);
		prev = next;
	}
	/*if((int)path.size()>3)
	{
		printf("Path: ");
		for(size_t i=0;i<path.size();i++)
			printf("%d-", path[i]->id);
		printf("\n");
		getchar();
	}*/

	int combined_demand = n->q + n->q_e;
	int mu_pos = std::min(Q, combined_demand + n->maxWm);
	
	int lambda_pos = 0;
	if(Parameters::GetBSSType() == CS){
		lambda_pos = std::max(-Q, combined_demand - n->h_i0 - n->h_e_i0);
	} else if(Parameters::GetBSSType() == SW){
		lambda_pos = std::max(-Q, combined_demand - n->h_i0 - n->h_e_i0 - n->h_u_i0);
	}

	int pos = 0;
	prev = s.GetNode( d->StartNodeID );
	double DeltaCost = INFINITE;  // UPDATE INSIDE THE LOOOP!
	bool found_improving_move = false;
	while(prev->type != NODE_TYPE_END_DEPOT)
	{
		//Initialize the move data

		Node * next = s.Next[ prev->id ];
		//printf("Prev:%d next:%d\n", prev->id, next->id);
		if(prev->type != NODE_TYPE_START_DEPOT)
			iter_swap(path.begin() + pos, path.begin() + pos + 1);
		
		double deltaDistance = s.GetDist(prev,n) + s.GetDist(n,next) - s.GetDist(prev,next);
		if(d->curDistance + deltaDistance > Parameters::MaxRouteDistance()) 
		{
			prev = next; pos++; continue;
		}
		
        int lambda = prev->lambda + lambda_pos;
        int mu = prev->mu + mu_pos;
		
		if (Parameters::GetCostPolicy() == CN && !AlnsOutils::Is_feasible(prev, next, lambda, mu, Q)) {
			prev = next; pos++; continue;
		}
		
		// Just store delta distance to sort, and set as true.
		if(deltaDistance < DeltaCost)
		{
			DeltaCost = deltaDistance;
			
			m.IsFeasible = true;
			m.DeltaCost = deltaDistance;
			m.DeltaDistance = deltaDistance;
			m.n = n;
			m.to = d;
			m.pos = pos;
			m.move.prev = prev;

			found_improving_move = true;
		} 
		
		prev = next;
		pos++;
	}
	
	if(!found_improving_move)
	{
		m.IsFeasible = false;
		m.DeltaCost = INFINITE;
		m.DeltaDistance = INFINITE;
		m.n = n;
		m.to = d;
		m.pos = 0;
		m.move.prev = s.GetNode( d->StartNodeID );
	}
			
}

double RegretInsertBRPCS::GetRegretCost(Sol & s, Node * n, Move & m)
{
	int nbfeasible = 0;
	move_vect.clear(); move_vect.reserve( s.GetDriverCount() );
	
	for(int j = 0 ; j < s.GetDriverCount() ; j++)
	{
		move_vect.push_back(&_moves[n->id][ s.GetDriver(j)->id ]);
		if(_moves[n->id][ s.GetDriver(j)->id ].IsFeasible)
			nbfeasible++;
	}
	
	size_t sort_count = std::min(move_vect.size(), static_cast<size_t>(2 * _used_k));

	std::partial_sort( move_vect.begin(), move_vect.begin() + sort_count, move_vect.end(), MovePtrLess()); // Sorts on DeltaDistance
	
	for(size_t j = 0; j < std::min(move_vect.size(),(size_t)2*_used_k); j++)
	{
		Move * mv = move_vect[j];
		if(!mv->IsFeasible) continue;
		
		Driver* d = mv->to;
		int Q = d->capacity;
		Node* n = mv->n;
		int pos = mv->pos;
		
		std::vector<Node*> path;
		int init_q, init_qe; bool has_zero_first_pass;
		AlnsOutils::FillPathWithPos(s, d, n, pos, path, init_q, init_qe, has_zero_first_pass);
		double cost = AlnsOutils::CalculateRouteCost(_r, path, Q, init_q, init_qe, has_zero_first_pass);
		
		if(cost < 9990)
		{
			mv->DeltaRec = cost;
			mv->DeltaCost = mv->DeltaDistance + mv->DeltaRec;
		} else {
			mv->DeltaCost = INFINITE;
			mv->IsFeasible = false;
		}		
	}
	
	std::partial_sort(move_vect.begin(), move_vect.begin()+ sort_count,  move_vect.end(), MovePtrLess()); // Now sorts on Move.DeltaCost		
	
	double cost = 0;
	m = *move_vect[0];
	for(int j = 1 ; j < _used_k ; j++)
		cost += move_vect[j]->DeltaCost - move_vect[0]->DeltaCost;
	return cost;
}