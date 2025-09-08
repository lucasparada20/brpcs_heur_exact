#include "SequentialInsertionBRPCS.h"

void SeqInsertBRPCS::Insert(Sol & s, bool show)
{
	clock_t begin = clock();
	s.Update();
	clock_t end = clock();
	double elapsed_update = (double)( end - begin ) / CLOCKS_PER_SEC;
	//printf("In insertion. Time to update:%.1lf\n",elapsed_update);
	
	std::vector<Node*> nodes; nodes.reserve(100); //list of customers to insert
	std::vector<Node*> refused; refused.reserve(100);//list of infeasible customers
	_insrmv.FillInsertionList(s, nodes);

	clock_t startTime = clock();
	for (size_t i = 0; i < nodes.size(); i++) 
	{
		Node* n = nodes[i];

		Move best;
		best.IsFeasible = false;
		best.DeltaCost = INFINITE;

		std::vector< Move > move_vec;
		move_vec.reserve( s.GetDriverCount()*100 );
		
		
		bool skip_extra_empty = (nodes.size() <= 0.1 * s.GetProb()->GetCustomerCount());
		bool has_seen_empty_driver = false;
		
		//printf("In Insert skip_extra_empty:%d allDrivers:%d\n",skip_extra_empty,s.GetDriverCount());
		for (int j = 0; j < s.GetDriverCount(); j++)
		{
			if(s.GetRouteLength( s.GetDriver(j)) == 0 && has_seen_empty_driver && skip_extra_empty) continue;
			
			FillMoveVec(s, s.GetDriver(j), n, move_vec); //Stores all feasible moves without computing Mip or the Lb
			
			if(s.GetRouteLength( s.GetDriver(j)) == 0)
				has_seen_empty_driver = true;
		}
			
		std::sort( move_vec.begin(), move_vec.end() );
		
		//printf("moves from FillMoveVec:%d for nodeId:%d\n",(int)move_vec.size(),n->id);
		double elapsed_seconds = 0.0;
		clock_t begin_drv_loop = clock();
		int best_idx=0;
		Move m1;
		for (size_t k = 0; k < move_vec.size(); k++) 
		{	
			Move& move = move_vec[k];
			best_idx = k;
			
			Driver* d = move.to;
			int Q = d->capacity;
			
			std::vector<Node*> path;
			int init_q; int init_qe; bool has_zero_first_pass;
			
			AlnsOutils::FillPathWithPos(s, d, n, move.pos, path, init_q, init_qe, has_zero_first_pass); // fills std::vector<Node*> path

			double cost = AlnsOutils::CalculateRouteCost(_r, path, Q, init_q, init_qe, has_zero_first_pass);
						
			//printf("Cost in Insert:%.1lf nodesId:%d\n",cost,n->id);
			
			if (cost < 9999.0) {
				move.IsFeasible = true;
				move.DeltaCost = move.DeltaDistance + cost;

				m1 = move;

				// Check next nb_look_ahead moves for improvements. If not, then break!
				bool is_better_than_next_two = true;
				//int nb_look_ahead = 3; // Testing ....
				int nb_look_ahead = 2; 
				if (Parameters::GetBSSType() == SW && Parameters::GetCostPolicy() == CN 
					|| Parameters::GetBSSType() == CS && Parameters::GetCostPolicy() == CN)
					nb_look_ahead = 3;
				
				for (int l = 1; l <= nb_look_ahead; ++l) {
					if (k + l < move_vec.size()) {
						Move& next_move = move_vec[k + l];

						Driver * nextDriver = next_move.to;
						
						AlnsOutils::FillPathWithPos(s, nextDriver, n, next_move.pos, path, init_q, init_qe, has_zero_first_pass);
						
						double nextRec = AlnsOutils::CalculateRouteCost(_r, path, Q, init_q, init_qe, has_zero_first_pass);
						
						if (nextRec < 9999.0) {
							next_move.IsFeasible = true;
							next_move.DeltaCost = next_move.DeltaDistance + nextRec;

							if (next_move.DeltaCost < move.DeltaCost) {
								is_better_than_next_two = false;
								break;
							}
						}
					}
				}

				if (is_better_than_next_two) {
					break; // move is better than next two consecutive moves
				}
			}
			
		}
		clock_t end_drv_loop = clock();
		double time_drv_loop = (double)( end_drv_loop - begin_drv_loop ) /CLOCKS_PER_SEC;
		//printf("NodeToInsert:%d Moves:%d found_best_at:%d elapsed[s]:%.1lf\n",n->id,(int)move_vec.size(),best_idx,time_drv_loop);
		//m1.Show();
		
		
		//clock_t begin_post_loop = clock();
		if (m1.IsFeasible) 
		{
			best = m1;
			//printf("node %d/%d feasible\n",i,nodes.size());
		}	

		best.from = NULL;
		if(best.IsFeasible)
		{
			_insrmv.ApplyInsertMove(s, best);
			s.Update(best.to);
		}
		else
		{
			refused.push_back(n);
			s.RemoveFromUnassigneds(n);
		}
		//clock_t end_post_loop = clock();
		//double elapsed_post_loop = (double)(end_post_loop-begin_post_loop)/CLOCKS_PER_SEC;
		//printf("NodeInserted:%d post_loop_time:%.1lf\n",n->no,elapsed_post_loop);
		//printf("NodeInserted:%d Cost:%.2lf Unassigneds:%d\n",n->no,s.GetCost(),s.GetUnassignedCount());

	}//end node for


	for(size_t i = 0 ; i < refused.size() ; i++)
	{
		//printf("Unassigneds:%d nId:%d\n",s.GetUnassignedCount(),n->id);
		s.AddToUnassigneds( refused[i] );
	}
		
	//printf("Cost after SeqInsertion:%.2lf nbUnassigneds:%d\n",s.GetCost(),s.GetUnassignedCount());
	//printf("Unassigneds: ");
	//for(int i=0;i<s.GetUnassignedCount();i++)
	//	printf("%d ",s.GetUnassigned(i)->id);
	//printf("\n");
	//getchar();
	
	clock_t end_insert = clock();
	double elapsed_insert = (double)( end_insert - begin ) / CLOCKS_PER_SEC;
	//printf("Total insertion time:%.1lf\n",elapsed_insert);	
}

void SeqInsertBRPCS::FillMoveVec(Sol & s, Driver * d, Node * n, std::vector<Move> & move_vec)
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
	//if((int)path.size()>3)
	//{
	//	printf("nId:%d Dist:%.1lf Cost:%.1lf Path: ",n->id,d->curDistance,d->curRecourse);
	//	for(size_t i=0;i<path.size();i++)
	//		printf("%d-", path[i]->id);
	//	printf("\n");
	//}

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
	while(prev->type != NODE_TYPE_END_DEPOT)
	{
		//Initialize the move 
		Move m;
		m.IsFeasible = false;
		m.DeltaCost = INFINITE;
		m.n = n;
		m.to = d;	

		Node * next = s.Next[ prev->id ];
		//printf("Prev:%d next:%d\n", prev->id, next->id);
		if(prev->type != NODE_TYPE_START_DEPOT)
			iter_swap(path.begin() + pos, path.begin() + pos + 1);
		
		double deltaDistance = s.GetDist(prev,n) + s.GetDist(n,next) - s.GetDist(prev,next);
		if(d->curDistance + deltaDistance > Parameters::MaxRouteDistance()) 
		{
			//printf("nId:%d pos:%d deltaDistance:%.1lf newDist:%.1lf\n",n->id,pos,deltaDistance,d->curDistance + deltaDistance);
			
			prev = next; pos++; continue;
		}
		
        int lambda = prev->lambda + lambda_pos;
        int mu = prev->mu + mu_pos;
		
		//Debugging : Is not the cost, its the distance!
		/*if(!AlnsOutils::Is_feasible(prev, next, lambda, mu, Q))
		{
			std::vector<Node*> path1;
			int init_q; int init_qe; bool has_zero_first_pass;
			AlnsOutils::FillPathWithPos(s, d, n, pos, path1, init_q, init_qe, has_zero_first_pass);			
			double rec = AlnsOutils::CalculateRouteCost(_r, path, Q, init_q, init_qe, has_zero_first_pass);
			IloEnv env; RouteFeasibility r(s.GetProb());
			double rec1 = r.CalculateContinueToNextMIP(path,Q,1,env);
			env.end();
			if(rec<9990 || rec1 > 9990)
			{
				printf("cost:%.1lf cost1:%.1lf\n",rec,rec1);
			} 
			printf("nId:%d pos:%d cost:%.1lf cost1:%.1lf\n",n->id,pos,rec,rec1);	
		}*/
		
		if (Parameters::GetCostPolicy() == CN && !AlnsOutils::Is_feasible(prev, next, lambda, mu, Q)) {
			prev = next;
			pos++;
			continue;
		}

		
		m.DeltaCost = deltaDistance; //just store delta distance for a quick sort later
		m.move.prev = prev;
		m.pos = pos;
		
		//move_vec.push_back(m); // Change later to emplace_back() ...
		move_vec.emplace_back(
			n,
			d,
			nullptr, //Driver * from
			deltaDistance, // DeltaCost
			deltaDistance, // DeltaDistance
			9999, // DeltaRec
			false, // Always false for RT as they can later be found feasible. Also set as always false for CN.
			prev,//just to initialize Node * m.move.prev with a Node * prev
			pos, 
			9999 // RecourseLb
		);
		
		//printf("nId:%d pos:%d move_vec:%zu\n",n->id,pos,move_vec.size());
		
		prev = next;
		pos++;
	}
	//printf("For nId:%d found %zu moves\n",n->id,move_vec.size());
	//if(!move_vec.size())
	//	getchar();
}