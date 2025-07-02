#include "SequentialInsertionBRPCS.h"

void SeqInsertBRPCS::FillPathWithPos(Sol & s, Driver* d, Node* n, int pos) {
    path.clear();
    path.reserve(s.RoutesLength[d->id] + 3);  // start, end depot plus the insertion node

    Node* prev = s.GetNode(d->StartNodeID);
    path.push_back(prev);

    int currentPos = 0; 

    while (prev->type != NODE_TYPE_END_DEPOT) {
        if (currentPos == pos) 
            path.push_back(n);  

        prev = s.Next[prev->id];
        path.push_back(prev); 

        currentPos++;
    }
}


bool SeqInsertBRPCS::Is_feasible(Node* prev, Node* next, int lambda, int mu, int Q) 
{
	if (prev->type == NODE_TYPE_CUSTOMER && !prev->is_end_feasible) 
		return false; 

	if (next->type == NODE_TYPE_CUSTOMER && !next->is_start_feasible)
		return false;

    int lb3 = lambda - prev->min_lambda;
    int ub3 = mu + Q - prev->max_mu;
    int lb4 = next->gamma - next->min_gamma;
    int ub4 = next->zeta + Q - next->max_zeta;
	
    return (lb3 <= ub4 && lb4 <= ub3);
}


void SeqInsertBRPCS::FillMoveVec(Sol & s, Driver * d, Node * n, std::vector<Move> & move_vec)
{	
	int Q = d->capacity;

	path.clear();
	path.reserve( s.RoutesLength[d->id]+3 );
	//move_vec.reserve( s.RoutesLength[d->id]+3 );
	//move_lb_vec.reserve( s.RoutesLength[d->id]+3 );
	
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
	int lambda_pos = std::max(-Q, combined_demand - n->h_i0 - n->h_e_i0);
	int mu_pos = std::min(Q, combined_demand + n->maxWm);
	
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
			prev = next; pos++; continue;
		}
		
        int lambda = prev->lambda + lambda_pos;
        int mu = prev->mu + mu_pos;
		if (!Is_feasible(prev, next, lambda, mu, Q && !Parameters::GetCostPolicy())) { //Checks feasibility based on end and start load windows
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
			false, // IsFeasible
			prev,//just to initialize Node * m.move.prev with a Node * prev
			pos, 
			9999 // RecourseLb
		);
		
		//Charging station tests based on extreme cases ...
		//Update() : you store all the unchared bikes in the route, at each node ..
		/*if ( (n->is_chargeable && d->has_uncharged_bikes && prev->has_accumulated_uncharged == 0) ||
			(!n->is_chargeable && d->has_uncharged_bikes && prev->has_accumulated_uncharged > 0) ||
			(n->is_chargeable && d->has_uncharged_bikes && prev->has_accumulated_uncharged > n->maxWm) )
		{
			prev = next;
			pos++;
			continue;
		}*/

		prev = next;
		pos++;
	}
}

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
		
		for (int j = 0; j < s.GetDriverCount(); j++)
			FillMoveVec(s, s.GetDriver(j), n, move_vec); //Stores all feasible moves without computing Mip or the Lb
		std::sort( move_vec.begin(), move_vec.end() );
		
		//printf("moves:%d\n",(int)move_vec.size());
		double elapsed_seconds = 0.0;
		clock_t begin_drv_loop = clock();
		int best_idx=0;
		Move m1;
		for (size_t k = 0; k < move_vec.size(); k++) 
		{	
			Move& move = move_vec[k];
			best_idx = k;
			
			Driver* d = move.to;

			FillPathWithPos(s, d, n, move.pos); // fills std::vector<Node*> path
			
			int Q = d->capacity;
			
			bool has_zero_rec; double cost;
			if(Parameters::GetCostPolicy()) // Restock
			{
				has_zero_rec = RouteFeasibility::HasZeroHCUncharged(path,Q,false);
				cost = has_zero_rec ? 0.0 : _r->CalculateRestockingTrips(path, d->capacity, 1);
			} else { // Continue-to-next
				has_zero_rec = RouteFeasibility::HasZeroHCUnchargedViolations(path,Q,false);
				cost = has_zero_rec ? 0.0 : _r->CalculateContinueToNextMIP(path, d->capacity, 1);	
			}
						
			//printf("Cost in Insert:%.1lf Policy:%d\n",cost,Parameters::GetCostPolicy());
			
			if (cost < 9999.0) {
				move.IsFeasible = true;
				move.DeltaCost = move.DeltaDistance + cost;
				move.DeltaRec = d->curRecourse - cost;

				m1 = move;

				// Check next 2 (two) moves for improvements. If not, then break!
				bool is_better_than_next_two = true;
				//int nb_look_ahead = Parameters::GetCostPolicy() ? 1 : 2;
				int nb_look_ahead = 2;
				for (int l = 1; l <= nb_look_ahead; ++l) {
					if (k + l < move_vec.size()) {
						Move& next_move = move_vec[k + l];

						Driver * nextDriver = next_move.to;
						FillPathWithPos(s, nextDriver, n, next_move.pos);
						//double nextRec = Parameters::GetCostPolicy() ?
						//				_r->CalculateRestockingTripsMIP(path,Q,1)
						//				: _r->CalculateContinueToNextMIP(path, d->capacity, 1);
						
						double nextRec;
						if(Parameters::GetCostPolicy()) // Restock
						{
							has_zero_rec = RouteFeasibility::HasZeroHCUncharged(path,Q,false);
							nextRec = has_zero_rec ? 0.0 : _r->CalculateRestockingTrips(path, d->capacity, 1);
						} else { // Continue-to-next
							has_zero_rec = RouteFeasibility::HasZeroHCUnchargedViolations(path,Q,false);
							nextRec = has_zero_rec ? 0.0 : _r->CalculateContinueToNextMIP(path, d->capacity, 1);	
						}
						
						
						if (nextRec < 9999.0) {
							next_move.IsFeasible = true;
							next_move.DeltaCost = next_move.DeltaDistance + nextRec;
							next_move.DeltaRec = nextDriver->curRecourse - nextRec;

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
		//printf("NodeToInsert:%d Moves:%d found_best_at:%d elapsed[s]:%.1lf\n",n->no,(int)move_vec.size(),best_idx,time_drv_loop);
		
		/*Move m1;
		for (size_t k = 0; k < move_vec.size(); k++) {
			Move& move = move_vec[k];
			
			Driver * d = move.to;
			
			FillPathWithPos(s, d, n, move.pos); //fills std::vector<Node*> path with the insertion at position move.pos
			int rec = _r->CalculateContinueToNextMIP(path, d->capacity, 1);

			if (rec < 9999) {
				move.IsFeasible = true;
				move.DeltaCost = move.DeltaDistance + rec;
				move.DeltaRec = d->curRecourse - rec;

				if (m1.IsFeasible && m1.DeltaCost < move.DeltaCost) {
					break; // Means that you go to the next driver ...
				}

				m1 = move;
			}
		}*/
		
		//clock_t begin_post_loop = clock();
		if (m1.IsFeasible) best = m1;		

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
		s.AddToUnassigneds( refused[i] );
	//printf("Cost after SeqInsertion:%.2lf nbUnassigneds:%d\n",s.GetCost(),s.GetUnassignedCount());
	
	clock_t end_insert = clock();
	double elapsed_insert = (double)( end_insert - begin ) / CLOCKS_PER_SEC;
	//printf("Total insertion time:%.1lf\n",elapsed_insert);	
}
