#include "SteepestDescentInsertionBRPCS.h"
#include "RouteFeasibilityBRPCS.h"

bool SteepestDescentInsertionBRPCS::Is_feasible(Node* prev, Node* next, int lambda, int mu, int Q) {
    int lb3 = lambda - prev->min_lambda;
    int ub3 = mu + Q - prev->max_mu;
    int lb4 = next->gamma - next->min_gamma;
    int ub4 = next->zeta + Q - next->max_zeta;
    return (lb3 <= ub4 && lb4 <= ub3);
}

void SteepestDescentInsertionBRPCS::FindMove(Sol & s, Move& m, Driver* d, Node* n)
{
    m.IsFeasible = false;
    m.DeltaCost = INFINITE;
    m.n = n;
    m.to = d;    

    int Q = d->capacity;
    
    // Precompute the path instead of dynamically modifying the vector
    path.clear();
    path.reserve(s.RoutesLength[d->id] + 2);

    Node* prev = s.GetNode(d->StartNodeID);
    path.push_back(prev); 
    path.push_back(n);
    
    while (prev->type != NODE_TYPE_END_DEPOT) {
        prev = s.Next[prev->id];
        path.push_back(prev);
    }

    // Combined demand calculations
    int combined_demand = n->q + n->q_e;
	int mu_pos = std::min(Q, combined_demand + n->maxWm);
	
	int lambda_pos = 0;
	if(Parameters::GetBSSType() == CS){
		lambda_pos = std::max(-Q, combined_demand - n->h_i0 - n->h_e_i0);
	}
	else if (Parameters::GetBSSType() == SW){
		lambda_pos = std::max(-Q, combined_demand - n->h_i0 - n->h_e_i0 - n->h_u_i0);
	}
    
    int pos = 0;
    prev = s.GetNode(d->StartNodeID);

    while (prev->type != NODE_TYPE_END_DEPOT)
    {
        Node* next = s.Next[prev->id];

        if (prev->type != NODE_TYPE_START_DEPOT)
            std::swap(path[pos], path[pos + 1]);

        double deltaDistance = s.GetDist(prev, n) + s.GetDist(n, next) - s.GetDist(prev, next);
        if (d->curDistance + deltaDistance > Parameters::MaxRouteDistance()) {
            prev = next; 
            pos++; 
            continue;
        }

        // Check feasibility (precomputed values)
        if (prev->type == NODE_TYPE_CUSTOMER && !prev->is_end_feasible) {
            prev = next; 
            pos++; 
            continue;
        }

        if (next->type == NODE_TYPE_CUSTOMER && !next->is_start_feasible) {
            prev = next; 
            pos++; 
            continue;
        }       

        int lambda = prev->lambda + lambda_pos;
        int mu = prev->mu + mu_pos;
		
		//Avoids creating the vectors
        if (!Is_feasible(prev, next, lambda, mu, Q)) {
            prev = next; 
            pos++; 
            continue;
        }
		
		bool HasZeroRec = false;
		if(Parameters::GetBSSType() == CS){
			HasZeroRec = RouteFeasibility::HasZeroHCUnchargedViolations(path, Q, false);
		} else if (Parameters::GetBSSType() == CS){
			HasZeroRec = RouteFeasibility::HasZeroHCBase(path, Q, false);
		}
        int rec = HasZeroRec ? 0 : _r->CalculateContinueToNextMIP(path, Q, 1);

        if (rec > 9999) {
            prev = next; 
            pos++; 
            continue;
        }

        double best_so_far = m.DeltaCost;

        if (deltaDistance + rec < best_so_far) {
            m.DeltaCost = deltaDistance + rec;
            m.DeltaDistance = deltaDistance;
            m.DeltaRec = d->curRecourse - rec;
            m.IsFeasible = true;
            m.move.prev = prev;

            if (std::abs(m.DeltaCost / best_so_far) < _steepest && INFINITE - best_so_far > 1.0) {
				//printf("Steepest curCost:%d steepCost:%d pos:%d nodes:%d\n",(int)best_so_far,(int)m.DeltaCost,pos,(int)path.size()-2);
				return;
            }
        }

        prev = next;
        pos++;
    }
}

void SteepestDescentInsertionBRPCS::Insert(Sol & s, bool show)
{
	s.Update();
	std::vector<Node*> nodes; //list of customers to insert
	std::vector<Node*> refused; //list of infeasible customers
	_insrmv.FillInsertionList(s, nodes);

	//printf("Nodes to insert:");
	//for(size_t i=0;i<nodes.size();i++)
	//{
	//	printf("%d-",nodes[i]->no);
	//	if(nodes[i]->type != NODE_TYPE_CUSTOMER)
	//	{
	//		printf("\nPhil Collins, 1989... exiting."); exit(1);
	//	}
	//}
	//printf("\n");

	clock_t startTime = clock();
	for(size_t i=0;i<nodes.size();i++)
	{
		Node * n = nodes[i];
		if (show && i % 10 == 0)
		{
			clock_t endTime = clock();
			double elapsedSeconds = (double)(endTime - startTime) / CLOCKS_PER_SEC;
			printf("SteepestDescent Inserting Node:%d ElapsedSeconds:%.1lf\n", n->id, elapsedSeconds);
		}
		
		Move best;
		best.IsFeasible = false;
		best.DeltaCost = INFINITE;
		
		for(int j = 0 ; j < s.GetDriverCount() ; j++)
		{
			Driver * d = s.GetDriver(j);
			int Q = d->capacity;
			//To test infeasible set
			//Too Relax (1984), don't uncomment unless improved feasibility check
			/*Node * prev = s.GetNode( d->StartNodeID );
			path.push_back(prev); 
			path.push_back(n);
			while(prev->type != NODE_TYPE_END_DEPOT)
			{
				Node * next = s.Next[ prev->id ];
				path.push_back(next);
				prev = next;
			}
			bool is_feasible = true;
			if(!d->has_uncharged_bikes && n->h_u_i0 == 0)
			{
				is_feasible = RouteFeasibility::IsFeasibleSet(path,Q,true);
				if(!is_feasible) inf_set_cntr_steep++;
				if(inf_set_cntr_steep % 10 == 0 && inf_set_cntr_steep)
					printf("inf_set_cntr_steep:%d\n",inf_set_cntr_steep);
			
			}
			if(!is_feasible) continue;*/
			
			Move m;
			FindMove(s,m,d,n);
			
			if(m.IsFeasible && m.DeltaCost < best.DeltaCost) // Just if its feasible and then store in an std::vector<Move> and sort
				best = m;
		}

		best.from = NULL;
		if(best.IsFeasible)
		{
			_insrmv.ApplyInsertMove(s, best);
			s.Update(best.to); // Will call a different MIP depending on the cost policy!
		}
		else
		{
			refused.push_back(n);
			s.RemoveFromUnassigneds(n);
		}
	
		//fprintf(file,"NodeInserted:%d Cost:%.2lf Unassigneds:%d\n",n->no,s.GetCost(),s.GetUnassignedCount());

	}//end node for
	

	clock_t endTime = clock();
	double elapsedSeconds = (double)(endTime - startTime) / CLOCKS_PER_SEC;
	printf("SteepestDescent Inserted:%d nodes ElapsedSeconds:%.1lf\n", (int)nodes.size(), elapsedSeconds);

	for(size_t i = 0 ; i < refused.size() ; i++)
	s.AddToUnassigneds( refused[i] );
	//printf("Cost after SeqInsertion:%.2lf nbUnassigneds:%d\n",s.GetCost(),s.GetUnassignedCount());
	//fclose(file);
}