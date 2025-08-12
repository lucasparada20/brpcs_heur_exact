#include "AlnsOutils.h"

void AlnsOutils::FillPathWithPos(Sol & s, Driver* d, Node* n, int pos, std::vector<Node*> & path, int & init_q, int & init_qe, bool & has_zero_rec) {
    path.clear();
    path.reserve(s.RoutesLength[d->id] + 3);  // start, end depot plus the insertion node

    Node* prev = s.GetNode(d->StartNodeID);
    path.push_back(prev);
	
	init_q = 0; init_qe = 0;
    int currentPos = 0; 
	
	init_q = 0; init_qe = 0; // Just in case you forgot when passing the arguments ...
    while (prev->type != NODE_TYPE_END_DEPOT) {
        if (currentPos == pos)
		{
			path.push_back(n); 
			init_q += n->q; init_qe += n->q_e;
		}
		
        prev = s.Next[prev->id];
        path.push_back(prev);
		
		if(prev->type == NODE_TYPE_CUSTOMER)
		{
			init_q += prev->q; init_qe += prev->q_e;
		}
		
        currentPos++;
    }
	
	int Q = d->capacity;
	has_zero_rec = true;
	if(init_q > Q || init_q < -Q || init_qe > Q || init_qe < -Q) has_zero_rec = false;
	if(init_q+init_qe > Q || init_q+init_qe < -Q) has_zero_rec = false; 
}

bool AlnsOutils::Is_feasible(Node* prev, Node* next, int lambda, int mu, int Q) 
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

// path has depots
double AlnsOutils::CalculateRouteCost(RouteFeasibility * r, std::vector<Node*> & path, int Q, int init_q, int init_qe, bool has_zero_first_pass)
{
	bool has_zero_rec = false; double cost = 0.0;
	if(Parameters::GetCostPolicy() == RT) // Restock
	{
		if(Parameters::GetBSSType() == CS && has_zero_first_pass){
			has_zero_rec = RouteFeasibility::HasZeroHCUncharged(path,Q,false,init_q,init_qe);
		} else if (Parameters::GetBSSType() == SW && has_zero_first_pass){
			has_zero_rec = RouteFeasibility::HasZeroHCBase(path,Q,false,init_q,init_qe);
		}
		cost = has_zero_rec ? 0.0 : r->CalculateRestockingTrips(path, Q, 1);
		
	} else { // Continue-to-next
	
		if(Parameters::GetBSSType() == CS && has_zero_first_pass){
			has_zero_rec = RouteFeasibility::HasZeroHCUnchargedViolations(path,Q,false,init_q,init_qe);
		} else if (Parameters::GetBSSType() == SW && has_zero_first_pass){
			has_zero_rec = RouteFeasibility::HasZeroHCBase(path,Q,false,init_q,init_qe);
		}
		cost = has_zero_rec ? 0.0 : r->CalculateContinueToNextMIP(path, Q, 1);	
	}

	return cost;	
}