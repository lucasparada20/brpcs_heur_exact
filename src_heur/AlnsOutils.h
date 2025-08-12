#ifndef ALNS_OUTILS
#define ALNS_OUTILS

#include "Solution.h"
#include "DriverBRPCS.h"
#include "NodeBRPCS.h"
#include "RouteFeasibilityBRPCS.h"
#include "Parameters.h"

class AlnsOutils
{	
	public:
		
		static void FillPathWithPos(Sol & s, Driver* d, Node* n, int pos, std::vector<Node*> & path, int & init_q, int & init_qe, bool & has_zero_rec);
		static bool Is_feasible(Node* prev, Node* next, int lambda, int mu, int Q);
		static double CalculateRouteCost(RouteFeasibility * r, std::vector<Node*> & path, int Q, int init_q, int init_qe, bool has_zero_first_pass);
};


#endif