#ifndef INSRMV_BRPCS
#define INSRMV_BRPCS

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include "MoveBRPCS.h"
#include "ProblemDefinition.h"
#include "MoveBRPCS.h"
#include "RouteFeasibilityBRPCS.h"
#include "Solution.h"

class InsRmvMethodBRPCS
{
	public:
		InsRmvMethodBRPCS(RouteFeasibility * r, Prob * prob) : _r(r), _prob(prob){}; 

		void InsertCost(Sol & s, Node * n, Driver * d, Move & m);
		void ApplyInsertMove(Sol & s, Move & m);
		void RemoveCost(Sol & s, Node * n, Move & m);
		void CheckMove(Sol & s, Move & m){};
		void FillInsertionList(Sol & s, std::vector<Node*> & list);


	private:
		std::vector<Node*> path;
		RouteFeasibility * _r;
		Prob * _prob;
};


#endif