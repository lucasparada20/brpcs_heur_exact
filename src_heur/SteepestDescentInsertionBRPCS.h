#ifndef STEEPEST_INSERT
#define STEEPEST_INSERT

#include "Solution.h"
#include "OperatorBase.h"
#include "InsRmvMethodBRPCS.h"
#include "MoveBRPCS.h"
#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include <ctime> //clock_t
#include "RouteFeasibilityBRPCS.h"

class SteepestDescentInsertionBRPCS : public InsertOperator
{
	public:
		SteepestDescentInsertionBRPCS(InsRmvMethodBRPCS & insrmv, RouteFeasibility * r, double steepest)  
			: _insrmv(insrmv), _r(r), _steepest(steepest), inf_set_cntr_steep(0), has_zero_rec_cntr_steep(0) {}  

		bool Is_feasible(Node* prev, Node* next, int lambda, int mu, int Q);
		void Insert(Sol & s, bool show) override;
		void FindMove(Sol & s, Move & m, Driver * d, Node * n); 

	private:
		InsRmvMethodBRPCS _insrmv;
		std::vector<Node*> path;
		RouteFeasibility * _r;
		double _steepest;
		
		int inf_set_cntr_steep;
		int has_zero_rec_cntr_steep;
};



#endif