#ifndef SEQ_INSERT
#define SEQ_INSERT

#include "Solution.h"
#include "OperatorBase.h"
#include "InsRmvMethodBRPCS.h"
#include "MoveBRPCS.h"
#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include <ctime> //clock_t
#include "RouteFeasibilityBRPCS.h"
#include "AlnsOutils.h"

class SeqInsertBRPCS : public InsertOperator
{
	public:
		SeqInsertBRPCS(InsRmvMethodBRPCS & insrmv, RouteFeasibility * r)  
			: _insrmv(insrmv), _r(r) {}  

		void Insert(Sol & s, bool show);
		void FillMoveVec(Sol & s, Driver * d, Node * n, std::vector<Move> & move_vec);	

	private:
		InsRmvMethodBRPCS _insrmv;
		std::vector<Node*> path;
		RouteFeasibility * _r;
		
};



#endif
