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
#include <set>

class SeqInsertBRPCS : public InsertOperator
{
	public:
		SeqInsertBRPCS(InsRmvMethodBRPCS & insrmv, RouteFeasibility * r)  
			: _insrmv(insrmv), _r(r) {}  

		void FillPathWithPos(Sol & s, Driver* d, Node* n, int pos);
		bool Is_feasible(Node* prev, Node* next, int lambda, int mu, int Q);
		void Insert(Sol & s, bool show);
		void FillMoveVec(Sol & s, Driver * d, Node * n, std::vector<Move> & move_vec);	

	private:
		InsRmvMethodBRPCS _insrmv;
		std::vector<Node*> path;
		RouteFeasibility * _r;
		
		std::set<int> attempted_moves;
};



#endif
