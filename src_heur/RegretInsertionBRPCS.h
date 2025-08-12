#ifndef REGRET_INSERT
#define REGRET_INSERT

#include "Solution.h"
#include "OperatorBase.h"
#include "InsRmvMethodBRPCS.h"
#include "MoveBRPCS.h"
#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include <ctime> //clock_t
#include "RouteFeasibilityBRPCS.h"
#include "AlnsOutils.h"

// You need the sorter to sort pointers to Move objects (the overloaded operators will not help)
struct MovePtrLess {
	bool operator()(const Move* a, const Move* b) const {
		return a->DeltaCost < b->DeltaCost;
	}
};

class RegretInsertBRPCS : public InsertOperator
{
	public:
		RegretInsertBRPCS(Prob * pr, InsRmvMethodBRPCS & insrmv, RouteFeasibility * r)  
			: _insrmv(insrmv), _pr(pr), _r(r), _used_k(2),_k_regret(2) 
			{
				_moves.resize( _pr->GetCustomerCount() );
				for(int i = 0 ; i < _pr->GetCustomerCount() ; i++)
					_moves[i].resize( _pr->GetDriverCount() );
				printf("Regret init size of _moves: %d x %d\n",_pr->GetCustomerCount(),_pr->GetDriverCount());
			}  
		
		void SetK(int k){_k_regret = k;}
		void Insert(Sol & s, bool show);
		void InsertCost(Sol & s, Node * n, Driver * d,  Move & m);
		double GetRegretCost(Sol & s, Node * n, Move & m);

	private:
		InsRmvMethodBRPCS _insrmv;
		std::vector<Node*> path;
		Prob * _pr;
		
		int _used_k;
		int _k_regret;
		
		RouteFeasibility * _r;
		
		std::vector< std::vector< Move > > _moves; //[nodes][driver]
		std::vector< Move* > move_vect;
		
};

#endif
