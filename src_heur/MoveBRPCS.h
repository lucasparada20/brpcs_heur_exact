#ifndef MOVE_H_
#define MOVE_H_
#include <stdlib.h>
#include "constants.h"
#include <vector>

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"

class MoveBRPCS
{
	public:
	MoveBRPCS() : prev(NULL){};
	Node * prev;
};

class Move
{
	public:
		Node * n;
		Driver * to;
		Driver * from;
		double DeltaCost;
		double DeltaDistance;
		double DeltaRec;
		bool IsFeasible;
		MoveBRPCS move;
		int pos;
		int RecourseLb;

		Move(): n(NULL), to(NULL),from(NULL), DeltaCost(INFINITE), DeltaDistance(INFINITE), DeltaRec(INFINITE), IsFeasible(false), move(), pos(-1), RecourseLb(INFINITE) {}
		
		Move(const Move & other) : n(other.n), to(other.to), from(other.from), DeltaCost(other.DeltaCost), DeltaDistance(other.DeltaDistance), DeltaRec(other.DeltaRec), IsFeasible(other.IsFeasible), move(other.move), pos(other.pos), RecourseLb(other.RecourseLb) {}

		// Assignment operator
		Move& operator=(const Move & other)
		{
			if (this != &other)
			{
				n = other.n;
				to = other.to;
				from = other.from;
				DeltaCost = other.DeltaCost;
				DeltaDistance = other.DeltaDistance;
				DeltaRec = other.DeltaRec;
				IsFeasible = other.IsFeasible;
				move = other.move;
				pos = other.pos;
				RecourseLb = other.RecourseLb; // I forgot to add and it lead to malloc run time error
			}
			return *this;
		}
		
		//For emplace_back
		Move(Node* n_,
			 Driver* to_,
			 Driver* from_,
			 double DeltaCost_,
			 double DeltaDistance_,
			 double DeltaRec_,
			 bool IsFeasible_,
			 Node* prev, // just the Node pointer
			 int pos_,
			 int RecourseLb_)
			: n(n_),
			  to(to_),
			  from(from_),
			  DeltaCost(DeltaCost_),
			  DeltaDistance(DeltaDistance_),
			  DeltaRec(DeltaRec_),
			  IsFeasible(IsFeasible_),
			  move(), // default construct first
			  pos(pos_),
			  RecourseLb(RecourseLb_)
		{
			move.prev = prev; 
		}

		
		
		void Show(){printf("Mo n:%d DriverTo:%d DriverFrom:%d IsFeas:%d dCost:%.1lf dDist:%.1lf dRec:%.1lf prev:%d pos:%d\n",n==NULL?-1:n->no,to==NULL?-1:to->id,from==NULL?-1:from->id,IsFeasible,DeltaCost,DeltaDistance,DeltaRec,move.prev == NULL ? -1: move.prev->no, pos);}

		bool operator < (const Move & m) const
		{
			return (DeltaCost < m.DeltaCost);
		}

		bool operator < (const Move * m) const
		{
			return (DeltaCost < m->DeltaCost);
		}

};

struct RecourseLbComparator {
    template <typename Node, typename Driver, typename MoveT>
    bool operator()(const Move &m1, const Move &m2) const {
        return m1.RecourseLb < m2.RecourseLb;
    }
};

#endif
