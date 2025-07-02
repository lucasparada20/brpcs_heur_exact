#ifndef RELATEDNESS_H
#define RELATEDNESS_H

#include "Solution.h"
#include "OperatorBase.h"

class RelatednessRemoveBRPCS : public RemoveOperator
{
	public:
		RelatednessRemoveBRPCS(double ** m): _m(m){}	
	
		double GetRelatedness(Node * n1, Node* n2){ return _m[n1->distID][n2->distID];}

		void Remove(Sol & s, int count) override;
		
		//for those with memory (not this case)
		void Increase(Sol & s){};
		void Decrease(Sol & s){}; 
		
	private:
		std::vector<Node*> vect;
		std::vector<Node*> removed;
		
	private:
		double ** _m;
};


struct RelatedPairSorter
{
	Node* selectedNode;
	RelatednessRemoveBRPCS * rel;
	bool operator() (Node * n1,Node * n2)
	{ 
		return ( rel->GetRelatedness(n1,selectedNode) < rel->GetRelatedness(n2,selectedNode)); 
	}
};

#endif
