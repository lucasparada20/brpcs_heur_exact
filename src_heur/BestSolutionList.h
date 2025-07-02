#ifndef BESTSOLUTIONLIST_H
#define BESTSOLUTIONLIST_H

#include <stdio.h>
#include "Solution.h"
#include "RelatednessRemoveBRPCS.h"
#include <list>

class BestSolutionList
{
	public:
		BestSolutionList(Prob * prob,int max_count):
										_prob(prob), _max_list_count(max_count), _list(0), _related_functions(0),_nbitems(0){}
		~BestSolutionList()
		{
			
			while(_list.size() > 0)
			{
				sol_list_it it = _list.begin();
				Sol* s = *it;
				_list.erase(it);
				delete s;
			}
			_list.clear();
		}
		
		Sol* GetSolution(int i);
		
		void GetSolutions(std::vector<Sol*> & v);
		
		int GetSolutionCount(){return (int)_list.size();}
		Prob* GetProblemDefinition(){return _prob;}
	
		void Add(RelatednessRemoveBRPCS * func){ _related_functions.push_back(func); }
	
		void Add(Sol & s);

		void Add(BestSolutionList & list);
	
		void Show();
	
		void Resize(int size);
		
		int GetSize(){return _max_list_count;}
		
	private:
		
	Prob * _prob;
	int _max_list_count;
	std::list< Sol* > _list; //sorted list of solution
	std::vector< RelatednessRemoveBRPCS* > _related_functions;
	typedef typename std::list< Sol* >::iterator sol_list_it;
	int _nbitems;
};

#endif

