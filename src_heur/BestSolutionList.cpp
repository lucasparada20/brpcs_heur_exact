#include "BestSolutionList.h"

Sol* BestSolutionList::GetSolution(int i)
{ 
	sol_list_it it = _list.begin();
	int j = 1;
	while(j++ < i) it++;
	return *it;
}

void BestSolutionList::GetSolutions(std::vector<Sol*> & v)
{
	sol_list_it it = _list.begin();
	for(;it != _list.end();it++)
		v.push_back(*it);
}

void BestSolutionList::Add(Sol & s)
{
	double d1 = s.GetLastCalculatedCost();
	if(_list.size() == (size_t) _max_list_count)
	{
		sol_list_it it = _list.end();
		it--;
		double d2 = (*it)->GetLastCalculatedCost();
		if(d2 <= d1) 
			return;
		else
		{
			Sol* sol = *it;
			_list.erase(it);
			
			for(size_t i = 0 ; i < _related_functions.size() ; i++)
				_related_functions[i]->Decrease(*sol);
			delete sol;
		}
	}

	for(size_t i = 0 ; i < _related_functions.size() ; i++)
		_related_functions[i]->Increase(s);

	sol_list_it it = _list.begin();
	while(it != _list.end())
	{
		double d2 = (*it)->GetLastCalculatedCost();
		
		//avoid solutions with exactly the same cost
		if(d1 == d2) 
			return;
		else if(d1 < d2)
		{
			Sol * newsol = new Sol(s);
			_list.insert(it, newsol);
			break;	
		}
		it++;	
	}
	if(it == _list.end())
	{
		Sol * newsol = new Sol(s);
		_list.push_back(newsol);
	}
}


void BestSolutionList::Add(BestSolutionList & list)
{
	_max_list_count += list._max_list_count;
	sol_list_it it = list._list.begin();
	while(it != list._list.end())
	{
		_list.push_back( (*it) );
		it++;	
	}
	list._list.clear();
}

void BestSolutionList::Show()
{
	printf("List of best solutions count:%d\n",(int)_list.size() );
	sol_list_it it = _list.begin();
	int i = 0;
	while(it != _list.end())
	{
		double d = (*it)->GetLastCalculatedCost();
		printf("i:%d cost:%.3lf\n", i++ ,d);
		it++;
	}
	
}

void BestSolutionList::Resize(int size)
{
	if(size < _list.size())
	{
		printf("Cannot resize this list to a smaller size. Current size:%d new size:%d\n",(int)_list.size(),size);
		exit(1);
	}
	_max_list_count = size;
}