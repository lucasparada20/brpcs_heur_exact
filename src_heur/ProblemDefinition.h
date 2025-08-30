#ifndef PROBLEM_DEF_H
#define PROBLEM_DEF_H

#include "NodeBRPCS.h"
#include "DriverBRPCS.h"
#include <stddef.h>
#include <vector>
#include <cstring>
#include "Parameters.h"

class Prob
{
	public:
	Prob(): _nodes(0),_customers(0),_drivers(0),_distances(NULL), _dimension(0), _driver_count_lb(1), _upper_bound(9999999999.9), _delete_matrices(true), L(0)
	{
		_nodes.reserve(4000);		//to avoid costly reallocations
		_customers.reserve(4000);
		_drivers.reserve(200);

	}
	~Prob()
	{
		if(_delete_matrices)
		{
			if(	_distances != NULL)
			{
				for(int i = 0 ; i < _dimension ; i++)
					delete [] _distances[i];
				delete [] _distances;
			}
			_distances = NULL;
		}
	}

	void AddCustomer(Node * n){ _customers.emplace_back(n->id); }	
	void AddNode(Node & n){ _nodes.push_back(n); }
	
	//Overlead for std::move()
	void AddNode(Node&& n){ _nodes.push_back(std::move(n)); }

	int GetCustomerCount(){ return (int)_customers.size();}
	int GetNodeCount(){ return (int)_nodes.size();}

	Node* GetCustomer(int i){ return &_nodes[ _customers[i] ]; }
	Node* GetNode(int i){ return &_nodes[i]; }

	void AddDriver(Driver & d){ _drivers.push_back(d);}
	int GetDriverCount(){ return (int)_drivers.size();}
	Driver* GetDriver(int i){ return &_drivers[i];}

	void SetMaxtrices(double ** d, int dim){ _distances = d; _dimension = dim;}
	double ** GetDistances(){ return _distances;}
	double GetDistance(Node * i, Node * j){ return _distances[i->distID][j->distID];}
	double GetDist(Node * i, Node * j){ return _distances[i->distID][j->distID];}
	int GetDimension(){return _dimension;}

	void TruncateMatrices()
	{
		for(int i=0;i<_dimension;i++)
			for(int j=0;j<_dimension;j++)
			{
				_distances[i][j] = (int)_distances[i][j];
			}
	}

	void ShowNodes()
	{
		for(size_t i=0;i<_nodes.size();i++)
			if(_nodes[i].type == NODE_TYPE_CUSTOMER)
				_nodes[i].Show();
	}

	static void GetIdListNoDepot(std::vector<Node*> & nodes, std::vector<int> & ids)
	{
		ids.clear(); ids.reserve( (int)nodes.size() );
		for(size_t k=0;k<nodes.size();k++)
			if(nodes[k]->type == NODE_TYPE_CUSTOMER)
				ids.emplace_back(nodes[k]->id);
	}

	Node* GetNodeByOriginID(int id)
	{
		for(size_t i=0;i<_nodes.size();i++)
			if(_nodes[i].origin_id == id)
				return &(_nodes[i]);
		return NULL;
	}

	double GetUpperBound(){return _upper_bound;}
	void SetUpperBound(double ub){_upper_bound = ub;}

	int GetDriverCountLB(){return _driver_count_lb;}
	void SetDriverCountLB(int d){_driver_count_lb = d;}
	
	void SetL(int l){ L = l;}
	int GetL(){ return L;}
 	
	private:
	std::vector<Node> _nodes;			//list of nodes
	std::vector<int> _customers;		//list of nodes that are customers
	std::vector<Driver> _drivers;		//list of drivers

	double ** _distances;
	int _dimension;

	int _driver_count_lb;
	double _upper_bound;
	bool _delete_matrices;		//if the problem definition comes from a copy it is false,
										//if it is original it is true
	int L;

  
};

#endif
