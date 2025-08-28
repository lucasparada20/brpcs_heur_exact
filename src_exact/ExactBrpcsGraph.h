

#ifndef EXACT_BRP_GRAPH_H
#define EXACT_BRP_GRAPH_H

#include <algorithm>
#include <vector>
#include <map>
#include <iostream>
#include "../src_heur/NodeBRPCS.h"
#include "../src_heur/DriverBRPCS.h"
#include "../src_heur/ProblemDefinition.h"
#include "../src_heur/RouteFeasibilityBRPCS.h"
#include "../src_heur/Parameters.h"
#include "Network.h"

#define EXACT_SBRP_MIN_VIOLATION 0.2

class ExBrpArcO
{
public:
	Node* from;
	Node* to;
	int index;
	double cost;
	double time;
	double value;
	char walked_on;
	const void Show() { printf("a:%d from:%d to:%d cost:%.1lf value:%.1lf\n",
						index, from->no, to->no, cost, value); }
};

class ExBrpSortPath
{
public:
	int path_no;
	int nb_customers;
};

class ExBrpSorterPath
{
public:
	bool operator()(const ExBrpSortPath& a, const ExBrpSortPath& b)
	{
		if (a.nb_customers - b.nb_customers != 0)
			return a.nb_customers < b.nb_customers;
		else
			return a.path_no < b.path_no;
	}
};

class ExactBrpGraphO
{
public:
	ExactBrpGraphO(Prob* prob, RouteFeasibility * _r);

	int GetNodeCount() { return (int)_nodes.size(); }
	Node* GetNode(int index) { return _nodes[index]; }
	Node* GetDepot() { return _nodes[0]; }

	int GetArcCount() { return (int)_arcs.size(); }
	ExBrpArcO* GetArc(int index) { return &_arcs[index]; }
	ExBrpArcO* GetArc(int from, int to);

	//Originally set in the constructor
	int GetDemandMax() { return demandMax; }

	int GetPosArcCount() { return (int)_pos_arcs.size(); }
	int GetArcsOfCount(int index) { return (int)_arcs_of[index].size(); }
	int GetArcsInOfCount(int index) { return (int)_arcs_in_of[index].size(); }
	int GetArcsOutOfCount(int index) { return (int)_arcs_out_of[index].size(); }
	int GetArcsOfPosCount(int i) { return (int)_pos_arcs_of[i].size(); }
	int GetArcsInPosCount(int i) { return (int)_pos_arcs_in[i].size(); }
	int GetArcsOutPosCount(int i) { return (int)_pos_arcs_out[i].size(); }

	ExBrpArcO* GetArcsOfPos(int i, int j) { return _pos_arcs_of[i][j]; }
	ExBrpArcO* GetArcOf(int index, int j) { return _arcs_of[index][j]; }
	ExBrpArcO* GetArcInOf(int index, int j) { return _arcs_in_of[index][j]; }
	ExBrpArcO* GetArcOutOf(int index, int j) { return _arcs_out_of[index][j]; }
	ExBrpArcO* GetPosArc(int i) { return _pos_arcs[i]; }
	ExBrpArcO* GetArcsInPos(int j, int i) { return _pos_arcs_in[j][i]; }
	ExBrpArcO* GetArcsOutPos(int i, int j) { return _pos_arcs_out[i][j]; }
	ExBrpArcO* GetArcsPos(int from, int to);
	void UnWalkOnPosArc(int i){_pos_arcs[i]->walked_on = 0;}
	void WalkOnPosArc(std::vector<ExBrpArcO*> CycleArcs)
	{
		for(size_t i = 0; i < CycleArcs.size(); i++)
			for (size_t j = 0; j < _pos_arcs.size(); j++)
			{
			  if(CycleArcs[i]->index == _pos_arcs[j]->index)
			  {
				_pos_arcs[j]->walked_on = 1;
				break;
			  }
			}
	}
  
	void UnWalkOnArc(int i){_arcs[i].walked_on = 0;}
	void WalkOnArc(int i){_arcs[i].walked_on = 1;}
	void WalkOnArc(std::vector<ExBrpArcO*> CycleArcs){
		for(size_t i = 0; i < CycleArcs.size(); i++)
			for (size_t j = 0; j < _arcs.size(); j++)
			{
			  if(CycleArcs[i]->index == _arcs[j].index)
			  {
				_arcs[j].walked_on = 1;
				break;
			  }
			}
	}
  
	void WalkOnPosArc(int i){_pos_arcs[i]->walked_on = 1;}

	int GetPathCount() { return (int)_paths.size(); }
	std::vector<Node*>& GetPath(int i) { return _paths[i]; }
	std::vector<Node*>& GetSortedPath(int i) { return _paths[_sorted_paths[i].path_no]; }

	double GetSumArcValue(int node)
	{
		double v = 0;
		for (int i = 0; i < GetArcsOfPosCount(node); i++)
			v += GetArcsOfPos(node, i)->value;
		return v;
	}

	double GetTheta(int i){return thetas[i];}
	void SetTheta(int i, double v){thetas[i] = v;}
	void SetThetasSize(int size){thetas.resize(size);}

	double GetCost() { return _cost; }
	bool IsInteger() { return _is_integer; }
	void AssignPositiveValues();
	void ShowPosValueArcs();
	void ShowValueArcs();
	void MakePaths();
	void ShowPaths();
	void PrintGraph(char* filename);

	void SetNodeID(int nodeid) { _nodeid = nodeid; }
	int GetNodeID() { return _nodeid; }

	Prob* GetProblem() { return _prob; }
	int removed_arcs;
	
private:
	
	int _nodeid;
	Prob* _prob;
	RouteFeasibility * _r;
	std::vector<Node*> _nodes;	//nodes [0, ..., n] 0 is the depot, others are customers
	std::vector< ExBrpArcO > _arcs;
	std::vector< std::vector<ExBrpArcO*> > _arcs_of;
	std::vector< std::vector<ExBrpArcO*> > _arcs_in_of;
	std::vector< std::vector<ExBrpArcO*> > _arcs_out_of;

	std::vector< ExBrpArcO* > _pos_arcs; //arcs with a positive value
	std::vector< std::vector<ExBrpArcO*> > _pos_arcs_of; //arcs with a positive value
	std::vector< std::vector<ExBrpArcO*> > _pos_arcs_out; //arcs with a positive value
	std::vector< std::vector<ExBrpArcO*> > _pos_arcs_in; //arcs with a positive value

	std::vector< std::vector<Node*> > _paths;
	std::vector< ExBrpSortPath > _sorted_paths;
	std::map<int, ExBrpArcO*> _map_arcs;
	std::vector<std::vector<ExBrpArcO*>> _matrix_of_arcs;
	std::map<int, ExBrpArcO*> _map_pos_arcs;
	int demandMax;
	bool _is_integer;
	double _cost;

	std::vector<double> thetas;
	double recourse_theta;
};


#endif
