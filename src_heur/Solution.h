#ifndef _SOLUTION_H
#define _SOLUTION_H

#include <stdio.h>
#include <stdlib.h>
#include "ProblemDefinition.h"
#include "CostFunctionBRPCS.h"
#include <vector>
#include <sstream>
#include <iostream>
#include "Parameters.h"
#include "RouteFeasibilityBRPCS.h"
#include <algorithm>

class Sol
{

	public:
		Sol():show_output(true){}
		Sol(Prob * prob, CostFunctionBRPCS * cost_func):
										  Next(0),Prev(0),AssignTo(0),
										  RoutesLength(prob->GetDriverCount()),
										  _prob(prob),UnassignedCount(0),
										  Unassigneds(0),UnassignedIndex(0),
										  _cost_func(cost_func), _last_cost(0),_is_feasible(true),_total_distances(0),show_output(true)

		{
			//std::cout << "Solution constructor Nodes:" << _prob->GetNodeCount() << " Drivers:"  << _prob->GetDriverCount() << std::endl;
			
			for(int i = 0 ; i < _prob->GetNodeCount() ; i++)
			{
				Next.push_back(NULL);
				Prev.push_back(NULL);
				AssignTo.push_back(NULL);
				Unassigneds.push_back(NULL);
				UnassignedIndex.push_back(-1);
			}

			for(int i = 0 ; i < _prob->GetDriverCount() ; i++)
			{
				RoutesLength[i] = 0;
				Driver * d = GetDriver(i);
				Node * n1 = GetNode(d->StartNodeID);
				Node * n2 = GetNode(d->EndNodeID);
				//d->Show();
				//std::cout << "n1:" << std::endl; n1->Show();
				//std::cout << "n2:" << std::endl; n2->Show();
				Next[n1->id] = n2;
				Prev[n2->id] = n1;
				AssignTo[n1->id] = d;
				AssignTo[n2->id] = d;
			}
		}

		void AddNode()
		{
			Next.push_back(NULL);
			Prev.push_back(NULL);
			AssignTo.push_back(NULL);
			Unassigneds.push_back(NULL);
			UnassignedIndex.push_back(-1);
		}

		void AddDriver()
		{
			RoutesLength.push_back(0);
		}

		void InsertAfter(Node * n, Node * prev)
		{
			RoutesLength[ AssignTo[prev->id]->id ]++;
			AssignTo[n->id] = AssignTo[prev->id];
			// Print the IDs for debugging
			//printf("Assigning node %d to driver %d\n", n->id, AssignTo[n->id]->id);
			//printf("Node %d was previously assigned to driver %d\n", prev->id, AssignTo[prev->id]->id);
			Next[n->id] = Next[	prev->id ];
			Prev[n->id] = prev;
			if(Next[prev->id] != NULL)
				Prev[ Next[prev->id]->id ] = n;

			Next[prev->id] = n;
		}

		void Remove(Node * n)
		{
			RoutesLength[ AssignTo[n->id]->id ]--;
			if(Next[n->id] != NULL) Prev[ Next[n->id]->id ] = Prev[n->id];
			if(Prev[n->id] != NULL) Next[ Prev[n->id]->id ] = Next[n->id];
			AssignTo[n->id] = NULL;
		}

		void AddToUnassigneds(Node * n)
		{
			UnassignedIndex[n->id] = UnassignedCount;
			Unassigneds[UnassignedCount] = n;
			UnassignedCount++;
		}

		void RemoveFromUnassigneds(Node * n)
		{
			int ind = UnassignedIndex[n->id];
			//printf("RemoveFromUnassigneds:%d index:%d count:%d\n", n->id, ind,UnassignedCount);
			if(UnassignedCount > 1)
			{
				Node * rep = Unassigneds[UnassignedCount - 1];
				Unassigneds[ind] = rep;
				UnassignedIndex[ rep->id ] = ind;
			}

			Unassigneds[UnassignedCount - 1] = NULL;
			UnassignedCount--;
			UnassignedIndex[n->id] = -1;
		}
		void RemoveAndUnassign(Node * n)
		{
			Remove(n);
			AddToUnassigneds(n);
		}
		
		void UnassignAllCustomers()
		{
			for(int i=0;i<GetCustomerCount();i++)
			{
				Node * n = GetCustomer(i);
				if(AssignTo[n->id] != NULL)
					RemoveAndUnassign(n);
			}
		}

		void PutAllNodesToUnassigned()
		{
			for(int i=0;i<GetCustomerCount();i++)
				AddToUnassigneds( GetCustomer(i));
		}
		
		//Improved search for paths to merge
		/*void MergeAllPaths()
		{
			// This method requires a previous call to Update() to compute the distances of the drivers
			int initial_drvs = GetUsedDriverCount();
			std::vector<Driver*> drivers;
			
			// Collect all drivers with non-zero route lengths
			for(int i = 0; i < GetDriverCount(); i++)
			{
				Driver* d = _prob->GetDriver(i);
				if(RoutesLength[i] && d->curDistance < Parameters::MaxRouteDistance())
					drivers.push_back(d);
			}
			
			// Compare each unique pair of drivers
			for(size_t i = 0; i < drivers.size(); i++)
			{
				for(size_t j = i + 1; j < drivers.size(); j++)
				{
					Driver* d1 = drivers[i];
					Driver* d2 = drivers[j];

					// Check if the sum of distances is less than or equal to MaxRouteDistance
					if(d1->curDistance + d2->curDistance <= Parameters::MaxRouteDistance())
					{
						// Process the pair (d1, d2) as needed
						Node* last_d1_node = Prev[GetNode(d1->EndNodeID)->id];
						Node* n = GetNode(d2->StartNodeID);
						n = Next[n->id]; // Move to the first actual node in d2
						while(n->type != NODE_TYPE_END_DEPOT)
						{
							Node* next_in_d2 = Next[n->id];
							//Remove first
							Remove(n);
							// Insert n after last_d1_node
							InsertAfter(n, last_d1_node);
							// Update last_d1_node to n
							last_d1_node = n;
							// Move to the next node in d2
							n = next_in_d2;
						}

						Update(d1);
						Update(d2);

						// Remove merged drivers from the list to avoid double merging
						drivers.erase(drivers.begin() + j); // Remove d2
						drivers.erase(drivers.begin() + i); // Remove d1
						break; // Exit the inner loop to avoid accessing removed elements
					}
				}
			}

			printf("Initial Drvs:%d New Drivers:%d\n",initial_drvs, GetUsedDriverCount());
		}*/
		
		
		//Sequential search for paths to merge
		void MergeAllPaths()
		{
			//This method requires a previous call to Update() to compute the distances of the drivers
			int initial_drvs = GetUsedDriverCount();
			std::vector<Driver*> drivers;
			for(int i = 0; i < GetDriverCount(); i++)
			{
				Driver* d = _prob->GetDriver(i);
				if(RoutesLength[i] && d->curDistance < Parameters::MaxRouteDistance())
					drivers.push_back( d );
					//d->Show();
			}					
			//printf("MergeAllPaths drvs to merge:%d\n",(int)drivers.size());
			for(size_t i = 0; i < drivers.size()-1; i+=2)
			{
				if (i + 1 >= drivers.size()) break; // Ensure there is a next driver available

				Driver* d1 = drivers[i];
				Driver* d2 = drivers[i+1];
				
				if(Parameters::RecoursePolicy() == 1  &&  d1->curDistance + d2->curDistance > Parameters::MaxRouteDistance())
					continue;
				if(Parameters::RecoursePolicy() == 2  &&  d1->curDistance + d1->curRecourse + d2->curDistance + d2->curRecourse > Parameters::MaxRouteDistance()) 	continue;
				
				//printf("Merging d%d dist:%.1lf with d%d dist:%.1lf ...\n",d1->id,d1->curDistance,d2->id,d2->curDistance);
				//Show(d1); Show(d2);
				Node* last_d1_node = Prev[ GetNode(d1->EndNodeID)->id ];
				
				// Traverse nodes of d2
				Node* n = GetNode(d2->StartNodeID);
				n = Next[n->id]; // Move to the first actual node in d2
				while(n->type != NODE_TYPE_END_DEPOT)
				{
					Node* next_in_d2 = Next[n->id];
					//Remove first
					Remove(n);
					// Insert n after last_d1_node
					InsertAfter(n, last_d1_node);
					// Update last_d1_node to n
					last_d1_node = n;
					// Move to the next node in d2
					n = next_in_d2;
				}
				
				Update(d1); Update(d2);
				if(d1->curRecourse > 9990)
					_is_feasible = false;
				/*printf("New d%d\n",d1->id);
				Show(d1);
				printf("New d%d\n",d2->id);
				Show(d2);
				getchar();*/
			}
			//printf("Initial Drvs:%d New Drivers:%d\n",initial_drvs, GetUsedDriverCount());
		}

		
		//Revert the path of n1 to n2 inclusively
		void RevertPath(Node * n1, Node * n2)
		{
			Node * prev = Prev[n1->id];
			Node * next = Next[n2->id];

			Node * cur = n1;
			Node * last = next;
			Node * prevlast = n2;
			while(cur != n2)
			{
				Node * tmp = Next[cur->id];
				Next[cur->id] = last;
				Prev[last->id] = cur;
				Prev[cur->id] = prevlast;
				Next[prevlast->id] = cur;

				last = cur;
				cur = tmp;
			}

			Next[prev->id] = n2;
			Prev[n2->id] = prev;
		}
		//Take the set of nodes from s1 to e1 and swap it with the set of nodes from s2 to e2
		//e1 and e2 are included
		void SwapTwoChains(Node* s1, Node* e1, Node* s2, Node* e2)
		{
			 //printf("s1:%d e1:%d s2:%d e2:%d\n",s1->id,e1->id,s2->id,e2->id);
			 Node* prev_s1 = Prev[s1->id];
			 Node* prev_s2 = Prev[s2->id];

			 Node* next_e1 = Next[e1->id];
			 Node* next_e2 = Next[e2->id];

			 Driver* d1 = AssignTo[s1->id];
			 Driver* d2 = AssignTo[s2->id];

			 Next[prev_s1->id] = s2;
			 Next[prev_s2->id] = s1;

			 Prev[next_e1->id] = e2;
			 Prev[next_e2->id] = e1;

			 Prev[s1->id] = prev_s2;
			 Prev[s2->id] = prev_s1;

			 Next[e1->id] = next_e2;
			 Next[e2->id] = next_e1;

			 Node* n = s1;
			 int cnt1 = 0;
			 while(n != next_e2)
			 {
				  AssignTo[n->id] = d2;
				  n = Next[n->id];
				  cnt1++;
			 }

			 n = s2;
			 int cnt2 = 0;
			 while(n != next_e1)
			 {
				  AssignTo[n->id] = d1;
				  n = Next[n->id];
				  cnt2++;
			 }
			 RoutesLength[d1->id] += (-cnt1+cnt2);
			 RoutesLength[d2->id] += (-cnt2+cnt1);
		}

		void RelocateChain(Node* s, Node* e, Node* prev)
		{
			Driver* receiving      = AssignTo[prev->id];
			Driver* giving         = AssignTo[s->id];
			Node* next     = Next[prev->id];
			Node* prev_s   = Prev[s->id];
			Node* next_e   = Next[e->id];

			Next[prev_s->id] = next_e;
			Prev[next_e->id] = prev_s;

			Next[prev->id]  = s;
			Prev[s->id]     = prev;
			Prev[next->id]  = e;
			Next[e->id]     = next;

			Node* n = s;
			int cnt = 0;
			while(n != next)
			{
				AssignTo[n->id] = receiving;
				n = Next[n->id];
				cnt++;
			}
			RoutesLength[receiving->id] += cnt;
			RoutesLength[giving->id] -= cnt;
		}

		void MakePath(int driver, std::vector<Node*> & path)
		{
			Driver * d = GetDriver(driver);
			Node * prev = GetNode(d->StartNodeID);
			for(size_t j=0;j<path.size();j++)
				//if((path[j]->type & NODE_TYPE_CUSTOMER) == NODE_TYPE_CUSTOMER)
				if(path[j]->type == NODE_TYPE_CUSTOMER)
				{
					if(AssignTo[ path[j]->id ] != NULL)
						Remove( path[j] );
					if( UnassignedIndex[ path[j]->id ] != -1)
						RemoveFromUnassigneds( path[j] );
					InsertAfter(path[j], prev);
					prev = path[j];
				}
		}

		void GetPath(Driver * d, std::vector<Node*> & path)
		{
			path.clear();
			Node * n = GetNode(d->StartNodeID);
			while(n != NULL)
			{
				path.push_back(n);
				n = Next[n->id];
			}
		}

		//return true if the solution has a route that contains only one customer
		bool ContainSingleCustomerRoute()
		{
			for(int i=0;i<GetDriverCount();i++)
				if(RoutesLength[i] == 1)
					return true;
			return false;
		}

		unsigned long long GetDriverRouteKey(Driver * d)
		{
			unsigned long long l = 1;
			Node * n = GetNode( d->StartNodeID );
			while(n != NULL)
			{
				if((n->type & NODE_TYPE_CUSTOMER) == NODE_TYPE_CUSTOMER)
					l *= prime_get_ith(n->id);
				n = Next[ n->id ];
			}
			return l;
		}

		void GetDriverRouteString(Driver * d, std::string & s)
		{
			std::ostringstream sstream;
			Node * n = GetNode( d->StartNodeID );
			while(n != NULL)
			{
				if((n->type & NODE_TYPE_CUSTOMER) == NODE_TYPE_CUSTOMER)
					sstream << n->origin_id << "-";
				n = Next[ n->id ];
			}
			s = sstream.str();
		}
		//Make the solution feasible by checking every route and removing infeasible customers until the route is feasible
		void MakeFeasible()
		{
			//printf("MakeFeasible() Drivers:%d\n",GetDriverCount()); getchar();
			int cntr=0;
			for(int i=0;i<GetDriverCount();i++)
			{
				if(GetDriver(i)->nb_customers == 0) continue;
				RemoveInfeasibleCustomers_HC(GetDriver(i)); //The function from the sbrpod but with modified slacks, surplus
				//_cost_func->Update(*this,GetDriver(i)); // No need to call because later insert calls Update()

			}
			//printf("Infeasible Drivers:%d\n",cntr);
		}
		void RemoveInfeasibleCustomers_HC(Driver * d)
		{
			int Q = d->capacity;
			int sumLambda = 0; int minLambda = 0; int prevMinLambda = 0; // Store previous minLambda
			int sumMu = 0; int maxMu = 0; int prevMaxMu = 0; // Store previous maxMu
	 
			int cntr=0; std::vector<int> RmvCust;
			Node * prev = GetNode(d->StartNodeID);
			//printf("RmvInfCust d_id:%d Nodes:%d-",d->id,prev->no);
			while (prev->type != NODE_TYPE_END_DEPOT)
			{
				Node * next = Next[prev->id];
				//printf("%d-",next->no);
				int combined_demand = next->q + next->q_e;
				int lambda_i = std::max(-Q, combined_demand - next->h_e_i0 - next->h_i0);
				int mu_i = std::min(Q, combined_demand + next->maxWm);
				
				// End load feasibility
				sumLambda += lambda_i;
				minLambda = std::min(sumLambda, minLambda);
				sumMu += mu_i;
				maxMu = std::max(sumMu, maxMu);
				int lb = sumLambda - minLambda;
				int ub = sumMu + Q - maxMu;
				if (lb > ub)
				{
					RemoveAndUnassign(next);
					RmvCust.push_back(next->no);
					sumLambda -= lambda_i;
					sumMu -= mu_i;
					// Restore minLambda and maxMu
					minLambda = prevMinLambda;
					maxMu = prevMaxMu;
					cntr++;
				}
				else
				{
					// Update prevMinLambda and prevMaxMu
					prevMinLambda = minLambda;
					prevMaxMu = maxMu;
				}
				prev = next;
			}
			if(RmvCust.size()>0)
			{
				printf("Continue-to-Next RecoursePolicy:%d CustRmv:%d Customers:\n",Parameters::RecoursePolicy(),cntr);
				for(size_t i=0;i<RmvCust.size();i++)
				{
					printf("%d ",RmvCust[i]);
				}
				printf("\n");
				d->nb_customers -= RmvCust.size();
			}
			//getchar();
		}		
		
		
		void Show()
		{
			int nbnonempty = 0;
			for(int i=0;i<GetDriverCount();i++)
				if(RoutesLength[i])
					nbnonempty++;
			printf("Solution non-empty routes:%d routes:%d cost:%.2lf\n", nbnonempty,GetDriverCount(), _cost_func->GetCost(*this) );
			for(int i=0;i<GetDriverCount();i++)
				if(show_output && RoutesLength[i] >= 1)
					Show(GetDriver(i));

			if(show_output && GetUnassignedCount() >= 1)
			{
				printf("Unassigneds:");
				for(int i=0;i<GetUnassignedCount();i++)
					printf("%d ", GetUnassigned(i)->no);
				printf("\n");
			}
		}

		void Show(Driver * d)
		{
			if(_cost_func != NULL)
				_cost_func->Show(this, d);
		}

		//fill the list of nodes from the route i
		void GetRoute(int i, std::vector<Node*> & nodes)
		{
			nodes.clear();
			Node * cur = GetNode( GetDriver(i)->StartNodeID );
			while(cur != NULL)
			{
				nodes.push_back(cur);
				cur = Next[ cur->id ];
			}
		}

		//fill the list of nodes from the route i
		//with only the customer nodes
		void GetRouteNoDepot(int i, std::vector<Node*> & nodes)
		{
			nodes.clear();
			Node * cur = GetNode( GetDriver(i)->StartNodeID );
			while(cur != NULL)
			{
				if((cur->type & NODE_TYPE_CUSTOMER) == NODE_TYPE_CUSTOMER)
					nodes.push_back(cur);
				cur = Next[ cur->id ];
			}
		}

		//fill the list of nodes from the route i
		//with only the customer nodes
		void GetIdRouteNoDepot(int i, std::vector<int> & nodes)
		{
			nodes.clear();
			Node * cur = GetNode( GetDriver(i)->StartNodeID );
			while(cur != NULL)
			{
				if( (cur->type & NODE_TYPE_CUSTOMER) == NODE_TYPE_CUSTOMER)
					nodes.push_back(cur->id);
				cur = Next[ cur->id ];
			}
		}

		Driver* GetAssignedTo(Node * n){ return AssignTo[n->id];}
		Driver* GetAssignedTo(int node_id){ return AssignTo[node_id];}

		int GetCustomerCount(){ return _prob->GetCustomerCount();}
		Node * GetCustomer(int i){ return _prob->GetCustomer(i);}

		int GetNodeCount(){ return _prob->GetNodeCount();}
		Node * GetNode(int i){ return _prob->GetNode(i);}

		int GetDriverCount(){ return _prob->GetDriverCount();}
		Driver * GetDriver(int i){ return _prob->GetDriver(i);}

		int GetUsedDriverCount()
		{
			int nb = 0;
			for(int i=0;i<GetDriverCount();i++)
				if(RoutesLength[i])	
					nb++;
			return nb;
		}
		int GetUnassignedCount(){ return UnassignedCount;}
		Node * GetUnassigned(int i){ return Unassigneds[i];}

		double GetCost(){ _last_cost = _cost_func->GetCost(*this); return _last_cost;}
		double GetCost(Driver * d){ return _cost_func->GetCost(*this,d);}
		double GetCost(int i){ return _cost_func->GetCost(*this,GetDriver(i));}
		double GetLastCalculatedCost(){ return _last_cost; }
		CostFunctionBRPCS * GetCostFunction(){return _cost_func;}

		bool IsFeasible(){return _is_feasible;}
		void SetIsFeasible(bool f){_is_feasible = f;}

		void Update(){ _cost_func->Update(*this);}
		void Update(Driver * d){ _cost_func->Update(*this,d);}

		double GetTotalDistances(){return _total_distances;}
		void SetTotalDistances(double d){_total_distances = d;}

		double GetTotalRecourse(){return _total_recourse;}
		void SetTotalRecourse(double d){_total_recourse = d;}

		bool IsUnassigned(Node * n){return UnassignedIndex[n->id] != -1;}

		Prob * GetProblemDefinition(){return _prob;}
		Prob * GetProb(){return _prob;}

		double ** GetDistances(){ return _prob->GetDistances();}
		double GetDist(Node * n1, Node * n2){return _prob->GetDistances()[n1->distID][n2->distID];}
		int GetRouteLength(int i){return RoutesLength[i];}
		int GetRouteLength(Driver * d){return RoutesLength[d->id];}

		std::vector<Node*> Next;
		std::vector<Node*> Prev;
		std::vector<Driver*> AssignTo;
		std::vector<int> RoutesLength;//# of customers in each route

		bool show_output;
   
		void RevertPath(Driver * d)
		{
			RevertPath( GetNext(GetNode(d->StartNodeID)), GetPrev(GetNode(d->EndNodeID)) );
		}
		Node * GetNext(Node * n){return Next[n->id];}
		Node * GetPrev(Node * n){return Prev[n->id];}
    
	private:
		Prob * _prob;
		int UnassignedCount;
		std::vector<Node*> Unassigneds;
		std::vector<int> UnassignedIndex;
		CostFunctionBRPCS * _cost_func;

		double _last_cost;
		bool _is_feasible;
		double _total_distances;
		double _total_recourse;
};

#endif
