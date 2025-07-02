/*
 * Copyright Jean-Francois Cote 2012
 *
 * The code may be used for academic, non-commercial purposes only.
 *
 * Please contact me at cotejean@iro.umontreal.ca for questions
 *
 * If you have improvements, please contact me!
 *
*/

#ifndef DRIVER_BRPCS
#define DRIVER_BRPCS

class Driver
{
	public:
		Driver() : id(-1), StartNodeID(-1), EndNodeID(-1), capacity(-1), sum_demand(-1), curDistance(-1), curRecourse(-1), sum_q(0), sum_h_i0(0),sum_W(0),sum_q_e(0),sum_h_e_i0(0), is_feasible(true), nb_customers(0) {}
		int id;				//from 0 to nb drivers - 1
		int StartNodeID;	//
		int EndNodeID;		//
		int capacity;
		int sum_demand;
		double curDistance;
		double curRecourse;	
		
		/*-----QuantitiesForFeasibilityCheck------*/
		int sum_q; int sum_h_i0; int sum_W;
		int sum_q_e; int sum_h_e_i0;
		bool has_uncharged_bikes;

		void ResetFeasibilityQuantities(){ sum_q = 0; sum_h_i0 = 0; sum_W = 0;
											sum_q_e = 0; sum_h_e_i0 = 0; is_feasible = false; 
											 has_uncharged_bikes = false; }
		
		/*----------------------------------------*/		
		
		bool is_feasible;
		int16_t nb_customers;
		void SetCapacity(int q){ capacity = q; } 
		
		void Show() { printf("Driver:%d Start:%d End:%d Q:%d nbCust:%d sum_q:%d sum_q_e:%d Dist:%.2lf Cost:%.1lf isFeas:%d\n",id,StartNodeID,EndNodeID,capacity,nb_customers,sum_q,sum_q_e,curDistance,curRecourse,is_feasible); }

		int GetStartNodeId(){return StartNodeID;}
		int GetEndNodeId(){return EndNodeID;}
};

#endif
