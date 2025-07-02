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

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define NODE_TYPE_CUSTOMER 0x01 // 00000001
#define NODE_TYPE_START_DEPOT 0x02 // 00000010
#define NODE_TYPE_END_DEPOT 0x04 // 00000100
#define NODE_TYPE_PICKUP 0x03
#define NODE_TYPE_DROP 0x05

#define STATUS_UNDETERMINED 2		//we tried to solve but we couldn't
#define STATUS_SOLVED 1				//it has been solved!
#define STATUS_UNSOLVED 0			//not yet solved
#define STATUS_PACK_TIMEOUT 3		//got a packing timeout
#define STATUS_ROUTING_TIMEOUT 4	//got a timeout from a routing routine
#define STATUS_TIMEOUT 7			//got a timeout
#define STATUS_FEASIBLE 5			//the problem is feasible
#define STATUS_INFEASIBLE 6			//the problem is infeasible
#define STATUS_SET_INFEASIBLE 8		//the set of all routes is packing infeasible
#define STATUS_SET_FEASIBLE 9		//the set of all routes is packing feasible

#define INF_ROUTE_COST 1000000
#define UNASSIGNED_COST 1000000

#define EPSILON 10E-5
#define INF   99999999
#define INFINITE 99999999.9

#endif
