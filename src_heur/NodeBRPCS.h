#ifndef NODE_BRPCS
#define NODE_BRPCS
#include <iostream>
#include <algorithm>
#include "constants.h" // defines NODE_TYPE_CUSTOMER ..


class Node {
public:
	//Default
    Node()
        : id(-1), no(-1), distID(-1), type(0), q(0), q_e(0), s(0), s_e(0), h_i(0), h_i0(0), h_e_i0(0), h_u_i0(0), maxWp(0), maxWm(0), is_chargeable(false), lat(0.0), lon(0.0) {} 
	// Parameterized constructor
    Node(int id, int no, int distID, int type, int q, int q_e, int s, int s_e, int h_i, int h_i0, int h_e_i0, int h_u_i0, int maxWp, int maxWm, bool is_chargeable, double lat, double lon)
        : id(id), no(no), distID(distID), type(type), q(q), q_e(q_e), s(s), s_e(s_e), h_i(h_i), h_i0(h_i0), h_e_i0(h_e_i0), h_u_i0(h_u_i0), maxWp(maxWp), maxWm(maxWm), is_chargeable(is_chargeable), lat(lat), lon(lon)  {}
	
	// Copy constructor
    Node(const Node& other)
        : id(other.id), no(other.no), distID(other.distID), type(other.type),
          q(other.q), q_e(other.q_e), s(other.s), s_e(other.s_e), h_i(other.h_i),
          h_i0(other.h_i0), h_e_i0(other.h_e_i0), h_u_i0(other.h_u_i0), 
          maxWp(other.maxWp), maxWm(other.maxWm),
          is_chargeable(other.is_chargeable), lat(other.lat), lon(other.lon) {}
	
	// Move constructor
	Node(Node&& other) noexcept
	: id(other.id), no(other.no), distID(other.distID), type(other.type), q(other.q), q_e(other.q_e), s(other.s), s_e(other.s_e), h_i(other.h_i), h_i0(other.h_i0), h_e_i0(other.h_e_i0), h_u_i0(other.h_u_i0), maxWp(other.maxWp), maxWm(other.maxWm), is_chargeable(other.is_chargeable), lat(other.lat), lon(other.lon), has_accumulated_uncharged(0) {
		other.id = -1; other.no = -1; other.distID = -1; other.type = 0; other.q = -1; other.q_e = -1; other.s = -1; other.s_e = -1; other.h_i = -1; other.h_i0 = -1; other.h_e_i0 = -1; other.h_u_i0 = -1; other.maxWp = -1; other.maxWm = -1; other.is_chargeable = false; other.lat = 0.0;other.lon = 0.0;
	}

	
    int id;
	int no;
	int origin_id;  //Legacy. Not used anymore (I think...) : ID at the begining, it is never modified, only customers have a valid unique ID
	int distID; //indicate which line and column that contains the distance/time in the matrices
	char type;		//type of the node
    int q;      // Regular request
    int q_e;    // Electric request

    int s;      // Regular target
    int s_e;    // Electric target

    int h_i;    // Capacity

    int h_i0;   // Regular init
    int h_e_i0; // Electric init
	int h_u_i0; // Uncharged init

    int maxWp;  // Maximum Wp
    int maxWm;  // Maximum Wm

    bool is_chargeable; // Whether the station can charge electric bikes

	double lat; //Not used in main_example
	double lon; //Not used in main_example
	
	int has_accumulated_uncharged;
	
	// -------------------- Feasibility quantities --------------- //
	bool is_end_feasible; bool is_start_feasible;
	int lambda; int mu; int min_lambda; int max_mu;
	int gamma; int min_gamma; int zeta; int max_zeta;
	
	void ResetFeasibilityQuantities(){
		is_end_feasible = false; is_start_feasible = false;
		lambda = 0; mu = 0; min_lambda = 0; max_mu = 0;
		gamma = 0; min_gamma = 0; zeta = 0; max_zeta = 0;
	}

	void Show() const {
		std::cout << "Node Id:" << id                  
				  //<< " s:" << s
				  //<< " s_e:" << s_e
				  << " q:" << q 
				  << " q_e:" << q_e 
				  << " cap:" << h_i 
				  << " wpR:" << h_i0 
				  << " wpE:" << h_e_i0
				  << " wpU:" << h_u_i0
				  << " wm:" << maxWm				  
				  << (is_chargeable ? " Chargeable" : " NonChar")  
				  //<< " EmptyDocks:" << maxWm
				  << " type:";

		// Check for the different node types using bitwise AND
		if (type & NODE_TYPE_CUSTOMER) {
			std::cout << "Cust";
		} else if (type & NODE_TYPE_START_DEPOT) {
			std::cout << "StartDepot";
		} else if (type & NODE_TYPE_END_DEPOT) {
			std::cout << "EndDepot";
		} else {
			std::cout << "Unknown";
		}

		//std::cout << " lat:" << lat << " lon:" << lon << std::endl;
		std::cout << std::endl;
	}


    void UpdateW() {
        maxWp = h_i0 + h_u_i0 + h_e_i0;
        maxWm = h_i - (h_i0 + h_u_i0 + h_e_i0);
    }

    // Overload the < operator for the comparators
    bool operator<(const Node& other) const {
        return id < other.id;
    }
};

struct NodeComparatorByid {
    bool operator()(const Node& a, const Node& b) {
        return a.id < b.id;
    }
};

//A comparator to try to sort in lease-cost ways ...
struct NodeComparator {
    bool operator()(const Node& a, const Node& b) {
        if (a.is_chargeable != b.is_chargeable) 
            return !a.is_chargeable; // Non-chargeable nodes first
        
        if (!a.is_chargeable) {		
			// For non-chargeable nodes: non-increasing order of h_u_i0
            //if (a.h_u_i0 != b.h_u_i0) {
            //    return a.h_u_i0 > b.h_u_i0; 
            //}
            // If h_u_i0 is the same, sort in increasing order of maxWm
            return a.maxWm < b.maxWm;
        }
        
		// For chargeable nodes: non-increasing order of maxWm
		if (a.is_chargeable)
			return a.maxWm > b.maxWm;
    }
};


#endif

