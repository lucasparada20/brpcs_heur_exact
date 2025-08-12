#include "LoadBRPCS.h"
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream> // std::istringstream

constexpr double EarthRadius = 6371.0; // in kilometers

double LoadBRPCS::CalculateHarvesineDistance(Node * n1, Node * n2) 
 {
    double lat1 = n1->lat;
    double lon1 = n1->lon;
    double lat2 = n2->lat;
    double lon2 = n2->lon;

    // Convert latitude and longitude from degrees to radians
    lat1 *= M_PI / 180.0;
    lon1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = EarthRadius * c;

    return distance;
}

void LoadBRPCS::LoadInstance(Prob & pr, char * filename)
{
    std::ifstream infile(filename);
    if (!infile)
    {
        std::cerr << "Error in the input filename: " << filename << std::endl;
        exit(1);
    }
	std::cout << "Loading filename:" << filename << std::endl;
		
    // Read the meta data
	std::string line;
    std::getline(infile, line);
	std::istringstream iss(line);
	
	//output from the generator is (to be read in the same order):
	//stations, sumCap, sumInitReg, sumInitElec, sumInitU, sumqReg, sumqElec, sumAbsReg, sumAbsElec
	std::string bss_type;
	int stations, nbCharging, sumCap, sumInitReg, sumInitElec, sumInitU, sumqReg, sumqElec, sumAbsReg, sumAbsElec;
	if(!(iss >> bss_type >> stations >> nbCharging >> sumCap >> sumInitReg >> sumInitElec >> sumInitU >> sumqReg >> sumqElec >> sumAbsReg >> sumAbsElec ))
	{
		std::cout << "Error in meta data line ..." << std::endl; 
		std::cout << line << std::endl;
		exit(1);
	}
	std::cout << "BSSType:" << bss_type << " Stations:" << stations << " nbCharging:" << nbCharging  
			  << " sumCap:" << sumCap << " sumInitReg:" << sumInitReg 
			  << " sumInitElec:" << sumInitElec << " sumInitU:" << sumInitU << " sumqReg:" << sumqReg
			  << " sumqElec:" << sumqElec << " sumAbsReg:" << sumAbsReg << " sumAbsElec:" << sumAbsElec << std::endl;
	
	Parameters::SetBSSType((char*)bss_type.c_str());
	printf("Bss type from load: %s\n",Parameters::GetBSSType() == 1? "CS" : "SW");
	
	std::vector<Node> nodes;
	nodes.reserve(stations+1);
	
	std::getline(infile, line);
	std::istringstream idepotss(line);
	int depotInt = -1; double depotLat = 0.0; double depotLon = 0.0;
	if( !( idepotss >> depotInt >> depotLat >> depotLon ) )
	{
		std::cout << "Depot line is missing data\n" << line << std::endl; exit(1);
	}
	Node depot;
	depot.id = 0; depot.distID = 0; depot.lat = depotLat; depot.lon = depotLon; 
	printf("Depot (read from .txt):\n"); depot.Show();
	//nodes.push_back(depot);
	
	for(int i=0;i<stations-1;i++)
	{
		std::getline(infile, line);
		std::istringstream inodess(line);
		
		//output from generator is: s.cap,s.charges,s.tgtRegular,s.tgtElectric,s.initRegular,s.initElectric,s.uElectric,s.lat,s.lon
		int cap, charges, tgtRegular, tgtElectric, initRegular, initElectric, uElectric; 
		double lat, lon;
		if(!(inodess >> cap >> charges >> tgtRegular >> tgtElectric >> initRegular >> initElectric >> uElectric >> lat >> lon)) // 9 items
		{
			std::cout << "line " << i+3 << " is missing data\n" << line << std::endl; 
			std::cout << "cap charges tgtReg tgtElec initReg initElec uElec lat lon" << std::endl;
			exit(1);
		}
		
		//std::cout << "Instance line: " << line << std::endl;
		//Copy the format from the brpod to ease the ALNS implementation ...
		//n.id = i; //n.no = i+1; //n.distID = i+1;
		nodes.emplace_back( i, //id
						i+1, //no
						i+1,  //distID
						NODE_TYPE_CUSTOMER,  //type
						initRegular - tgtRegular, // q
						initElectric - tgtElectric, //q_e
						tgtRegular, 
						tgtElectric, 
						cap, 
						initRegular, 
						initElectric, 
						uElectric,
						initRegular + initElectric + uElectric,
						cap - (initRegular + initElectric + uElectric),
						charges == 1 ? true : false,
						lat, 
						lon );
		//printf("Loaded node %d:\n",i); nodes[i].Show();
	}
	
	//std::cout << "Size of nodes vec:" << nodes.size() << std::endl;
	if((int)nodes.size() != stations-1)
	{
		std::cout << "Read an erroneous number of stations ... stations-1:" << stations-1 << std::endl; exit(1);
	}
	
	for(int i = 0; i< stations-1; i++)
		pr.AddNode(nodes[i]);
	
	for(int i = 0; i< stations-1; i++)	
		pr.AddCustomer( &nodes[i] );

	
	//std::cout << "Customers in Load:" << pr.GetCustomerCount() << std::endl;
	//for(int i=0; i<pr.GetCustomerCount();  i++)
		//pr.GetCustomer(i)->Show();
	
	//Add the depots : two for each customer
	int max_drivers = std::ceil( (stations-1)*0.1 );
	//int max_drivers = stations-1;
	for(int i = 0 ; i < std::min(max_drivers,50); i++)
	{
		Node dep1;
		dep1.id = stations - 1 + i*2;
		dep1.no = 0;
		dep1.distID = 0;
		dep1.type = NODE_TYPE_START_DEPOT;
		dep1.h_i = 0;
		dep1.lat = depot.lat;
		dep1.lon = depot.lon;

		Node dep2(dep1);
		dep2.id = stations + i*2;
		dep2.type = NODE_TYPE_END_DEPOT;
		dep2.h_i = 0;
		dep2.lat = depot.lat;
		dep2.lon = depot.lon;
		
		Driver d;
		d.capacity = 40;
		d.StartNodeID = dep1.id;
		d.EndNodeID = dep2.id;
		d.id = i;

		pr.AddDriver(d);
		//d.Show();
		pr.AddNode(std::move(dep1));
		pr.AddNode(std::move(dep2));
	}
	//printf("Sample driver:\n");
	//pr.GetDriver(0)->Show();
	
	int dim = nodes.size()+1;
	double ** d = new double*[dim];

	//Assumption for maximum route duration
	//1.-Amazon drivers drive, on average 100+ miles per day. We thus set the hard limit to <= 100 miles = 160 km
	//Parameters::SetMaxRouteDistance(160);
	Parameters::SetMaxRouteDistance(100.0);
	//Parameters::SetMaxRouteDistance(50.0);
	
	//printf("Distance Matrix:\n");
	// Order is:  Nodes 0 ... dim-2 are customers. Node dime-1 is the first depot created
	double max_distance = 0.0;
	for(int i=0;i<dim;i++)
	{
      Node * n1 = pr.GetNode(i);
      d[n1->distID] = new double[dim];
      for(int j=0;j<dim;j++)
      {
         Node * n2 = pr.GetNode(j);
		 //printf("distID:%d distID:%d\n",n1->distID,n2->distID);
		 //n1->Show();
		 //n2->Show();
		 
		d[n1->distID][n2->distID] = i==j ? 0 : CalculateHarvesineDistance(n1,n2);
		if(d[n1->distID][n2->distID] > max_distance)
			max_distance = d[n1->distID][n2->distID];
		//printf("distID:%d distID:%d = %d\n",n1->distID,n2->distID,(int)d[n1->distID][n2->distID]);
		//all_distances.push_back(d[n1->distID][n2->distID]); // Store the distance in the vector
		if(d[n1->distID][n2->distID] > 100.0) // An intercity distance of >100km should be wrong!
		{
			printf("distance:%.3lf Nodes:\n",d[n1->distID][n2->distID]); n1->Show(); n2->Show(); getchar();
		}
		/*if(d[n1->distID][n2->distID] < 0.001 && n1->distID != n2->distID) // An intercity distance of >100km should be wrong!
		{
			printf("distance:%.3lf Nodes:\n",d[n1->distID][n2->distID]); n1->Show(); n2->Show(); getchar();
		}*/

      }
	  //getchar();
	}
	printf("Max distance in the matrix:%.1lf\n",max_distance);
	
	/*for(int i=0; i<dim; i++)
	{
		Node * n1 = pr.GetNode(i);
		printf("distId:%d ",n1->distID);  
		for (int j = 0; j < dim; j++) 
			printf("%2.1lf ", d[n1->distID][j]);
		printf("\n"); 
	}*/
	
	pr.SetMaxtrices(d,dim);	
	
}

void LoadBRPCS::LoadSolution(Prob & pr, Sol & sol, char * filename)
{
    std::ifstream infile(filename);
    if (!infile)
    {
        std::cerr << "Error in the solution filename: " << filename << std::endl;
        exit(1);
    }
	std::cout << "Loading solution filename:" << filename << std::endl;
		
    // Read the meta data
	std::string line;
    std::getline(infile, line);
	std::istringstream file_line_1(line);
	int nb_stations = -1; int nb_routes = -1; char comma; // to discard commas
	if( !(file_line_1 >> nb_stations >> comma >> nb_routes) || nb_routes < 0)
	{
		printf("Error reading the first line. Exiting\n"); 
		printf("line:%s\n",line.c_str());
		exit(1);
	}
	
	std::vector<std::vector<int>> route_ints; route_ints.resize( nb_routes, std::vector<int>(0) );
	for (int i = 0; i < nb_routes; i++)
	{
		std::string header_line;
		if (!std::getline(infile, header_line)) {
			std::cout << "Unexpected EOF while reading header for route " << i << "\n";
			exit(1);
		}
		
		// --- Read header line ---
		std::istringstream header_stream(header_line);
		int route_id, nb_stations, sum_q, sum_qe;
		double dist;
		

		if (!(header_stream >> route_id >> comma >> nb_stations >> comma >> sum_q >> comma >> sum_qe >> comma >> dist)) {
			std::cerr << "Error parsing header line for route " << i << ": " << header_line << "\n";
			exit(1);
		}

		std::cout << "Route " << route_id << " | Stations: " << nb_stations
				  << " | SumQ: " << sum_q << " | SumQE: " << sum_qe
				  << " | Dist: " << dist << "\n";

		// --- Read sequence line ---
		std::string seq_line;
		if (!std::getline(infile, seq_line)) {
			std::cout << "Unexpected EOF while reading sequence for route " << i << "\n";
			exit(1);
		}

		std::vector<int> sequence; sequence.reserve(nb_stations+2);
		std::stringstream seq_stream(seq_line);
		std::string token;
		while (std::getline(seq_stream, token, '-')) {
			if (!token.empty()) {
				sequence.push_back(std::stoi(token));
			}
		}

		// Print sequence for debug
		std::cout << "Sequence:";
		for (int node : sequence) std::cout << " " << node;
		std::cout << "\n";
		
		route_ints[i] = sequence;
	}	
	
	for (int i = 0; i < nb_routes; i++)
	{
		std::vector<Node*> path; path.reserve( (int)route_ints[i].size() );
		for(int k=0;k<(int)route_ints[i].size();k++)
		{
			Node * n = pr.GetNode( route_ints[i][k] ); 
			path.emplace_back( n );
		}
			
		sol.MakePath(i,path);
	}

}