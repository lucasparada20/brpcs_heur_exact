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
	
	//Assumption for maximum route duration
	//1.-Amazon drivers drive, on average 100+ miles per day. We thus set the hard limit to <= 100 miles = 160 km
	//Parameters::SetMaxRouteDistance(160);
	Parameters::SetMaxRouteDistance(100);
	
    // Read the meta data
	std::string line;
    std::getline(infile, line);
	std::istringstream iss(line);
	
	//output from the generator is (to be read in the same order):
	//stations, sumCap, sumInitReg, sumInitElec, sumInitU, sumqReg, sumqElec, sumAbsReg, sumAbsElec
	int stations, sumCap, sumInitReg, sumInitElec, sumInitU, sumqReg, sumqElec, sumAbsReg, sumAbsElec;
	if(!(iss >> stations >> sumCap >> sumInitReg >> sumInitElec >> sumInitU >> sumqReg >> sumqElec >> sumAbsReg >> sumAbsElec ))
	{
		std::cout << "Error in meta data line ..." << std::endl; exit(1);
	}
	std::cout << "Stations:" << stations << " sumCap:" << sumCap << " sumInitReg:" << sumInitReg 
			  << " sumInitElec:" << sumInitElec << " sumInitU:" << sumInitU << " sumqReg:" << sumqReg
			  << " sumqElec:" << sumqElec << " sumAbsReg:" << sumAbsReg << " sumAbsElec:" << sumAbsElec << std::endl;
	
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
	for(int i = 0 ; i < stations-1; i++)
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
	
	//printf("Distance Matrix:\n");
	// Order is:  Nodes 0 ... dim-2 are customers. Node dime-1 is the first depot created
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