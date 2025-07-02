#include "Parameters.h"
#include "Station.h"
#include "MakeUncharged.h"
#include "MakeFeasible.h"
#include "Load.h"
#include "FileSystemIO.h"
#include "RandomNumbers.h"
#include "PrintInstance.h"
#include "StoreDepot.h"
#include "RoundToNearestTenth.h"
#include <string>
#include <map>

int main(int argc, char * argv[])
{
	std::map<std::string,std::string> parameters;
	
	ProcessKeywordParameters(argc,argv,parameters);
	
	if (parameters.find("city_name") == parameters.end() || parameters.find("seed") == parameters.end()) {
		printf("Wrong usage: You did not specify the station data, status and/or the instance type. Exiting. Hit `make usage' for options ... \n");
		exit(1);
	}
	
	std::string city_name = (parameters.find("city_name"))->second;
	double uncharged_percentage = std::stod( (parameters.find("uncharged_percentage"))->second );
	int seed = std::stoi( (parameters.find("seed"))->second );
	
	std::vector<std::string> station_information_file_names;
	std::vector<std::string> station_status_file_names;
	
	// Stores all file name strings and sorts from earliest to latest 
	LoadFileNames(city_name,station_information_file_names,station_status_file_names);
	
	std::vector<Station> stations;	
	
	// Creates a node object for the depot and hard codes the lat, lon
	StoreDepot(city_name,stations);
	
	// Use the most up-to-date station capacities
	std::string last_station_information = station_information_file_names[ station_information_file_names.size()-1 ];
	
	// Loads data from the Json files and generate targets
	RandomNumbers rn; rn.init( seed );
	LoadJson(city_name,last_station_information,station_status_file_names,stations,rn);
	
	// Attemtps to match the uncharged_percentage of bikes
	MakeUncharged(stations,uncharged_percentage,rn);
	
	// Checks for any infeasible node and makes it feasible
	MakeFeasible(stations);
	
	// Round station count down to nearest tenth
	//RoundToNearestTenth(stations,rn);
	
	// Finally, print the instance
	PrintInstance(stations,city_name,uncharged_percentage);
	
	return 0;
}

