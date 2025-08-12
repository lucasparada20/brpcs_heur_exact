#include "Parameters.h"
#include "Station.h"
#include "MakeUncharged.h"
#include "MakeUnchargedSW.h"
#include "MakeFeasible.h"
#include "MakeFeasibleSW.h"
#include "Load.h"
#include "FileSystemIO.h"
#include "RandomNumbers.h"
#include "PrintInstance.h"
#include "StoreDepot.h"
#include "RoundToNearestTenth.h"
#include "CheckMaxDistance.h"
#include <cstring>
#include <string>
#include <map>

int main(int argc, char * argv[])
{
	std::map<std::string,std::string> parameters;
	
	ProcessKeywordParameters(argc,argv,parameters);
	
	if (parameters.find("city_name") == parameters.end() || parameters.find("seed") == parameters.end()) {
		printf("Wrong usage: You did not specify the station data, status and/or the instance type. \n");
		exit(1);
	}
	
	std::string city_name = (parameters.find("city_name"))->second;
	double uncharged_percentage = std::stod( (parameters.find("uncharged_percentage"))->second );
	int seed = std::stoi( (parameters.find("seed"))->second );
	std::map<std::string,std::string>::iterator it;
	it = parameters.find("bss_type");
	if (it == parameters.end())
	{
		printf("Phil Colins (1989). In this version, you need to specify the bss_type\n"); exit(1);
	}
	
	std::string bss_type = (parameters.find("bss_type"))->second;
	printf("bss_type:%s\n",bss_type.c_str());
	
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
	LoadJson(city_name,last_station_information,station_status_file_names,stations,rn,bss_type);
	
	// Attemtps to match the uncharged_percentage of bikes
	if(strcmp(bss_type.c_str(),"CS")==0)
		MakeUncharged(stations,uncharged_percentage,rn);
	else 
		MakeUnchargedSW(stations,uncharged_percentage,rn);
	
	// Checks for any infeasible node and makes it feasible
	if(strcmp(bss_type.c_str(),"CS")==0)
		MakeFeasible(stations);
	else 
		MakeFeasibleSW(stations);
	
	// Round station count down to nearest tenth
	//RoundToNearestTenth(stations,rn);
	
	CheckMaxDistance(stations);
	
	// Finally, print the instance
	PrintInstance(stations,city_name,uncharged_percentage,bss_type);
	
	return 0;
}

