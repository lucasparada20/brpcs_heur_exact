#include "Parameters.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <string.h>
#include <cstring>  // For strcpy, strstr, strtok, atoi
#include <cctype>   // For isdigit
#include <vector>
#include <limits>
#include <algorithm>
//#include <filesystem>

int Parameters::cost_policy = CN;
int Parameters::bss_type = CS;
std::string Parameters::instance_file;
std::string Parameters::re_file_name;
std::string Parameters::output_file_name;
std::string Parameters::city_name;
std::string Parameters::construction_heuristic;
std::string Parameters::initial_solution_file;
int Parameters::nb_stations=0;
int Parameters::u_value=0;
int Parameters::add_depot_stations=0;
double Parameters::max_route_distance=0.0;
int Parameters::recourse_policy=0;
int Parameters::delta = 1;
int Parameters::HardQ = 100;

void Parameters::SetCityName(const char* filePath) {
    if (!filePath) {
        std::cerr << "Error: filePath is NULL!" << std::endl;
        return;
    }

    // Print the received file path
    std::cout << "Received filePath: " << filePath << std::endl;

    // Convert C-string to std::string
    std::string full_path(filePath);

    // Extract directory path
    size_t last_slash = full_path.find_last_of('/');
    std::string base_path = (last_slash != std::string::npos) ? full_path.substr(0, last_slash) : ".";

    // Replace "instances" with "results"
    size_t pos = base_path.find("instances");
    if (pos != std::string::npos) {
        base_path.replace(pos, 9, "results");
    }

    // Extract city name and number
    size_t pos_start = last_slash + 1;
    size_t num_start = pos_start;
    
    while (num_start < full_path.size() && !isdigit(full_path[num_start])) {
        num_start++;
    }

    city_name = full_path.substr(pos_start, num_start - pos_start);
	
	size_t pos_underscore = full_path.find('_');
    std::string number_str = full_path.substr(num_start, pos_underscore - num_start);
	
	size_t pos_point = full_path.find('.'); 
	std::string u_value_str = full_path.substr(pos_underscore+1, pos_point - pos_underscore - 1);

    // Print extracted values for debugging
	std::cout << "pos_start: " << pos_start << " num_start: " << num_start << " underscore_start: " << pos_underscore << " pos_point: " << pos_point << std::endl;
    std::cout << "Extracted base path: " << base_path << std::endl;
    std::cout << "Extracted city name: " << city_name << std::endl;
    std::cout << "Extracted number string: " << number_str << std::endl;
	std::cout << "Extracted u value string: " << u_value_str << std::endl;

    // Ensure integers are is valid
    nb_stations = (!number_str.empty() && isdigit(number_str[0])) ? std::stoi(number_str) : 0;
	u_value = (!u_value_str.empty() && isdigit(u_value_str[0])) ? std::stoi(u_value_str) : 0;

    // Construct file paths in "results/"
    re_file_name = base_path + "/re_" + city_name + number_str;
    output_file_name = base_path + "/out_" + city_name + number_str;

    // Debug output
    std::cout << "In parameters parsing ..." << std::endl;
    std::cout << "City name: " << city_name << std::endl;
    std::cout << "NbStations: " << nb_stations << std::endl;
	std::cout << "U_value: " << u_value << std::endl;
    std::cout << "Re file name: " << re_file_name << std::endl;
    std::cout << "Out file name: " << output_file_name << std::endl;
}


/*void Parameters::SetCityName(const std::string& filePath) {
    // Extract the directory path of `instances/`
    std::filesystem::path full_path = std::filesystem::canonical(filePath);
    std::filesystem::path base_path = full_path.parent_path(); // Get the directory containing the file

    // Replace "instances" with "results"
    std::string results_path = base_path.string();
    size_t pos = results_path.find("instances");
    if (pos != std::string::npos) {
        results_path.replace(pos, 9, "results");  
    }

    // Extract city name and number
    size_t pos_start = filePath.find_last_of('/') + 1;
    size_t num_start = pos_start;
    while (num_start < filePath.size() && !isdigit(filePath[num_start])) {
        num_start++;
    }
    city_name = filePath.substr(pos_start, num_start - pos_start);
    std::string number_str = filePath.substr(num_start);
	nb_stations = std::stoi(number_str);

    // Construct file paths in "results/"
    re_file_name = results_path + "/re_" + city_name + number_str;
    output_file_name = results_path + "/out_" + city_name + number_str;
	
	std::cout << "In parameters parsing ..." << std::endl;
	std::cout << "City name: " << Parameters::GetCityName() << std::endl;
	std::cout << "NbStations: " << nb_stations << std::endl; 
    std::cout << "Re file name: " << re_file_name << std::endl;
    std::cout << "Out file name: " << output_file_name << std::endl;
}*/


void Parameters::Read(int arg, char ** argv)
{
	printf("Reading parameters\n");
	for(int i=0;i<arg;i++)
	{
		char * first = strtok (argv[i]," ;=");
		char * second = strtok (NULL, " ;=");
		printf ("Parameter:%s value:%s\n",first,second);
		if(second == NULL) continue;
		
		if(strcmp(first, "instance_file") == 0)
		{
			instance_file = std::string(second);
			SetCityName(instance_file.c_str());
		}
		else if(strcmp(first,"construction_heuristic")==0)
		{
			construction_heuristic = std::string(second);
		}
		else if(strcmp(first,"initial_solution_file")==0)
		{
			initial_solution_file = std::string(second);
		}
	}
}