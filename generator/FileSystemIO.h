#ifndef FILE_SYSTEM_IO
#define FILE_SYSTEM_IO

#include <string>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <stdio.h>

struct DateComparator
{
	bool operator()(const std::string & a, const std::string & b)
	{
		size_t pos_a = a.find("2025");
		size_t pos_b = b.find("2025");

		if (pos_a == std::string::npos || pos_b == std::string::npos)
		{
			printf("DateComparator Phil Colling (1989): Not for the year 2025. Exiting ...\n"); exit(1);
		}
			
		int month_a = std::stoi(a.substr(pos_a + 4, 2));
		int month_b = std::stoi(b.substr(pos_b + 4, 2));

		int day_a = std::stoi(a.substr(pos_a + 6, 2));
		int day_b = std::stoi(b.substr(pos_b + 6, 2));

		if (month_a != month_b)
			return month_a < month_b;
		return day_a < day_b;
	}
};


void LoadFileNames(const std::string & city,
                   std::vector<std::string> & station_information_file_names,
                   std::vector<std::string> & station_status_file_names)
{
    station_information_file_names.reserve(90);
    station_status_file_names.reserve(90);

    std::filesystem::path dir_path = "data_brpcs/" + city;

    if (std::filesystem::exists(dir_path) && std::filesystem::is_directory(dir_path))
    {
        const std::string status_str = "status";
        const std::string information_str = "information";

        for (const auto & entry : std::filesystem::directory_iterator(dir_path))
        {
            std::string path_str = entry.path().string();

            if (path_str.find(status_str) != std::string::npos)
                station_status_file_names.push_back(path_str);

            if (path_str.find(information_str) != std::string::npos)
                station_information_file_names.push_back(path_str);
        }
    }
	
	for(size_t i=0;i<station_information_file_names.size();)
	{
		std::string & a = station_information_file_names[i];
		size_t pos_year = a.find("2025");
		int month = std::stoi(a.substr(pos_year + 4, 2));
		int day = std::stoi(a.substr(pos_year+6,2));
		
		if (month < 5 || ( month == 7 && day > 4 )) 
			station_information_file_names.erase(station_information_file_names.begin()+i);
		else 
			i++;
	}
	for(size_t i=0;i<station_status_file_names.size();)
	{
		std::string & a = station_status_file_names[i];
		size_t pos_year = a.find("2025");
		int month = std::stoi(a.substr(pos_year + 4, 2));
		int day = std::stoi(a.substr(pos_year+6,2));
		
		if (month < 5 || ( month == 7 && day > 4 )) 
			station_status_file_names.erase(station_status_file_names.begin()+i);
		else 
			i++;
	}	
	
    std::sort(station_information_file_names.begin(), station_information_file_names.end(), DateComparator());
    std::sort(station_status_file_names.begin(), station_status_file_names.end(), DateComparator());
}

#endif