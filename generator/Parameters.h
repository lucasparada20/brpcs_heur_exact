#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <map>
#include <string>
#include <iostream>
#include <stdio.h>

void ProcessKeywordParameters(int argc, char * argv[], std::map<std::string, std::string> & parameters)  
{
	printf("Initializing generator. Parsing parameters ... \n");
	// Loop through the arguments starting from index 1 because index 0 is the executable
    for (int i = 1; i < argc; ++i) {
        std::string argument = argv[i];

        // Check if the argument contains "="
        size_t pos = argument.find('=');
        if (pos != std::string::npos) {
            // Split the argument into parameter name and value
            std::string parameter_name = argument.substr(0, pos);
            std::string parameter_value = argument.substr(pos + 1);

            parameters[parameter_name] = parameter_value;
        }
    }

    for (const auto& param : parameters) {
        std::cout << "Parameter Name: " << param.first << ", Value: " << param.second << std::endl;
    }
}

#endif