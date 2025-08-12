#ifndef MAX_ROUTE_DIST
#define MAX_ROUTE_DIST

#include "Station.h"
#include <cmath>

constexpr double EarthRadius = 6371.0; // in kilometers

double CalculateHarvesineDistance(Station & s1, Station & s2) 
{
    double lat1 = s1.lat;
    double lon1 = s1.lon;
    double lat2 = s2.lat;
    double lon2 = s2.lon;

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

void CheckMaxDistance(std::vector<Station> &stations)
{
    // Remove bad stations
    for (int i = 0; i < stations.size(); ++i)
    {
        for (int j = 0; j < stations.size(); ++j)
        {
            if (i == j) continue;

            double dist = CalculateHarvesineDistance(stations[i], stations[j]);
            if (dist > 100.0)
            {
                std::cout << "Removing station " << j << " due to distance " << dist
                          << " from station " << i << "\n";
                stations.erase(stations.begin() + j);

                // Restart because indices have changed
                i = -1;
                break;
            }
        }
    }

    // Recompute distance matrix after cleanup
    int dim = stations.size();
    double **d = new double *[dim];
    for (int i = 0; i < dim; ++i)
    {
        d[i] = new double[dim];
        for (int j = 0; j < dim; ++j)
        {
            d[i][j] = (i == j) ? 0.0 : CalculateHarvesineDistance(stations[i], stations[j]);
        }
    }

    // Verify all distances
    std::cout << "Verifying distance matrix and station count..." << std::endl;
    bool valid = true;
    for (int i = 0; i < dim; ++i)
    {
        for (int j = 0; j < dim; ++j)
        {
            if (i == j) continue;

            if (d[i][j] > 100.0)
            {
                std::cout << "Error: Distance between station " << i << " and station " << j
                          << " is too large: " << d[i][j] << " km\n";
                valid = false;
            }
        }
    }

    std::cout << "Final station count: " << dim << "\n";
    if (valid)
        std::cout << "All pairwise distances verified to be ≤ 100 km.\n";
    else
        std::cout << "Warning: Some distances exceed 100 km after cleanup!\n";

    // Cleanup memory
    for (int i = 0; i < dim; ++i)
        delete[] d[i];
    delete[] d;

    std::cout << "All distances feasible!\n";
}


#endif