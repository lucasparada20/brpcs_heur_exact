#include "Station.h"
#include "RandomNumbers.h"

void RoundToNearestTenth(std::vector<Station>& stations, RandomNumbers & rn)
{
    if (stations.size() <= 1)
        return; // Nothing to remove

    int original_size = stations.size();
    int target_size = (original_size / 10) * 10;
    int num_to_remove = original_size - target_size;

    if (num_to_remove == 0)
        return; // Already at multiple of 10

    std::vector<int> available_indices;
    for (int i = 1; i < stations.size(); ++i)
        available_indices.emplace_back(i);

    std::vector<int> indices_to_remove;
    for (int i = 0; i < num_to_remove; ++i)
    {
        int idx = rn.randInt(0, (int)available_indices.size() - 1);
        indices_to_remove.push_back(available_indices[idx]);
        available_indices.erase(available_indices.begin() + idx); // Remove chosen index
    }

    // Sort selected indices in descending order
    std::sort(indices_to_remove.begin(), indices_to_remove.end(), std::greater<int>());

    int old_count = (int)stations.size();
	for (size_t idx : indices_to_remove)
        stations.erase(stations.begin() + idx);
	printf("RoundToNearestTenth from:%d to %d\n",old_count,(int)stations.size());
}