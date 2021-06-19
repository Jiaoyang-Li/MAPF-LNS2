#include "BasicLNS.h"
BasicLNS::BasicLNS(const Instance& instance, double time_limit, int neighbor_size, int screen) :
        instance(instance), time_limit(time_limit), neighbor_size(neighbor_size), screen(screen) {}

void BasicLNS::rouletteWheel()
{
    double sum = 0;
    for (const auto& h : destroy_weights)
        sum += h;
    if (screen >= 2)
    {
        cout << "destroy weights = ";
        for (const auto& h : destroy_weights)
            cout << h / sum << ",";
    }
    double r = (double) rand() / RAND_MAX;
    double threshold = destroy_weights[0];
    selected_neighbor = 0;
    while (threshold < r * sum)
    {
        selected_neighbor++;
        threshold += destroy_weights[selected_neighbor];
    }
}
