#pragma once
#include "ECBS.h"
#include "SpaceTimeAStar.h"
#include <chrono>
#include <utility>

//pibt related
#include "simplegrid.h"
#include "pibt_agent.h"
#include "problem.h"
#include "mapf.h"
#include "pibt.h"
#include "pps.h"
#include "winpibt.h"
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;
enum destroy_heuristic { RANDOMAGENTS, RANDOMWALK, INTERSECTION, DESTORY_COUNT };

struct Agent
{
    int id;
    SpaceTimeAStar path_planner; // start, goal, and heuristics are stored in the path planner
    Path path;

    Agent(const Instance& instance, int id) : id(id), path_planner(instance, id) {}

    int getNumOfDelays() const { return (int) path.size() - 1 - path_planner.my_heuristic[path_planner.start_location]; }

};


struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;
    vector<Path> old_paths;
};

// TODO: adaptively change the neighbor size, that is,
// increase it if no progress is made for a while
// decrease it if replanning fails to find any solutions for several times

class LNS
{
public:
    vector<Agent> agents;
    list<IterationStats> iteration_stats; //stats about each iteration
    double preprocessing_time = 0;
    double initial_solution_runtime = 0;
    double runtime = 0;
    int initial_sum_of_costs = -1;
    int sum_of_costs = -1;
    int sum_of_costs_lowerbound = -1;
    int sum_of_distances = -1;
    double average_group_size = -1;
    int num_of_failures = 0; // #replanning that fails to find any solutions
    LNS(const Instance& instance, double time_limit,
        string init_algo_name, string replan_algo_name, string destory_name,
        int neighbor_size, int num_of_iterations, int screen, PIBTPPS_option pipp_option);

    bool getInitialSolution();
    bool run();
    void validateSolution() const;
    void writeIterStatsToFile(string file_name) const;
    void writeResultToFile(string file_name) const;
    void writePathsToFile(string file_name) const;
    string getSolverName() const { return "LNS(" + init_algo_name + ";" + replan_algo_name + ")"; }
private:
    int num_neighbor_sizes = 1; //4; // so the neighbor size could be 2, 4, 8, 16

    // input params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    double replan_time_limit; // time limit for replanning
    string init_algo_name;
    string replan_algo_name;
    int screen;
    destroy_heuristic destroy_strategy = RANDOMWALK;
    int neighbor_size;
    int num_of_iterations;

    high_resolution_clock::time_point start_time;


    PathTable path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.

    Neighbor neighbor;

    unordered_set<int> tabu_list; // used by randomwalk strategy
    list<int> intersections;

    // adaptive LNS
    bool ALNS = false;
    double decay_factor = 0.01;
    double reaction_factor = 0.01;
    vector<double> destroy_weights;
    int selected_neighbor;

    bool runEECBS();
    bool runCBS();
    bool runPP();
    bool runPIBT();
    bool runPPS();
    bool runWinPIBT();

    PIBTPPS_option pipp_option;

    MAPF preparePIBTProblem(vector<int> shuffled_agents);
    void updatePIBTResult(const PIBT_Agents& A,vector<int> shuffled_agents);

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByRandomWalk();
    //bool generateNeighborByStart();
    bool generateNeighborByIntersection(bool temporal = true);

    int findMostDelayedAgent();
    int findRandomAgent() const;
    void randomWalk(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);
};
