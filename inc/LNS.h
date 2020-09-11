#pragma once
#include "ECBS.h"
#include "SpaceTimeAStar.h"
#include <chrono>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

#define DEFAULT_GROUP_SIZE 5

struct Agent
{
    int id;
    SpaceTimeAStar path_planner;
    Path path;

    Agent(const Instance& instance, int id):
        id(id), path_planner(instance, id) {}
};

struct IterationStats
{
    int sum_of_costs;
    double runtime;
};



class LNS
{
public:
    vector<Agent> agents;
    list<IterationStats> iteration_stats; //stats about each iteration


    double preprocessing_time = 0;

    LNS(const Instance& instance, int screen): instance(instance), screen(screen)
    {
        start_time = Time::now();
        int N = instance.getDefaultNumberOfAgents();
        agents.reserve(N);
        for (int i = 0; i < N; i++)
            agents.emplace_back(instance, i);
        preprocessing_time = ((fsec)(Time::now() - start_time)).count();
        if (screen >= 2)
            cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
    }


    bool getInitialSolution(const string& algorithm_name, double _time_limit);

    // bool run(double time_limit);

private:
    const Instance& instance;
    int screen;
    high_resolution_clock::time_point start_time;
    double runtime = 0;

    //data for neighbors
    list<int> neighbor;

    bool runEECBS(double _time_limit);

    /*list<Path> neighbor_paths;
    int neighbor_sum_of_costs = 0;
    int neighbor_sum_of_showup_time = 0;
    int neighbor_makespan = 0;
    int delta_costs = 0;
    int group_size = DEFAULT_GROUP_SIZE; // this is useful only when we use CBS to replan
    int max_group_size = DEFAULT_GROUP_SIZE;

    vector<int> intersections;
    unordered_map<int, list<int>> start_locations;  // <start location, corresponding agents>

    // intput params
    double time_limit = 0;

    int destroy_strategy = 0; // 0: random walk; 1: start; 2: intersection
    int prirority_ordering_strategy = 0; // 0: random; 1: max regret
    int replan_strategy = 0; // 0: CBS; 1: prioritized planning

    bool adaptive_destroy = false;
    bool iterative_destroy = false;
    double decay_factor = 0.01;
    double reaction_factor = 0.1;
    vector<double> destroy_heuristics;

    // Generate initial solutions
    bool runPP();

    void replanByPP();
    bool replanByCBS();

    void generateNeighborByRandomWalk(boost::unordered_set<int>& tabu_list);
    bool generateNeighborByStart();
    bool generateNeighborByIntersection();
    bool generateNeighborByTemporalIntersection();

    void sortNeighborsRandomly();
    void sortNeighborsByRegrets();
    void sortNeighborsByStrategy();

    //tools
    void updateNeighborPaths();
    void updateNeighborPathsCosts();
    void addAgentPath(int agent, const Path& path);
    void deleteNeighborPaths();
    void quickSort(vector<int>& agent_order, int low, int high, bool regret);
    void randomWalk(int agent_id, const PathEntry& start, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);


    // bool hasConflicts(const vector<Path>& paths) const;


    inline bool compareByRegrets(int a1, int a2)
    {
        int r1 = (int)(al.paths_all[a1].size() -
                       al.agents_all[a1].distance_to_goal / al.agents_all[a1].speed);
        int r2 = (int)(al.paths_all[a2].size() -
                       al.agents_all[a2].distance_to_goal / al.agents_all[a2].speed);
        if (r1 == r2)
            return rand() % 2;
        else
            return r1  > r2;
    }*/
};
