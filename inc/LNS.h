#pragma once
#include "ECBS.h"
#include "SpaceTimeAStar.h"
#include <chrono>
#include <utility>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;
enum destroy_heuristic { RANDOMWALK, INTERSECTION, DESTORY_COUNT };

struct Agent
{
    int id;
    SpaceTimeAStar path_planner; // start, goal, and heuristics are stored in the path planner
    Path path;

    Agent(const Instance& instance, int id) : id(id), path_planner(instance, id) {}

    int getNumOfDelays() const { return path.size() - 1 - path_planner.my_heuristic[path_planner.start_location]; }

};


struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;
    vector<Path> old_paths;
};


struct IterationStats
{
    int sum_of_costs;
    double runtime;
    int num_of_agents;
    IterationStats(int num_of_agents, int sum_of_costs, double runtime) :
            num_of_agents(num_of_agents), sum_of_costs(sum_of_costs), runtime(runtime) {}
};



class LNS
{
public:
    vector<Agent> agents;
    list<IterationStats> iteration_stats; //stats about each iteration
    double preprocessing_time = 0;
    double runtime = 0;
    int initial_sum_of_costs = -1;
    int sum_of_costs = -1;

    LNS(const Instance& instance, double time_limit, string init_algo_name, string replan_algo_name, string destory_name,
        int screen): instance(instance), time_limit(time_limit), init_algo_name(std::move(init_algo_name)),
                     replan_algo_name(replan_algo_name), screen(screen), path_table(instance.map_size)
    {
        start_time = Time::now();
        if (destory_name == "Adaptive")
            ALNS = true;
        else if (destory_name == "RandomWalk")
            destroy_strategy = RANDOMWALK;
        else if (destory_name == "Intersection")
            destroy_strategy = INTERSECTION;
        else
        {
            cerr << "Destroy heuristic " << destory_name << " does not exists. " << endl;
            exit(-1);
        }

        int N = instance.getDefaultNumberOfAgents();
        agents.reserve(N);
        for (int i = 0; i < N; i++)
            agents.emplace_back(instance, i);
        preprocessing_time = ((fsec)(Time::now() - start_time)).count();
        if (screen >= 2)
            cout << "Pre-processing time = " << preprocessing_time << " seconds." << endl;
    }


    bool getInitialSolution();

    bool run();

private:
    // intput params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    string init_algo_name;
    string replan_algo_name;
    int screen;
    destroy_heuristic destroy_strategy = RANDOMWALK;

    int neighbor_size = 5;
    int num_of_iterations = 10000;

    high_resolution_clock::time_point start_time;


    PathTable path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.

    Neighbor neighbor;

    boost::unordered_set<int> tabu_list; // used by randomwalk strategy


    // adaptive LNS
    bool ALNS = false;
    double decay_factor = 0.01;
    double reaction_factor = 0.1;
    vector<double> destroy_weights;

    bool runEECBS();
    bool runCBS();
    bool runPP();

    void updateDestroyHeuristicbyALNS();

    void generateNeighborByRandomWalk(boost::unordered_set<int>& tabu_list);
    //bool generateNeighborByStart();
    bool generateNeighborByIntersection();
    bool generateNeighborByTemporalIntersection();

    void randomWalk(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);

    /*list<Path> neighbor_paths;
    int neighbor_sum_of_costs = 0;
    int neighbor_sum_of_showup_time = 0;
    int neighbor_makespan = 0;
    int delta_costs = 0;
    int group_size = DEFAULT_GROUP_SIZE; // this is useful only when we use CBS to replan
    int max_group_size = DEFAULT_GROUP_SIZE;

    vector<int> intersections;
    unordered_map<int, list<int>> start_locations;  // <start location, corresponding agents>





    bool adaptive_destroy = false;
    bool iterative_destroy = false;


    // Generate initial solutions
    bool runPP();

    void replanByPP();
    bool replanByCBS();



    void sortNeighborsRandomly();
    void sortNeighborsByRegrets();
    void sortNeighborsByStrategy();

    //tools
    void updateNeighborPaths();
    void updateNeighborPathsCosts();
    void addAgentPath(int agent, const Path& path);
    void deleteNeighborPaths();
    void quickSort(vector<int>& agent_order, int low, int high, bool regret);



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
