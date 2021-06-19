#pragma once
#include "BasicLNS.h"

enum init_destroy_heuristic { TARGET_BASED, COLLISION_BASED, RANDOM_BASED, INIT_COUNT };

class InitLNS : public BasicLNS
{
public:
    vector<Agent>& agents;
    int num_of_colliding_pairs = 0;

    InitLNS(const Instance& instance, vector<Agent>& agents, double time_limit,
            const string & replan_algo_name, const string & init_destory_name, int neighbor_size, int screen);

    bool getInitialSolution();
    bool run();
    void writeIterStatsToFile(const string & file_name) const;
    void writeResultToFile(const string & file_name, int sum_of_distances, double preprocessing_time) const;
    string getSolverName() const override { return "InitLNS(" + replan_algo_name + ")"; }

    void printPath() const;
    void printResult();
    void clear(); // delete useless data to save memory

private:
    string replan_algo_name;
    init_destroy_heuristic init_destroy_strategy = COLLISION_BASED;

    PathTableWC path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.

    vector<set<int>> collision_graph;
    vector<int> goal_table;


    bool runPP();
    bool runGCBS();
    bool runPBS();

    bool updateCollidingPairs(set<pair<int, int>>& colliding_pairs, int agent_id, const Path& path) const;

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByCollisionGraph();
    bool generateNeighborByTarget();
    bool generateNeighborRandomly();

    // int findRandomAgent() const;
    int randomWalk(int agent_id);

    void printCollisionGraph() const;

    static unordered_map<int, set<int>>& findConnectedComponent(const vector<set<int>>& graph, int vertex,
            unordered_map<int, set<int>>& sub_graph);

    bool validatePathTable() const;
};
