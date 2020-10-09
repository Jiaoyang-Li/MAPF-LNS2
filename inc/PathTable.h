#pragma once
#include "common.h"

#define NO_AGENT -1

class PathTable
{
public:
    int makespan = 0;
    vector< vector<int> > table; // this stores the collision-free paths, the value is the id of the agent
    vector<int> goals; // this stores the goal locatons of the paths: key is the location, while value is the timestep when the agent reaches the goal
    void reset() { auto map_size = table.size(); table.clear(); table.resize(map_size); goals.assign(map_size, MAX_COST); makespan = 0; }
    void insertPath(int agent_id, const Path& path);
    void deletePath(int agent_id, const Path& path);
    bool constrained(int from, int to, int to_time) const;

    void get_agents(set<int>& conflicting_agents, int loc) const;
    void get_agents(set<int>& conflicting_agents, int neighbor_size, int loc) const;
    void getConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to, int to_time) const;


    PathTable(int map_size = 0) : table(map_size), goals(map_size, MAX_COST) {}
};