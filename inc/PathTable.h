#pragma once
#include "common.h"

#define NO_AGENT -1

class PathTable
{
public:
    int makespan = 0;
    vector< vector<int> > table; // this stores the collision-free paths, the value is the id of the agent

    void reset() { auto map_size = table.size(); table.clear(); table.resize(map_size); makespan = 0; }
    void insertPath(int agent_id, const Path& path);
    void deletePath(int agent_id, const Path& path);
    bool constrained(int from, int to, int to_time) const;

    //void get_agents(set<int>& conflicting_agents, int loc) const;
    //void get_agents(list< pair<int, int> >& agents, int excluded_agent, const pair<int,int>& loc_time_pair) const;
    //void get_agents(set<int>& conflicting_agents, int groupsize, int loc) const;
    void getConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to, int to_time) const;


    PathTable(int map_size) : table(map_size) {}
};