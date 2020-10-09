#include "PathTable.h"

void PathTable::insertPath(int agent_id, const Path& path)
{
    if (path.empty())
        return;
    for (int t = 0; t < (int)path.size(); t++)
    {
        if (table[path[t].location].size() <= t)
            table[path[t].location].resize(t + 1, NO_AGENT);
        // assert(table[path[t].location][t] == NO_AGENT);
        table[path[t].location][t] = agent_id;
    }
    goals[path.back().location] = (int) path.size() - 1;
    makespan = max(makespan, (int) path.size() - 1);
}

void PathTable::deletePath(int agent_id, const Path& path)
{
    if (path.empty())
        return;
    for (int t = 0; t < (int)path.size(); t++)
    {
        assert(table[path[t].location].size() > t && table[path[t].location][t] == agent_id);
        table[path[t].location][t] = NO_AGENT;
        goals[path.back().location] = MAX_COST;
    }
    if (makespan == (int) path.size() - 1) // re-compute makespan
    {
        makespan = 0;
        for (int time : goals)
        {
            if (time < MAX_COST && time > makespan)
                makespan = time;
        }

    }
}

bool PathTable::constrained(int from, int to, int to_time) const
{
    if (!table.empty())
    {
        if (table[to].size() > to_time && table[to][to_time] != NO_AGENT)
            return true;  // vertex conflict with agent table[to][to_time]
        else if (table[to].size() >= to_time && table[from].size() > to_time && !table[to].empty() &&
                 table[to][to_time - 1] != NO_AGENT && table[from][to_time] == table[to][to_time - 1])
            return true;  // edge conflict with agent table[to][to_time - 1]
    }
    if (!goals.empty())
    {
        if (goals[to] <= to_time)
            return true; // target conflict
    }
    return false;
}

void PathTable::getConflictingAgents(int agent_id, set<int>& conflicting_agents, int from, int to, int to_time) const
{
    if (table.empty())
        return;
    if (table[to].size() > to_time && table[to][to_time] != NO_AGENT)
        conflicting_agents.insert(table[to][to_time]); // vertex conflict
    if (table[to].size() >= to_time && table[from].size() > to_time &&
        table[to][to_time - 1] != NO_AGENT && table[from][to_time] == table[to][to_time - 1])
        conflicting_agents.insert(table[from][to_time]); // edge conflict
    // TODO: collect target conflicts as well.
}

void PathTable::get_agents(set<int>& conflicting_agents, int loc) const
{
    if (loc < 0)
        return;
    for (auto agent : table[loc])
    {
        if (agent >= 0)
            conflicting_agents.insert(agent);
    }
}

void PathTable::get_agents(set<int>& conflicting_agents, int neighbor_size, int loc) const
{
    if (loc < 0 || table[loc].empty())
        return;
    int t_max = (int) table[loc].size() - 1;
    while (table[loc][t_max] == NO_AGENT && t_max > 0)
        t_max--;
    if (t_max == 0)
        return;
    int t0 = rand() % t_max;
    if (table[loc][t0] != NO_AGENT)
        conflicting_agents.insert(table[loc][t0]);
    int delta = 1;
    while (t0 - delta >= 0 || t0 + delta <= t_max)
    {
        if (t0 - delta >= 0 && table[loc][t0 - delta] != NO_AGENT)
        {
            conflicting_agents.insert(table[loc][t0 - delta]);
            if((int) conflicting_agents.size() == neighbor_size)
                return;
        }
        if (t0 + delta <= t_max && table[loc][t0 + delta] != NO_AGENT)
        {
            conflicting_agents.insert(table[loc][t0 + delta]);
            if((int) conflicting_agents.size() == neighbor_size)
                return;
        }
        delta++;
    }
}


