#include "ConstraintTable.h"

int ConstraintTable::getMaxTimestep() const // everything is static after the max timestep
{
    int rst = max(max(max(ct_max_timestep, cat_max_timestep), length_min), path_table.makespan);
    if (length_max < MAX_TIMESTEP)
        rst = max(rst, length_max);
    if (!landmarks.empty())
        rst = max(rst, landmarks.rbegin()->first);
    return rst;
}

void ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max)
{
	insert2CT(getEdgeIndex(from, to), t_min, t_max);
}

void ConstraintTable::insert2CT(size_t loc, int t_min, int t_max)
{
	assert(loc >= 0);
	ct[loc].emplace_back(t_min, t_max);
	if (t_max < MAX_TIMESTEP && t_max > ct_max_timestep)
	{
        ct_max_timestep = t_max;
	}
	else if (t_max == MAX_TIMESTEP && t_min > ct_max_timestep)
	{
        ct_max_timestep = t_min;
	}
}

void ConstraintTable::insertLandmark(size_t loc, int t)
{
	auto it = landmarks.find(t);
	if (it == landmarks.end())
	{
		landmarks[t] = loc;
	}
	else
		assert(it->second == loc);
}

// return the location-time pairs on the barrier in an increasing order of their timesteps
list<pair<int, int> > ConstraintTable::decodeBarrier(int x, int y, int t) const
{
	list<pair<int, int> > rst;
	int x1 = x / num_col, y1 = x % num_col;
	int x2 = y / num_col, y2 = y % num_col;
	if (x1 == x2)
	{
		if (y1 < y2)
			for (int i = min(y2 - y1, t); i>= 0; i--)
			{
				rst.emplace_back(x1 * num_col + y2 - i, t - i);
			}
		else
			for (int i = min(y1 - y2, t); i >= 0; i--)
			{
				rst.emplace_back(x1 * num_col + y2 + i, t - i);
			}
	}
	else // y1== y2
	{
		if (x1 < x2)
			for (int i = min(x2 - x1, t); i>= 0; i--)
			{
				rst.emplace_back((x2 - i) * num_col + y1, t - i);
			}
		else
			for (int i = min(x1 - x2, t); i>= 0; i--)
			{
				rst.emplace_back((x2 + i) * num_col + y1, t - i);
			}
	}
	return rst;
}

bool ConstraintTable::constrained(size_t loc, int t) const
{
	assert(loc >= 0);
	if (loc < map_size)
	{
	    if (path_table.constrained(loc, loc, t))
	        return true;
		const auto& it = landmarks.find(t);
		if (it != landmarks.end() && it->second != loc)
			return true;  // violate the positive vertex constraint
	}	

	const auto& it = ct.find(loc);
	if (it == ct.end())
	{
		return false;
	}
	for (const auto& constraint: it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}
	return false;
}

bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc, int next_t) const
{
    return path_table.constrained(curr_loc, next_loc, next_t) || constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
	length_min = other.length_min;
	length_max = other.length_max;
	goal_location = other.goal_location;
	num_col = other.num_col;
	map_size = other.map_size;
	ct = other.ct;
	ct_max_timestep = other.ct_max_timestep;
	cat = other.cat;
	cat_max_timestep = other.cat_max_timestep;
	landmarks = other.landmarks;
}

// build the constraint table for the given agent at the give node
void ConstraintTable::build(const HLNode& node, int agent)
{
	auto curr = &node;
	while (curr->parent != nullptr)
	{
        add(curr->constraints, agent);
		curr = curr->parent;
	}
}


// add constraints for the given agent
void ConstraintTable::add(const list<Constraint>& constraints, int agent)
{
    if (constraints.empty())
        return;
    int a, x, y, t;
    constraint_type type;
    tie(a, x, y, t, type) = constraints.front();
    switch (type)
    {
        case constraint_type::LEQLENGTH:
            assert(constraints.size() == 1);
            if (agent == a) // this agent has to reach its goal at or before timestep t.
                length_max = min(length_max, t);
            else // other agents cannot stay at x at or after timestep t
                insert2CT(x, t, MAX_TIMESTEP);
            break;
        case constraint_type::GLENGTH:
            assert(constraints.size() == 1);
            if (a == agent) // path of agent_id should be of length at least t + 1
                length_min = max(length_min, t + 1);
            break;
        case constraint_type::POSITIVE_VERTEX:
            assert(constraints.size() == 1);
            if (agent == a) // this agent has to be at x at timestep t
            {
                insertLandmark(x, t);
            }
            else // other agents cannot stay at x at timestep t
            {
                insert2CT(x, t, t + 1);
            }
            break;
        case constraint_type::POSITIVE_EDGE:
            assert(constraints.size() == 1);
            if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
            {
                insertLandmark(x, t - 1);
                insertLandmark(y, t);
            }
            else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
            {
                insert2CT(x, t - 1, t);
                insert2CT(y, t, t + 1);
                insert2CT(y, x, t, t + 1);
            }
            break;
        case constraint_type::VERTEX:
            if (a == agent)
            {
                for (const auto& constraint : constraints)
                {
                    tie(a, x, y, t, type) = constraint;
                    insert2CT(x, t, t + 1);
                }
            }
            break;
        case  constraint_type::EDGE:
            assert(constraints.size() == 1);
            if (a == agent)
                insert2CT(x, y, t, t + 1);
            break;
        case constraint_type::BARRIER:
            if (a == agent)
            {
                for (auto constraint : constraints)
                {
                    tie(a, x, y, t, type) = constraint;
                    auto states = decodeBarrier(x, y, t);
                    for (const auto& state : states)
                    {
                        insert2CT(state.first, state.second, state.second + 1);
                    }

                }
            }
            break;
        case constraint_type::RANGE:
            if (a == agent)
            {
                assert(constraints.size() == 1);
                insert2CT(x, y, t + 1); // the agent cannot stay at x from timestep y to timestep t.
            }
            break;
    }
}

// build the conflict avoidance table
void ConstraintTable::buildCAT(int agent, const vector<Path*>& paths)
{
    for (size_t ag = 0; ag < paths.size(); ag++)
    {
        if (ag == agent || paths[ag] == nullptr)
            continue;
        if (paths[ag]->size() == 1) // its start location is its goal location
        {
            cat[paths[ag]->front().location].emplace(0, MAX_TIMESTEP);
            continue;
        }
        int prev_location = paths[ag]->front().location;
        int prev_timestep = 0;
        for (int timestep = 0; timestep < (int) paths[ag]->size(); timestep++)
        {
            int curr_location = paths[ag]->at(timestep).location;
            if (prev_location != curr_location)
            {
                cat[prev_location].emplace(prev_timestep, timestep); // add vertex conflict
                cat[getEdgeIndex(curr_location, prev_location)].emplace(timestep, timestep + 1); // add edge conflict
                prev_location = curr_location;
                prev_timestep = timestep;
            }
        }
        cat[paths[ag]->back().location].emplace(paths[ag]->size() - 1, MAX_TIMESTEP);
        cat_max_timestep = max(cat_max_timestep, (int)paths[ag]->size() - 1);
    }
}

int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const
{
    int rst = 0;
    list<size_t> ids;
    ids.push_back(next_id); // vertex
    if (curr_id != next_id)
    {
        ids.push_back(getEdgeIndex(curr_id, next_id)); // edge
    }
    for (auto id : ids)
    {
        const auto& v1 = cat.find(id);
        if (v1 != cat.end())
        {
            for (const auto& action: v1->second)
            {
                if (action.first <= next_timestep && next_timestep < action.second)
                    return 1; //rst++;
                else if (action.first > next_timestep)
                    break;
            }
        }
    }
    return rst;
}




// return the earliest timestep that the agent can hold its goal location
int ConstraintTable::getHoldingTime() const
{
    assert(goal_location >= 0 and goal_location < path_table.table.size());
    int rst = length_min;
    if (!path_table.table.empty() &&
        (int) path_table.table[goal_location].size() > length_min)
    {
        rst = (int) path_table.table[goal_location].size();
        while (rst > length_min && path_table.table[goal_location][rst - 1] == NO_AGENT)
            rst--;
    }

	auto it = ct.find(goal_location);
	if (it != ct.end())
	{
		for (auto time_range : it->second)
			rst = max(rst, time_range.second);
	}
	for (auto landmark : landmarks)
	{
		if (landmark.second != goal_location)
			rst = max(rst, (int)landmark.first + 1);
	}
	return rst;
}
