#include "ReservationTable.h"


/*int ResevationTable::get_holding_time(int location)
{ 
	auto it = constraints.find(location);
	if (it != constraints.end())
	{
		for (auto constraint : it->second)
			insert_constraint(location, constraint.first, constraint.second);
	}
	
	if (RT.find(location) == RT.end()) 
	{
		return 0;
	}
	int t = std::get<1>(RT[location].back());
	if (t < INTERVAL_MAX)
		return INTERVAL_MAX;
	for (auto p =  RT[location].rbegin(); p != RT[location].rend(); ++p)
	{
		if (t == std::get<1>(*p))
			t = std::get<0>(*p);
		else
			break;
	}
	return t;
}*/

ReservationTable::ReservationTable(const ConstraintTable& other) :
    ConstraintTable(other.path_table, other.num_col, other.map_size)
{
    copy(other);
}

void ReservationTable::insert2SIT(size_t location, size_t t_min, size_t t_max)
{
	assert(t_min >= 0 && t_min < t_max);
    auto pi = sit.find(location);
    if (pi == sit.end())
    {
		assert(length_min <= length_max);
		int latest_timestep = min(length_max, MAX_TIMESTEP - 1) + 1;
		if (t_min > 0)
		{
			sit[location].emplace_back(0, t_min, false);
		}
		if ((int)t_max < latest_timestep)
		{
			sit[location].emplace_back(t_max, latest_timestep, false);
		}
        return;
    }
    auto& intervals = pi->second;
    for (auto it = intervals.begin(); it != intervals.end();)
    {
        if (t_min >= get<1>(*it))
			++it; 
        else if (t_max <= get<0>(*it))
            break;
        else if (get<0>(*it) < t_min && get<1>(*it) <= t_max)
        {
            (*it) = make_tuple(get<0>(*it), t_min, get<2>(*it));
			++it;
        }
        else if (t_min <= get<0>(*it) && t_max < get<1>(*it))
        {
            (*it) = make_tuple(t_max, get<1>(*it), get<2>(*it));
            break;
        }
        else if (get<0>(*it) < t_min && t_max < get<1>(*it))
        {
            intervals.insert(it, make_tuple(get<0>(*it), t_min, get<2>(*it)));
            (*it) = make_tuple(t_max, get<1>(*it), get<2>(*it));
            break;
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            it = sit[location].erase(it);
        }
    }
}

void ReservationTable::insertSoftConstraint2SIT(size_t location, size_t t_min, size_t t_max)
{
    assert(t_min >= 0 && t_min < t_max);
    auto pi = sit.find(location);
    if (pi == sit.end())
    {
        if (t_min > 0)
        {
			sit[location].emplace_back(0, t_min, false);
        }
		sit[location].emplace_back(t_min, t_max, true);
		sit[location].emplace_back(t_max, min(length_max + 1, MAX_TIMESTEP), false);
        return;
    }
    auto& intervals = pi->second;
    for (auto it = intervals.begin(); it != intervals.end(); ++it)
    {
        if (t_min >= get<1>(*it) || get<2>(*it))
            continue;
        else if (t_max <= get<0>(*it))
            break;

        auto i_min = get<0>(*it);
        auto i_max = get<1>(*it);
        if (i_min < t_min && i_max <= t_max)
        {
            if (it != intervals.end() and std::next(it) != intervals.end() and
                    (location != goal_location || i_max != length_min) and
                    i_max == get<0>(*std::next(it)) and get<2>(*std::next(it))) // we can merge the current interval with the next one
            {
                (*it) = make_tuple(i_min, t_min, false);
                ++it;
                (*it) = make_tuple(t_min, get<1>(*it), true);
            }
            else
            {
                intervals.insert(it, make_tuple(i_min, t_min, false));
                (*it) = make_tuple(t_min, i_max, true);
            }

        }
        else if (t_min <= i_min && t_max < i_max)
        {
            if (it != intervals.begin() and (location != goal_location || i_min != length_min) and
                    i_min == get<1>(*std::prev(it)) and get<2>(*std::prev(it))) // we can merge the current interval with the previous one
            {
                (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), t_max, true);
            }
            else
            {
                intervals.insert(it, make_tuple(i_min, t_max, true));
            }
            (*it) = make_tuple(t_max, i_max, false);
        }
        else if (i_min < t_min && t_max < i_max)
        {
			intervals.insert(it, make_tuple(i_min, t_min, false));
			intervals.insert(it, make_tuple(t_min, t_max, true));
            (*it) = make_tuple(t_max, i_max, false);
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            if (it != intervals.begin() and (location != goal_location || i_min != length_min) and
                i_min == get<1>(*std::prev(it)) and get<2>(*std::prev(it))) // we can merge the current interval with the previous one
            {
                if (it != intervals.end() and std::next(it) != intervals.end() and
                        (location != goal_location || i_max != length_min) and
                        i_max == get<0>(*std::next(it)) and get<2>(*std::next(it))) // we can merge the current interval with the next one
                {
                    (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), get<1>(*std::next(it)), true);
                    intervals.erase(std::next(it));
                    it = intervals.erase(it);
                }
                else
                {
                    (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), i_max, true);
                    it = intervals.erase(it);
                }
                --it;
            }
            else
            {
                if (it != intervals.end() and std::next(it) != intervals.end() and
                        (location != goal_location || i_max != length_min) and
                        i_max == get<0>(*std::next(it)) and get<2>(*std::next(it))) // we can merge the current interval with the next one
                {
                    (*it) = make_tuple(i_min, get<1>(*std::next(it)), true);
                    intervals.erase(std::next(it));
                }
                else
                {
                    (*it) = make_tuple(i_min, i_max, true);
                }
            }
        }
    }
}


//merge successive safe intervals with the same number of conflicts.
/*void ReservationTable::mergeIntervals(list<Interval >& intervals) const
{
	if (intervals.empty())
		return;
	auto prev = intervals.begin();
	auto curr = prev;
	++curr;
	while (curr != intervals.end())
	{
		if (get<1>(*prev) == get<0>(*curr) && get<2>(*prev) == get<2>(*curr))
		{
			*prev = make_tuple(get<0>(*prev), get<1>(*curr), get<2>(*prev));
			curr = intervals.erase(curr);
		}
		else
		{
			prev = curr;
			++curr;
		}
	}
}*/ // we cannot merge intervals for goal locations seperated by length_min


// update SIT at the given location
void ReservationTable::updateSIT(size_t location)
{
	if (sit.find(location) == sit.end())
	{
		// length constraints for the goal location
		if (location == goal_location) // we need to divide the same intervals into 2 parts [0, length_min) and [length_min, length_max + 1)
		{
			if (length_min > length_max) // the location is blocked for the entire time horizon
			{
				sit[location].emplace_back(0, 0, false);
				return;
			}
			if (0 < length_min)
			{
				sit[location].emplace_back(0, length_min, false);
			}
			assert(length_min >= 0);
			sit[location].emplace_back(length_min, min(length_max + 1, MAX_TIMESTEP), false);
		}


		// path table
		if (!path_table.table.empty())
        {
		    if (location < map_size) // vertex conflict
            {
                for (int t = 0; t < (int)path_table.table[location].size(); t++)
                {
                    if (path_table.table[location][t] != NO_AGENT)
                    {
                        insert2SIT(location, t, t+1);
                    }
                }
                if (path_table.goals[location] < MAX_TIMESTEP) // target conflict
                    insert2SIT(location, path_table.goals[location], MAX_TIMESTEP + 1);
            }
		    else // edge conflict
            {
		        auto from = location / map_size - 1;
		        auto to = location % map_size;
		        if (from != to)
                {
		            int t_max = (int) min(path_table.table[from].size(), path_table.table[to].size() + 1);
                    for (int t = 1; t < t_max; t++)
                    {
                        if (path_table.table[to][t - 1] != NO_AGENT and
                                path_table.table[to][t - 1] == path_table.table[from][t])
                        {
                            insert2SIT(location, t, t+1);
                        }
                    }
                }
            }
        }

		// negative constraints
		const auto& it = ct.find(location); 
		if (it != ct.end())
		{
			for (auto time_range : it->second)
                insert2SIT(location, time_range.first, time_range.second);
		}

		// positive constraints
		if (location < map_size)
		{
			for (auto landmark : landmarks)
			{
				if (landmark.second != location)
				{
                    insert2SIT(location, landmark.first, landmark.first + 1);
				}
			}
		}

		// soft constraints
		const auto& it2 = cat.find(location);
		if (it2 != cat.end())
		{
			for (auto time_range : it2->second)
                insertSoftConstraint2SIT(location, time_range.first, time_range.second);
		}
	}
}
void ReservationTable::updateSIT(size_t location, const PathTableWC& soft_path_table)
{
    if (sit.find(location) == sit.end())
    {
        // length constraints for the goal location
        if (location == goal_location) // we need to divide the same intervals into 2 parts [0, length_min) and [length_min, length_max + 1)
        {
            if (length_min > length_max) // the location is blocked for the entire time horizon
            {
                sit[location].emplace_back(0, 0, false);
                return;
            }
            if (0 < length_min)
            {
                sit[location].emplace_back(0, length_min, false);
            }
            assert(length_min >= 0);
            sit[location].emplace_back(length_min, min(length_max + 1, MAX_TIMESTEP), false);
        }

        // path table
        if (!path_table.table.empty())
        {
            if (location < map_size) // vertex conflict
            {
                for (int t = 0; t < (int)path_table.table[location].size(); t++)
                {
                    if (path_table.table[location][t] != NO_AGENT)
                    {
                        insert2SIT(location, t, t+1);
                    }
                }
                if (path_table.goals[location] < MAX_TIMESTEP) // target conflict
                    insert2SIT(location, path_table.goals[location], MAX_TIMESTEP + 1);
            }
            else // edge conflict
            {
                auto from = location / map_size - 1;
                auto to = location % map_size;
                if (from != to)
                {
                    int t_max = (int) min(path_table.table[from].size(), path_table.table[to].size() + 1);
                    for (int t = 1; t < t_max; t++)
                    {
                        if (path_table.table[to][t - 1] != NO_AGENT and
                            path_table.table[to][t - 1] == path_table.table[from][t])
                        {
                            insert2SIT(location, t, t+1);
                        }
                    }
                }
            }
        }

        // negative constraints
        const auto& it = ct.find(location);
        if (it != ct.end())
        {
            for (auto time_range : it->second)
                insert2SIT(location, time_range.first, time_range.second);
        }

        // positive constraints
        if (location < map_size)
        {
            for (auto landmark : landmarks)
            {
                if (landmark.second != location)
                {
                    insert2SIT(location, landmark.first, landmark.first + 1);
                }
            }
        }

        // soft path table
        if (!soft_path_table.table.empty())
        {
            if (location < map_size) // vertex conflict
            {
                for (int t = 0; t < (int)soft_path_table.table[location].size(); t++)
                {
                    if (!soft_path_table.table[location][t].empty())
                    {
                        insertSoftConstraint2SIT(location, t, t+1);
                    }
                }
                if (soft_path_table.goals[location] < MAX_TIMESTEP) // target conflict
                    insertSoftConstraint2SIT(location, soft_path_table.goals[location], MAX_TIMESTEP + 1);
            }
            else // edge conflict
            {
                auto from = location / map_size - 1;
                auto to = location % map_size;
                if (from != to)
                {
                    int t_max = (int) min(soft_path_table.table[from].size(), soft_path_table.table[to].size() + 1);
                    for (int t = 1; t < t_max; t++)
                    {
                        bool found = false;
                        for (auto a1 : soft_path_table.table[to][t - 1])
                        {
                            for (auto a2: soft_path_table.table[from][t])
                            {
                                if (a1 == a2)
                                {
                                    insertSoftConstraint2SIT(location, t, t+1);
                                    found = true;
                                    break;
                                }
                                if (found)
                                    break;
                            }
                        }
                    }
                }
            }
        }

        // soft constraints
        const auto& it2 = cat.find(location);
        if (it2 != cat.end())
        {
            for (auto time_range : it2->second)
                insertSoftConstraint2SIT(location, time_range.first, time_range.second);
        }
    }
}

// [lower_bound, upper_bound)
list<Interval> ReservationTable::get_safe_intervals(size_t location, size_t lower_bound, size_t upper_bound)
{
    list<Interval> rst;
    if (lower_bound >= upper_bound)
        return rst;

	updateSIT(location);
	
	const auto& it = sit.find(location);

    if (it == sit.end())
    {
		rst.emplace_back(0, min(length_max, MAX_TIMESTEP - 1) + 1, 0);
		return rst;
    }

    for(auto interval : sit[location])
    {
        if (lower_bound >= get<1>(interval))
            continue;
        else if (upper_bound <= get<0>(interval))
            break;
        else
        {
            rst.emplace_back(interval);
        }

    }
    return rst;
}
list<Interval> ReservationTable::get_safe_intervals(size_t location, size_t lower_bound, size_t upper_bound,
                                                    const PathTableWC& soft_path_table)
{
    list<Interval> rst;
    if (lower_bound >= upper_bound)
        return rst;

    updateSIT(location, soft_path_table);

    const auto& it = sit.find(location);

    if (it == sit.end())
    {
        rst.emplace_back(0, min(length_max, MAX_TIMESTEP - 1) + 1, 0);
        return rst;
    }

    for(auto interval : sit[location])
    {
        if (lower_bound >= get<1>(interval))
            continue;
        else if (upper_bound <= get<0>(interval))
            break;
        else
        {
            rst.emplace_back(interval);
        }

    }
    return rst;
}

// [lower_bound, upper_bound)
list<Interval> ReservationTable::get_safe_intervals(size_t from, size_t to, size_t lower_bound, size_t upper_bound)
{
    auto safe_vertex_intervals = get_safe_intervals(to, lower_bound, upper_bound);
    auto safe_edge_intervals = get_safe_intervals(getEdgeIndex(from, to), lower_bound, upper_bound);
    return get_interval_intersection_interval(safe_vertex_intervals, safe_edge_intervals);
}
list<Interval> ReservationTable::get_safe_intervals(size_t from, size_t to, size_t lower_bound, size_t upper_bound,
                                                    const PathTableWC& soft_path_table)
{
    auto safe_vertex_intervals = get_safe_intervals(to, lower_bound, upper_bound, soft_path_table);
    auto safe_edge_intervals = get_safe_intervals(getEdgeIndex(from, to), lower_bound, upper_bound, soft_path_table);
    return get_interval_intersection_interval(safe_vertex_intervals, safe_edge_intervals);
}

Interval ReservationTable::get_first_safe_interval(size_t location)
{
	updateSIT(location);
    const auto& it = sit.find(location);
    if (it == sit.end())
		return Interval(0, min(length_max + 1, MAX_TIMESTEP), 0);
    else
		return it->second.front();
}
Interval ReservationTable::get_first_safe_interval(size_t location, const PathTableWC& soft_path_table)
{
    updateSIT(location, soft_path_table);
    const auto& it = sit.find(location);
    if (it == sit.end())
        return Interval(0, min(length_max + 1, MAX_TIMESTEP), 0);
    else
        return it->second.front();
}

// find a safe interval with t_min as given
bool ReservationTable::find_safe_interval(Interval& interval, size_t location, size_t t_min)
{
	if (t_min >= min(length_max, MAX_TIMESTEP - 1) + 1)
		return false;
	updateSIT(location);
	const auto& it = sit.find(location);
    if (it == sit.end())
    {
		interval = Interval(t_min, min(length_max, MAX_TIMESTEP - 1) + 1, 0);
		return true;
    }
    for( auto i : it->second)
    {
        if ((int)get<0>(i) <= t_min && t_min < (int)get<1>(i))
        {
            interval = Interval(t_min, get<1>(i), get<2>(i));
            return true;
        }
        else if (t_min < (int)get<0>(i))
            break;
    }
    return false;
}
bool ReservationTable::find_safe_interval(Interval& interval, size_t location, size_t t_min,
                                          const PathTableWC& soft_path_table)
{
    if (t_min >= min(length_max + 1, MAX_TIMESTEP))
        return false;
    updateSIT(location, soft_path_table);
    const auto& it = sit.find(location);
    if (it == sit.end())
    {
        interval = Interval(t_min, min(length_max, MAX_TIMESTEP - 1) + 1, 0);
        return true;
    }
    for( auto i : it->second)
    {
        if ((int)get<0>(i) <= t_min && t_min < (int)get<1>(i))
        {
            interval = Interval(t_min, get<1>(i), get<2>(i));
            return true;
        }
        else if (t_min < (int)get<0>(i))
            break;
    }
    return false;
}

list<Interval> ReservationTable::get_interval_intersection_interval(const list<Interval>& interval1,
                                                  const list<Interval>& interval2)
{
    list<Interval> rst;
    auto it1 = interval1.begin();
    auto it2 = interval2.begin();
    while (it1 != interval1.end() && it2 != interval2.end())
    {
        auto t_min = max(get<0>(*it1), get<0>(*it2));
        auto t_max = min(get<1>(*it1), get<1>(*it2));
        if (t_min < t_max)
            rst.emplace_back(t_min, t_max, get<2>(*it1) or get<2>(*it2));
        if (t_max == get<1>(*it1))
            ++it1;
        if (t_max == get<1>(*it2))
            ++it2;
    }
    return rst;
}
void ReservationTable::print() const
{
    for (const auto& entry : sit)
    {
        cout << "loc=" << entry.first << ":";
        for (const auto& interval : entry.second)
        {
            cout << "[" << get<0>(interval) << "," << get<1>(interval) << "],";
        }
    }
    cout << endl;
}