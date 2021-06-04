// This is used by SIPP
#pragma once
#include "ConstraintTable.h"

typedef tuple<size_t, size_t, bool> Interval; // [t_min, t_max), has collision


class ReservationTable
{
public:
    const ConstraintTable& constraint_table;

    ReservationTable(const ConstraintTable& constraint_table, int goal_location) :
        constraint_table(constraint_table), goal_location(goal_location), sit(constraint_table.map_size) {}

	list<pair<Interval, int>> get_safe_intervals(size_t from, size_t to, size_t lower_bound, size_t upper_bound);
    Interval get_first_safe_interval(size_t location);
    bool find_safe_interval(Interval& interval, size_t location, size_t t_min);

private:
    int goal_location;
	// Safe Interval Table (SIT)
	typedef vector< list<Interval> > SIT;
    SIT sit; // location -> [t_min, t_max), num_of_collisions
    void insert2SIT(size_t location, size_t t_min, size_t t_max);
    void insertSoftConstraint2SIT(size_t location, size_t t_min, size_t t_max);
	// void mergeIntervals(list<Interval >& intervals) const;
	void updateSIT(size_t location); // update SIT at the given location
    int get_earliest_arrival_time(int from, int to, const Interval& interval,
            size_t lower_bound, size_t upper_bound) const;
    int get_earliest_no_collision_arrival_time(int from, int to, const Interval& interval,
                                  size_t lower_bound, size_t upper_bound) const;
};