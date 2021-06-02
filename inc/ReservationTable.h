// This is used by SIPP
#pragma once
#include "ConstraintTable.h"

typedef tuple<size_t, size_t, bool> Interval; // [t_min, t_max), has collision


class ReservationTable
{
public:
    const ConstraintTable& constraint_table;

    ReservationTable(const ConstraintTable& constraint_table, int goal_location) :
        constraint_table(constraint_table), goal_location(goal_location) {}

	list<Interval> get_safe_intervals(size_t from, size_t to, size_t lower_bound, size_t upper_bound);
    Interval get_first_safe_interval(size_t location);
    bool find_safe_interval(Interval& interval, size_t location, size_t t_min);

    void print() const;

private:
    int goal_location;
	// Safe Interval Table (SIT)
	typedef unordered_map<size_t, list<Interval > > SIT;
    SIT sit; // location/edge -> [t_min, t_max), num_of_collisions

    void insert2SIT(size_t location, size_t t_min, size_t t_max);
    void insertSoftConstraint2SIT(size_t location, size_t t_min, size_t t_max);
	// void mergeIntervals(list<Interval >& intervals) const;
	void updateSIT(size_t location); // update SIT at the given location

    list<Interval> get_safe_intervals(size_t location, size_t lower_bound, size_t upper_bound);
    static list<Interval> get_interval_intersection_interval(const list<Interval>& interval1,
            const list<Interval>& interval2);
};