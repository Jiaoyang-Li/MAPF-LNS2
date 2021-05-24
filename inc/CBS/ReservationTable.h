// This is used by SIPP
#pragma once
#include "ConstraintTable.h"

typedef tuple<size_t, size_t, bool> Interval; // [t_min, t_max), has collision

//TODO:: ReservationTable does not consider path table
class ReservationTable: public ConstraintTable
{
public:
    double runtime;

	//ReservationTable() = default;
	ReservationTable(const PathTable& path_table, size_t num_col, size_t map_size, int goal_location = -1) :
            ConstraintTable(path_table, num_col, map_size, goal_location) {}
	ReservationTable(const ConstraintTable& other);

    list<Interval> get_safe_intervals(size_t location, size_t lower_bound, size_t upper_bound);
	list<Interval> get_safe_intervals(size_t from, size_t to, size_t lower_bound, size_t upper_bound);

	// int get_holding_time(int location);
    Interval get_first_safe_interval(size_t location);
    bool find_safe_interval(Interval& interval, size_t location, size_t t_min);

    void print() const;

private:
	// Safe Interval Table (SIT)
	typedef unordered_map<size_t, list<Interval > > SIT;
    SIT sit; // location/edge -> [t_min, t_max), num_of_collisions

    void insert2SIT(size_t location, size_t t_min, size_t t_max);
    void insertSoftConstraint2SIT(size_t location, size_t t_min, size_t t_max);
	// void mergeIntervals(list<Interval >& intervals) const;
	void updateSIT(size_t location); // update SIT at the gvien location
};