#pragma once
#include "Conflict.h"


class GCBSNode
{
public:
    list<Constraint> constraints; // new constraints
    int sum_of_costs = 0;
    int colliding_pairs = 0;
    size_t depth = 0; // depth of this CT node
    size_t makespan = 0; // makespan over all paths

    uint64_t time_expanded = 0;
    uint64_t time_generated = 0;

    list<shared_ptr<Conflict > > conflicts; // conflicts with the planned paths.
    // if two agents have multiple conflicts, only consider the earliest conflict

    // The chosen conflict
    shared_ptr<Conflict> conflict;


    GCBSNode* parent;
    list< pair< int, Path > > paths; // new paths <agent id, path>

    // the following is used to comapre nodes in the FOCAL list
    struct compare_node_by_d
    {
        bool operator()(const GCBSNode* n1, const GCBSNode* n2) const
        {
            if (n1->colliding_pairs == n2->colliding_pairs)
            {
                if (n1->sum_of_costs == n2->sum_of_costs)
                {
                    return rand() % 2;
                }
                return n1->sum_of_costs >= n2->sum_of_costs;
            }
            return n1->colliding_pairs >= n2->colliding_pairs;
        }
    };  // used by FOCAL to compare nodes by distance_to_go (top of the heap has min distance_to_go)
    pairing_heap< GCBSNode*, compare<GCBSNode::compare_node_by_d> >::handle_type focal_handle;

    inline int getNumNewPaths() const { return (int) paths.size(); }
    inline string getName() const { return "GCBS Node"; }
    list<int> getReplannedAgents() const
    {
        list<int> rst;
        for (const auto& path : paths)
            rst.push_back(path.first);
        return rst;
    }
    void clear()
    {
        conflicts.clear();
    }
};


//std::ostream& operator<<(std::ostream& os, const GCBSNode& node)
//{
//    os << "Node " << node.time_generated << " ( f = "<< node.sum_of_costs <<
//        ", d = " << node.colliding_pairs << " ) with " << node.getNumNewPaths() << " new paths ";
//    return os;
//}