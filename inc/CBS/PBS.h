#pragma once
#include "SingleAgentSolver.h"

typedef tuple<int, int, bool> CollidingPair; // <a1, a2, internal conflict or not>

struct PBSNode
{
    int time_expanded = -1;
    int time_generated = -1;
    int sum_of_costs = 0;
    //int makespan = -1;
    //int colliding_pairs = 0;
    PBSNode* parent = nullptr;
    pair<int, int> priority; // the former has lower priority than the latter task
    list<pair<int, Path>> new_paths;
    list<CollidingPair> conflicts;
    CollidingPair chosen_conflict;
    list<CollidingPair> abandoned_conflicts;
    inline int getCollidingPairs() const {return conflicts.size() + abandoned_conflicts.size(); }

    struct compare_node
    {
        bool operator()(const PBSNode* n1, const PBSNode* n2) const
        {
            if (n1->getCollidingPairs() == n2->getCollidingPairs())
            {
                if (n1->conflicts.size() == n2->conflicts.size())
                {
                    if (n1->sum_of_costs == n2->sum_of_costs)
                        return rand() % 2;
                    return n1->sum_of_costs > n2->sum_of_costs;
                }
                return n1->conflicts.size() < n2->conflicts.size();
            }
            return n1->getCollidingPairs() > n2->getCollidingPairs();
        }
    };
    pairing_heap< PBSNode*, compare<PBSNode::compare_node> >::handle_type open_handle;

    inline int getNumNewPaths() const { return (int) new_paths.size(); }
    inline string getName() const { return "PBS Node"; }
    void clear() { conflicts.clear(); }
};


class PBS
{
public:
    // stats
    double runtime = 0;
    double runtime_generate_child = 0; // runtime of generating child nodes
    double runtime_build_CT = 0; // runtime of building constraint table
    double runtime_build_CAT = 0; // runtime of building conflict avoidance table
    double runtime_path_finding = 0; // runtime of finding paths for single agents
    double runtime_detect_conflicts = 0;
    uint64_t num_HL_expanded = 0;
    uint64_t num_HL_generated = 0;
    vector<Path*> paths;
    PBSNode* best_node = nullptr;

    explicit PBS(vector<SingleAgentSolver*>& search_engines,
                 PathTableWC & path_table, int screen);
    ~PBS();
    void setInitialPath(const vector<const Path*>& _paths) { initial_paths = _paths; }
    bool solve(double time_limit, int node_limit, int collsion_threshold); // the algorithm stops if it finds a solution with number of
    // colliding pairs smaller than the collision_threshold
    string getSolverName() const {return "PBS with" + search_engines[0]->getName();}
private:
    vector<SingleAgentSolver*> search_engines;  // used to find (single) agents' paths
    PathTableWC & path_table;
    int screen;
    double time_limit = -1;
    int collsion_threshold = -1;
    int node_limit = -1;
    clock_t start;
    int num_of_agents;
    PBSNode* root_node = nullptr;
    pairing_heap< PBSNode*, compare<PBSNode::compare_node> > open_list;
    list<PBSNode*> all_nodes;

    vector<vector<bool>> priorities; // pairwise priorities among agents
    vector<list<int>> higher_external_agents; // external agents that have higher priorities
    vector<list<int>> lower_external_agents; // external agents that have lower priorities
    vector<const Path*> initial_paths;
    inline void pushNode(PBSNode* node);
    inline bool terminate();
    void topologicalSort(list<int>& ordered_agents);
    void topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack);
    void update(PBSNode& node);
    bool generateRoot();
    bool generateChild(PBSNode& parent_node, bool left_child);
    bool planPath(int agent, PBSNode& node, const set<int> & higher_agents, const set<int> & lower_agents);

    void getHigherPriorityAgents(int agent, set<int>& higher_agents, const list<int> & ordered_agents) const;
    void getHigherPriorityAgents(const list<int>::const_iterator & p1, set<int> & higher_agents,
            const list<int> & ordered_agents) const;
    void getLowerPriorityAgents(int agent, set<int>& lower_agents, const list<int> & ordered_agents) const;
    void getLowerPriorityAgents(const list<int>::const_iterator & p1, set<int> & lower_agents,
            const list<int> & ordered_agents) const;

    bool findInternalConflicts(PBSNode& node, int a1, int a2) const;
    bool findExternalConflicts(PBSNode& node, int a, const set<int> & lower_agents) const;
    void deleteConflicts(list<CollidingPair>& conflicts, int a) const; // delete conflicts that involve agent a
};