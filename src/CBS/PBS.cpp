#include "PBS.h"

bool PBS::solve(double _time_limit, int _node_limit, int _collsion_threshold)
{
    // set timer
    start = clock();
    this->time_limit = _time_limit;
    this->node_limit = _node_limit;
    this->collsion_threshold = _collsion_threshold;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
        if (screen > 1)
            cout << endl;
    }

    if(!generateRoot())
        return false;

    while (!terminate())
    {
        // Select the node
        auto curr = open_list.top();
        open_list.pop();
        if (curr->conflicts.empty())
            continue;
        // Expand the node
        curr->time_expanded = num_HL_expanded;
        num_HL_expanded++;
        curr->chosen_conflict = curr->conflicts.front();
        if (screen > 1)
        {
            cout << "Expand Node " << curr->time_generated << " with an ";
            if (get<2>(curr->chosen_conflict))
                cout << "internal ";
            else
                cout << "external ";
            cout << "conflict between " << get<0>(curr->chosen_conflict) << " and " <<
                    get<1>(curr->chosen_conflict) << endl;
        }
        generateChild(*curr, false);
        generateChild(*curr, true);
    }
    update(*best_node);
    return true;
}

PBS::PBS(vector<SingleAgentSolver*>& search_engines, PathTableWC & path_table, int screen):
         search_engines(search_engines), path_table(path_table), screen(screen), num_of_agents(search_engines.size()) {}
PBS::~PBS()
{
    for (auto node : all_nodes)
        delete node;
    all_nodes.clear();
    open_list.clear();
}

bool PBS::generateRoot()
{
    if (screen >= 2)
        cout << "Generate Root node " << endl;

    root_node = new PBSNode();
    root_node->priority = make_pair(-1, -1);
    update(*root_node);
    set<int> higher_agents, lower_agents;

    // Plan paths
    if (initial_paths.empty())
    {
        for (auto i = 0; i < num_of_agents; i++)
        {
            if(!planPath(i, *root_node, higher_agents, lower_agents))
            {
                delete root_node;
                return false;
            }
        }
    }
    else
    {
        for (auto i = 0; i < num_of_agents; i++)
        {
            assert(initial_paths[i] != nullptr and
                    initial_paths[i]->front().location == search_engines[i]->start_location and
                    initial_paths[i]->back().location == search_engines[i]->goal_location);
            root_node->new_paths.emplace_back(i, *initial_paths[i]);
            paths[i] = &root_node->new_paths.back().second;
            root_node->sum_of_costs += (int)initial_paths[i]->size() - 1;
        }
    }

    // Find conflicts
    for (auto i = 0; i < num_of_agents; i++)
    {
        for (auto j = i + 1; j < num_of_agents; j++)
        {
            findInternalConflicts(*root_node, i, j);
        }
        findExternalConflicts(*root_node, i, lower_agents);
    }
    best_node = root_node;
    pushNode(root_node);
    return true;
}
bool PBS::generateChild(PBSNode& parent, bool left_child)
{
    if (screen > 1)
        cout << "Generate Node " << num_HL_generated + 1 << " with constraint ";
    auto child = new PBSNode();
    child->parent = &parent;
    child->conflicts = parent.conflicts;
    child->abandoned_conflicts = parent.abandoned_conflicts;
    child->sum_of_costs = parent.sum_of_costs;

    // Add constraint
    if (get<2>(parent.chosen_conflict)) // internal conflict
    {
        if (left_child)
        {
            child->priority = make_pair(get<0>(parent.chosen_conflict), get<1>(parent.chosen_conflict));
            if (screen > 1)
                cout << get<0>(parent.chosen_conflict) << "<" << get<1>(parent.chosen_conflict) << endl;
        }
        else
        {
            child->priority = make_pair(get<1>(parent.chosen_conflict), get<0>(parent.chosen_conflict));
            if (screen > 1)
                cout << get<1>(parent.chosen_conflict) << "<" << get<0>(parent.chosen_conflict) << endl;
        }

    }
    else // external conflict
    {
        if (left_child)
        {
            child->priority = make_pair(get<0>(parent.chosen_conflict), -1 - get<1>(parent.chosen_conflict));
            if (screen > 1)
                cout << get<0>(parent.chosen_conflict) << "<External " << get<1>(parent.chosen_conflict) << endl;
        }
        else
        {
            child->priority = make_pair(- 1 - get<1>(parent.chosen_conflict), get<0>(parent.chosen_conflict));
            assert(child->conflicts.front() == parent.chosen_conflict);
            child->conflicts.erase(child->conflicts.begin());
            child->abandoned_conflicts.push_back(parent.chosen_conflict);
            if (screen > 1)
                cout << " abandoned conflict" << endl;
            pushNode(child);
            return true;
        }
    }

    update(*child);

    list<int> ordered_agents;
    topologicalSort(ordered_agents);

    set<int> to_replan;
    to_replan.insert(child->priority.first);
    for(auto pt = std::find(ordered_agents.begin(), ordered_agents.end(), child->priority.first);
        !to_replan.empty(); ++pt)
    {
        int rst = to_replan.erase(*pt);
        if (rst == 0) // pt is not in to_replan
            continue;

        auto a1 = *pt;
        set<int> higher_agents;
        getHigherPriorityAgents(a1, higher_agents, ordered_agents);
        set<int> lower_agents;
        getLowerPriorityAgents(a1, lower_agents, ordered_agents);

        // Re-plan path
        if(!planPath(a1, *child, higher_agents, lower_agents))
        {
            delete child;
            return false;
        }

        // Update conflicts
        deleteConflicts(child->conflicts, a1); // Delete old conflicts
        deleteConflicts(child->abandoned_conflicts, a1); // Delete old conflicts

        for (auto a2 : ordered_agents) // Find new conflicts
        {
            if (a1 == a2)
                continue;
            if (higher_agents.count(a2) > 0) // the path will not collide with the agents with higher priority
            {
                assert(!findInternalConflicts(*child, a1, a2));
                continue;
            }
            if (findInternalConflicts(*child, a1, a2))
            {
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                    to_replan.insert(a2);
            }
        }
        findExternalConflicts(*child, a1, lower_agents);
    }

    pushNode(child);
    return true;
}

bool PBS::planPath(int agent, PBSNode& node, const set<int> & higher_agents, const set<int> & lower_agents)
{
    if (screen > 1)
        cout << "\t\tReplan path for agent " << agent << " by avoiding collisions with agents ";
    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(search_engines[agent]->instance.num_of_cols,
                                     search_engines[agent]->instance.map_size,nullptr, &path_table);
    for ( auto a : higher_agents)
    {
        if (a < 0) // external agent
        {
            constraint_table.insert2CT(*path_table.getPath(-a-1));
            if (screen > 1)
                cout << "External " <<  - a - 1 << "," << *path_table.getPath(-a-1) << endl;
        }
        else // internal agent
        {
            assert(paths[a] != nullptr and a != agent);
            constraint_table.insert2CT(*paths[a]);
            if (screen > 1)
                cout << a << "," << *paths[a] << endl;
        }

    }
    if (screen > 1)
        cout << endl;
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // build CAT
    t = clock();
    for (int a = 0; a < num_of_agents; a++)
    {
        if (paths[a] != nullptr and a != agent and higher_agents.count(a) == 0)
        {
            constraint_table.insert2CAT(*paths[a]);
        }
    }
    for (auto a : lower_agents)
    {
        if (a < 0)
            path_table.deletePath(-a-1);
    }
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // find a path
    t = clock();
    Path new_path = search_engines[agent]->findPath(constraint_table);
    for (auto a : lower_agents)
    {
        if (a < 0)
            path_table.insertPath(-a-1);
    }
    runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
    if (screen > 1)
        cout << "\t\t\tRuntime of single-agent search = " << (double)(clock() - t) / CLOCKS_PER_SEC <<
             "s with " << search_engines[agent]->getNumExpanded() << " expanded nodes" << endl;
    if (new_path.empty())
    {
        if (screen > 1)
            cout << "\t\t\t\tFail to find a path" << endl;
        return false;
    }
    assert(paths[agent] == nullptr or !isSamePath(*paths[agent], new_path));
    node.new_paths.emplace_back(agent, new_path);
    if (paths[agent] == nullptr)
        node.sum_of_costs += (int)new_path.size() - 1;
    else
        node.sum_of_costs += - (int)paths[agent]->size() + (int)new_path.size();
    paths[agent] = &node.new_paths.back().second;
    // node.makespan = max(node.makespan, (int) new_path.size() - 1);
    return true;
}

void PBS::topologicalSort(list<int>& stack)
{
    stack.clear();
    vector<bool> visited(num_of_agents, false);

    // Call the recursive helper function to store Topological
    // Sort starting from all vertices one by one
    for (int i = 0; i < num_of_agents; i++)
    {
        if (!visited[i])
            topologicalSortUtil(i, visited, stack);
    }
    stack.reverse();
}
void PBS::topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack)
{
    // Mark the current node as visited.
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    if (!priorities.empty())
    {
        for (int i = 0; i < (int) priorities[v].size(); i++)
        {
            if (priorities[v][i] and !visited[i])
                topologicalSortUtil(i, visited, stack);
        }
    }
    // Push current vertex to stack which stores result
    stack.push_front(v);
}

void PBS::getHigherPriorityAgents(int agent, set<int>& higher_agents, const list<int> & ordered_agents) const
{
    if (agent == ordered_agents.front())
    {
        higher_agents.insert(higher_external_agents[agent].begin(), higher_external_agents[agent].end());
        return;
    }

    auto p = std::find(ordered_agents.begin(), ordered_agents.end(), agent);
    getHigherPriorityAgents(p, higher_agents, ordered_agents);
    higher_agents.insert(higher_external_agents[agent].begin(), higher_external_agents[agent].end());
}
void PBS::getHigherPriorityAgents(const list<int>::const_iterator & p1, set<int> & higher_agents,
        const list<int> & ordered_agents) const
{
    assert(p1 != ordered_agents.begin());
    for (auto p2 = std::prev(p1); p2 != ordered_agents.begin(); --p2)
    {
        if (priorities[*p1][*p2])
        {
            auto ret = higher_agents.insert(*p2);
            if (ret.second) // insert successfully
            {
                getHigherPriorityAgents(*p2, higher_agents, ordered_agents);
            }
        }
    }
    if (priorities[*p1][ordered_agents.front()])
    {
        higher_agents.insert(ordered_agents.front());
    }
}
void PBS::getLowerPriorityAgents(int agent, set<int>& lower_agents, const list<int> & ordered_agents) const
{
    auto p = std::find(ordered_agents.begin(), ordered_agents.end(), agent);
    getLowerPriorityAgents(p, lower_agents, ordered_agents);
    lower_agents.insert(lower_external_agents[agent].begin(), lower_external_agents[agent].end());
}
void PBS::getLowerPriorityAgents(const list<int>::const_iterator & p1, set<int> & lower_agents,
        const list<int> & ordered_agents) const
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.end(); ++p2)
    {
        if (priorities[*p2][*p1])
        {
            auto ret = lower_agents.insert(*p2);
            if (ret.second) // insert successfully
            {
                getLowerPriorityAgents(p2, lower_agents, ordered_agents);
            }
        }
    }
}

bool PBS::findInternalConflicts(PBSNode& node, int a1, int a2) const
{
    assert(paths[a1] != nullptr and paths[a2] != nullptr);
    int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size()) - 1;
    if (paths[a1]->size() != paths[a2]->size())
    {
        int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
        int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
        int loc1 = paths[a1_]->back().location;
        for (int timestep = min_path_length; timestep < (int)paths[a2_]->size(); timestep++)
        {
            if (loc1 == paths[a2_]->at(timestep).location)
            {
                node.conflicts.emplace_front(a1_, a2_, true);
                return true;
            }
        }
    }
    for (int timestep = 0; timestep < min_path_length; timestep++)
    {
        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;
        if (loc1 == loc2  or // vertex
            (loc1 == paths[a2]->at(timestep + 1).location and loc2 == paths[a1]->at(timestep + 1).location)) // edge
        {
            node.conflicts.emplace_back(a1, a2, true);
            return true;
        }
    }
    return false;
}
bool PBS::findExternalConflicts(PBSNode& node, int a, const set<int> & lower_agents) const
{
    if (path_table.table.empty())
        return false;
    assert(paths[a] != nullptr and !paths[a]->empty());
    set<int> collisions;
    assert(path_table.table[paths[a]->front().location].empty() or
        path_table.table[paths[a]->front().location][0].empty());
    for (int timestep = 1; timestep < (int) paths[a]->size(); timestep++)
    {
        auto from = paths[a]->at(timestep - 1).location;
        auto to = paths[a]->at(timestep).location;
        if ((int)path_table.table[to].size() > timestep) // vertex conflict
        {
            collisions.insert(path_table.table[to][timestep].begin(), path_table.table[to][timestep].end());
        }
        if (from != to && path_table.table[to].size() >= timestep && path_table.table[from].size() > timestep)  // edge conflict
        {
            for (auto a1 : path_table.table[to][timestep - 1])
            {
                for (auto a2: path_table.table[from][timestep])
                {
                    if (a1 == a2)
                        collisions.insert(a1);
                }
            }
        }
        auto a2 = path_table.getAgentWithTarget(to, timestep);
        if (a2 >= 0) // target conflict - this agent traverses the target of another agent
            collisions.insert(a2);
    }
    auto goal = paths[a]->back().location; // target conflicts - some other agent traverses the target of this agent
    for (int t = (int)paths[a]->size(); t < path_table.table[goal].size(); t++)
    {
        collisions.insert(path_table.table[goal][t].begin(), path_table.table[goal][t].end());
    }

    for (auto c : collisions)
    {
        if (lower_agents.count(-c-1) == 0)
        {
            node.conflicts.emplace_back(a, c, false);
        }
        else
        {
            node.abandoned_conflicts.emplace_back(a, c, false);
        }
    }
    return !collisions.empty();
}

void PBS::update(PBSNode& node)
{
    priorities.assign(num_of_agents, vector<bool>(num_of_agents, false));
    higher_external_agents.assign(num_of_agents, list<int>());
    lower_external_agents.assign(num_of_agents, list<int>());
    paths.assign(num_of_agents, nullptr);
    for (auto curr = &node; curr != nullptr; curr = curr->parent)
    {
        if (curr->priority.first >= 0)
        {
            if (curr->priority.second >= 0) // internal priority constraint
                priorities[curr->priority.first][curr->priority.second] = true;
            else // external priority constraint
                higher_external_agents[curr->priority.first].push_back(curr->priority.second);
        }
        else
        {
            if (curr->priority.second >= 0) // external priority constraint
                lower_external_agents[curr->priority.second].push_back(curr->priority.first);
        }
        for (auto & path_pair : curr->new_paths)
        {
            if (paths[path_pair.first] == nullptr)
            {
                paths[path_pair.first] = &path_pair.second;
            }
        }
    }
}

inline bool PBS::terminate()
{
    runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    return open_list.empty() or runtime > time_limit or num_HL_expanded > node_limit or
            open_list.top()->getCollidingPairs() < collsion_threshold;
}

inline void PBS::pushNode(PBSNode* node)
{
    num_HL_generated++;
    node->time_generated = num_HL_generated;
    all_nodes.push_back(node);
    node->open_handle = open_list.push(node);
    if (best_node == nullptr or node->getCollidingPairs() < best_node->getCollidingPairs())
        best_node = node;
}

void PBS::deleteConflicts(list<CollidingPair>& conflicts, int a) const
{
    for (auto c = conflicts.begin();c != conflicts.end();) // Delete old conflicts
    {
        if (get<0>(*c) == a or (get<1>(*c) == a and get<2>(*c)))
        {
            c = conflicts.erase(c);
        }
        else
            ++c;
    }
}
