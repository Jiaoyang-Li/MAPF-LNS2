#include "SIPP.h"
#include "SpaceTimeAStar.h"

void SIPP::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
	path.resize(goal->timestep + 1);
	// num_of_conflicts = goal->num_of_conflicts;

	const auto* curr = goal;
	while (curr->parent != nullptr) // non-root node
	{
		const auto* prev = curr->parent;
		int t = prev->timestep + 1;
		while (t < curr->timestep)
		{
			path[t].location = prev->location; // wait at prev location
			t++;
		}
		path[curr->timestep].location = curr->location; // move to curr location
		curr = prev;
	}
	assert(curr->timestep == 0);
	path[0].location = curr->location;
}


// find path by A*
// Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
Path SIPP::findPath(const ConstraintTable& constraint_table)
{
    //Path path = findNoCollisionPath(constraint_table);
    //if (!path.empty())
    //    return path;
    ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
    num_expanded = 0;
    num_generated = 0;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    auto last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN & FOCAL list
    auto h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
    auto start = new SIPPNode(start_location, 0, h, nullptr, 0, interval, 0);
    num_generated++;
    start->in_openlist = true;
    start->focal_handle = focal_list.push(start); // we only use focal list; no open list is used
    allNodes_table.resize(instance.map_size, nullptr);
    allNodes_table[start->location] = insert(allNodes_table[start->location], start);
    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr->parent, path);
            break;
        }
        else if (curr->location == goal_location && // arrive at the goal location
                 !curr->wait_at_goal && // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            auto goal = new SIPPNode(*curr);
            goal->is_goal = true;
            goal->parent = curr;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(allNodes_table[goal->location], goal))
            {
                goal->focal_handle = focal_list.push(goal);
                goal->in_openlist = true;
                num_generated++;
                allNodes_table[goal->location] = insert(allNodes_table[goal->location], goal);
            }
            else
            {
                delete goal;
            }
        }
        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            int next_h_val = my_heuristic[next_location];
            for (auto& interval : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
            {
                if (interval.second + next_h_val > constraint_table.length_max)
                    break;
                if (!get<2>(interval.first))
                    next_h_val = max(next_h_val, curr->getFVal() - interval.second);  // path max
                else
                    next_h_val = max(next_h_val, holding_time - interval.second); // path max
                generateChildToFocal(interval.first, curr, next_location, interval.second, next_h_val);
            }
        }  // end for loop that generates successors
        // wait at the current location
        bool found = reservation_table.find_safe_interval(interval, curr->location, get<1>(curr->interval));
        if (found)
        {
            generateChildToFocal(interval, curr, curr->location, get<0>(interval), curr->h_val);
        }
    }  // end while loop

    //if (path.empty())
    //    printSearchTree();
    releaseNodes();
    return path;
}

Path SIPP::findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
	const vector<Path*>& paths, int agent, int lowerbound)
{
	return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
}

// find path by SIPP
// Returns a shortest path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
pair<Path, int> SIPP::findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
	const vector<Path*>& paths, int agent, int lowerbound, double w)
{
	this->w = w;

	// build constraint table
    auto t = clock();
    ConstraintTable constraint_table(initial_constraints);
    constraint_table.insert2CT(node, agent);
	runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
	int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
	t = clock();
    constraint_table.insert2CAT(agent, paths);
	runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

	// build reservation table
	ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
	num_expanded = 0;
	num_generated = 0;
	Interval interval = reservation_table.get_first_safe_interval(start_location);
	if (get<0>(interval) > 0)
		return {path, 0};

	 // generate start and add it to the OPEN list
	auto start = new SIPPNode(start_location, 0, max(my_heuristic[start_location], holding_time), nullptr, 0, interval, 0);

	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
    allNodes_table.resize(instance.map_size, nullptr);
    allNodes_table[start->location] = insert(allNodes_table[start->location], start);
	min_f_val = max(holding_time, max((int)start->getFVal(), lowerbound));

	while (!open_list.empty()) 
	{
		updateFocalList(); // update FOCAL if min f-val increased
		SIPPNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);
		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal node
        if (curr->location == goal_location && // arrive at the goal location
			!curr->wait_at_goal && // not wait at the goal location
			curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            updatePath(curr, path);
            break;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
		{
			for (auto & interval : reservation_table.get_safe_intervals(
				curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
			{
				generateChild(interval.first, curr, next_location, interval.second, reservation_table);
			}
		}  // end for loop that generates successors
		   
		// wait at the current location
		bool found = reservation_table.find_safe_interval(interval, curr->location, get<1>(curr->interval));
		if (found)
		{
			generateChild(interval, curr, curr->location, get<0>(interval), reservation_table);
		}
	}  // end while loop
	  
	  // no path found
	releaseNodes();
	return {path, min_f_val};
}

void SIPP::updateFocalList()
{
	auto open_head = open_list.top();
	if (open_head->getFVal() > min_f_val)
	{
		int new_min_f_val = (int)open_head->getFVal();
		for (auto n : open_list)
		{
			if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min_f_val)
				n->focal_handle = focal_list.push(n);
		}
		min_f_val = new_min_f_val;
	}
}


inline SIPPNode* SIPP::popNode()
{
	auto node = focal_list.top(); focal_list.pop();
	open_list.erase(node->open_handle);
	node->in_openlist = false;
	num_expanded++;
	return node;
}


inline void SIPP::pushNode(SIPPNode* node)
{
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;
	if (node->getFVal() <= w * min_f_val)
		node->focal_handle = focal_list.push(node);
}


void SIPP::releaseNodes()
{
	open_list.clear();
	focal_list.clear();
	for (auto n: allNodes_table)
        deleteNodes(n);
	allNodes_table.clear();
}

void SIPP::generateChild(const Interval& interval, SIPPNode* curr, int next_location, int next_timestep,
                         const ReservationTable& reservation_table)
{
    // compute cost to next_id via curr node
    int next_g_val = next_timestep;
    int next_h_val = max(my_heuristic[next_location], curr->getFVal() - next_g_val);  // path max
    if (next_g_val + next_h_val > reservation_table.constraint_table.length_max)
        return;
    int next_conflicts = curr->num_of_conflicts + (int)get<2>(interval) * (next_timestep - curr->timestep);

    // generate (maybe temporary) node
    auto next = new SIPPNode(next_location, next_g_val, next_h_val, curr, next_timestep, interval, next_conflicts);
    if (next_location == goal_location && curr->location == goal_location)
        next->wait_at_goal = true;

    if (dominanceCheck(allNodes_table[next->location], next))
    {
        pushNode(next);
        allNodes_table[next->location] = insert(allNodes_table[next->location], next);
        return;
    }
    else
    {
        delete next;
    }
}
void SIPP::generateChildToFocal(const Interval& interval, SIPPNode* curr, int next_location,
        int next_timestep, int next_h_val)
{
    int next_collisions = curr->num_of_conflicts + (int)get<2>(interval);
    // generate (maybe temporary) node
    auto next = new SIPPNode(next_location, next_timestep, next_h_val, curr, next_timestep, interval, next_collisions);
    if (next_location == goal_location && curr->location == goal_location)
        next->wait_at_goal = true;
    // try to retrieve it from the hash table
    if (dominanceCheck(allNodes_table[next->location], next))
    {
        next->focal_handle = focal_list.push(next);
        next->in_openlist = true;
        num_generated++;
        allNodes_table[next->location] = insert(allNodes_table[next->location], next);
    }
    else
    {
        delete next;
    }

}

// TODO:: currently this is implemented in SIPP inefficiently
int SIPP::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
	int length = MAX_TIMESTEP;
	auto root = new SIPPNode(start, 0, compute_heuristic(start, end), nullptr, 0, Interval(0, 1, 0), 0);
	root->open_handle = open_list.push(root);  // add root to heap
    allNodes_table.resize(instance.map_size, nullptr);
	allNodes_table[root->location] = insert(allNodes_table[root->location], root);
	SIPPNode* curr = nullptr;
	auto static_timestep = constraint_table.getMaxTimestep(); // everything is static after this timestep
	while (!open_list.empty())
	{
		curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			if (static_timestep <= curr->timestep)
			{
				if (curr->location == next_location)
				{
					continue;
				}
				next_timestep--;
			}
			if (!constraint_table.constrained(next_location, next_timestep) &&
				!constraint_table.constrained(curr->location, next_location, next_timestep))
			{  // if that grid is not blocked
				int next_h_val = compute_heuristic(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new SIPPNode(next_location, next_g_val, next_h_val, nullptr, next_timestep,
				        Interval(next_timestep, next_timestep + 1, 0), 0);

                if (dominanceCheck(allNodes_table[next->location], next))
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					allNodes_table[next->location] = insert(allNodes_table[next->location], next);
				}
				else delete next;
			}
		}
	}
	releaseNodes();
	return length;
	/*int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	pairing_heap< SIPPNode*, compare<SIPPNode::compare_node> > open_list;
	// boost::heap::pairing_heap< AStarNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	unordered_set<SIPPNode*, SIPPNode::NodeHasher, SIPPNode::eqnode> nodes;

	Interval interval = reservation_table.get_first_safe_interval(start);
	assert(get<0>(interval) == 0);
	auto root = new SIPPNode(start, 0, instance.getManhattanDistance(start, end), nullptr, 0, interval);
	root->open_handle = open_list.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)

	while (!open_list.empty())
	{
		auto curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		for (int next_location : instance.getNeighbors(curr->location))
		{
			if ((curr->location == blocked.first && next_location == blocked.second) ||
				(curr->location == blocked.second && next_location == blocked.first)) // use the prohibited edge
			{
				continue;
			}

			for (auto interval : reservation_table.get_safe_intervals(
				curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
			{
				int next_timestep = max(curr->timestep + 1, (int)get<0>(interval));
				int next_g_val = next_timestep;
				int next_h_val = instance.getManhattanDistance(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				auto next = new SIPPNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, interval);
				auto it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.update(existing_next->open_handle);
					}
				}
			}
		}
	}
	open_list.clear();
	for (auto node : nodes)
	{
		delete node;
	}
	nodes.clear();
	return length;*/
}


void SIPP::printSearchTree() const
{
    /*vector<list<SIPPNode*>> nodes;
    for (const auto & node_list : allNodes_table)
    {
        for (const auto & n : node_list->second)
        {
            if (nodes.size() <= n->timestep)
                nodes.resize(n->timestep + 1);
            nodes[n->timestep].emplace_back(n);
        }
    }
    cout << "Search Tree" << endl;
    for(int t = 0; t < nodes.size(); t++)
    {
        cout << "t=" << t << ":\t";
        for (const auto & n : nodes[t])
            cout << *n << "[" << get<0>(n->interval) << "," << get<1>(n->interval) << "],\t";
        cout << endl;
    }
*/
}

Path SIPP::findNoCollisionPath(const ConstraintTable& constraint_table)
{
    ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
    num_expanded = 0;
    num_generated = 0;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0 or get<2>(interval))
        return path;
    auto holding_time = max(constraint_table.getHoldingTime(goal_location, constraint_table.length_min),
                            constraint_table.getLastCollisionTimestep(goal_location) + 1);
    // generate start and add it to the OPEN & FOCAL list

    auto start = new SIPPNode(start_location, 0, max(my_heuristic[start_location], holding_time),
            nullptr, 0, interval, 0);
    num_generated++;
    start->in_openlist = true;
    start->focal_handle = focal_list.push(start); // we only use focal list; no open list is used
    allNodes_table.resize(instance.map_size, nullptr);
    allNodes_table[start->location] = insert(allNodes_table[start->location], start);
    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location && // arrive at the goal location
                 !curr->wait_at_goal && // not wait at the goal location
                 curr->timestep >= holding_time && // the agent can hold the goal location afterward
                 constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep) == 0) // no future collisions
        {
            updatePath(curr, path);
            break;
        }
        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            int next_h_val = my_heuristic[next_location];
            for (auto& interval : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, get<1>(curr->interval) + 1))
            {
                if (get<2>(interval.first))
                    continue;
                if (interval.second + next_h_val > constraint_table.length_max)
                    break;
                generateChildToFocal(interval.first, curr, next_location, interval.second, next_h_val);
            }
        }  // end for loop that generates successors
        // wait at the current location
        bool found = reservation_table.find_safe_interval(interval, curr->location, get<1>(curr->interval));
        if (found and !get<2>(interval))
        {
            generateChildToFocal(interval, curr, curr->location, get<0>(interval), curr->h_val);
        }
    }  // end while loop

    //if (path.empty())
    //    printSearchTree();
    releaseNodes();
    return path;
}

void SIPP::mergeNodes(SIPPNode* old_node, SIPPNode* new_node)
{
    /*auto n1 = old_node->timestep <= new_node->timestep? old_node : new_node; // the one with a smaller lower bound
    auto n2 = old_node->timestep <= new_node->timestep? new_node : old_node; // the one with a larger lower bound
    if (get<1>(n1->interval) >= get<1>(n2->interval)) // lb1 <= lb2 < ub2 <= ub1
    { // interval of n2 is a subset of interval of n1
        if (n1->num_of_conflicts <= n2->num_of_conflicts) // n1 has fewer collisions
        { // keep n1 and delete n2
            if (n1 == new_node)
                updateNodeToFocal(old_node, new_node);
            delete new_node;
            return;
        }
        else // n2 has fewer collisions
        {
            if (n1->timestep == n2->timestep) // lb1 = lb2 < ub2 <= ub1
            { // change interval of n1 to [ub2, ub1) if ub1 > ub2 and delete n1 otherwise
                if (get<1>(n1->interval) == get<1>(n2->interval)) // n1.interval = n2.interval
                { // delete n1 and keep n2
                    if (n2 == new_node)
                        updateNodeToFocal(old_node, new_node);
                    delete new_node;
                    return;
                }
                else // lb1 = lb2 < ub2 < ub1
                { // change interval of n1 to [ub2, ub1)
                    n1->interval = make_tuple(get<1>(n2->interval), get<1>(n1->interval), get<2>(n1->interval));
                    if (n1 == old_node)
                        focal_list.update(old_node->focal_handle);
                    new_node->focal_handle = focal_list.push(new_node);
                    new_node->in_openlist = true;
                    num_generated++;
                    allNodes_table.insert(new_node);
                    return;
                }
            }
            else // lb1 < lb2 < ub2 <= ub1
            { // keep both
                new_node->focal_handle = focal_list.push(new_node);
                new_node->in_openlist = true;
                num_generated++;
                allNodes_table.insert(new_node);
                return;

            }
        }
    }
    else // lb1 <= lb2 < ub1 < ub2
    {
        if (n1->num_of_conflicts <= n2->num_of_conflicts) // n1 has fewer collisions
        {// change the interval of n2 to [ub1, ub2)
            n2->interval = make_tuple(get<1>(n1->interval), get<1>(n2->interval), get<2>(n2->interval));
            if (n2 == old_node)
                focal_list.update(old_node->focal_handle);
            new_node->focal_handle = focal_list.push(new_node);
            new_node->in_openlist = true;
            num_generated++;
            allNodes_table.insert(new_node);
            return;
        }
        else // n2 has fewer collisions
        {// change the interval of n1 to [lb1, lb2) if lb2 > lb1 and delete n1 otherwise
            if (n1->timestep == n2->timestep) // lb1 = lb2 < ub1 < ub2 and n2 has fewer collisions
            {// delete n1 and keep n2
                if (n2 == new_node)
                    updateNodeToFocal(old_node, new_node);
                delete new_node;
                return;
            }
            else // lb1 < lb2 < ub1 < ub2
            { // keep both
                new_node->focal_handle = focal_list.push(new_node);
                new_node->in_openlist = true;
                num_generated++;
                allNodes_table.insert(new_node);
                return;
            }
        }
    }*/

    if (old_node->timestep > new_node->timestep || // prefer the one with smaller timestep
        (old_node->timestep == new_node->timestep && old_node->num_of_conflicts > new_node->num_of_conflicts))
        // or it remains the same but there's fewer conflicts
    {
        updateNodeToFocal(old_node, new_node);
    }
    delete new_node;
}

// return true iff we the new node is not dominated by any old node
bool SIPP::dominanceCheck(ITNode* root, SIPPNode* new_node)
{
    list<SIPPNode*> old_nodes;
    overlapSearch(root, new_node, old_nodes);
    for (auto & old_node : old_nodes)
    {
        if (old_node->wait_at_goal == new_node->wait_at_goal and
            old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <= new_node->num_of_conflicts and
            get<1>(old_node->interval) >= get<1>(new_node->interval))
        { // the new node is dominated by the old node
            return false;
        }
        /*else if (old_node->timestep >= new_node->timestep and old_node->num_of_conflicts >= new_node->num_of_conflicts and
                 get<1>(old_node->interval) <= get<1>(new_node->interval)) // the old node is dominated by the new node
        { // delete the old node
            if (old_node->in_openlist) // if its in open
            return true;
        }*/
    }
    return true;
}
void SIPP::updateNodeToFocal(SIPPNode* old_node, const SIPPNode* new_node)
{
    old_node->copy(*new_node);
    if (!old_node->in_openlist) // if its in the closed list (reopen)
    {
        old_node->focal_handle = focal_list.push(old_node);
        old_node->in_openlist = true;
        num_generated++; // reopen is considered as a new node
    }
    else
    {
        focal_list.update(old_node->focal_handle);
    }
}


// A utility function to create a new Interval Search Tree Node
ITNode* SIPP::newNode(SIPPNode* n)
{
    auto temp = new ITNode;
    temp->n = n;
    temp->max = get<1>(n->interval);
    temp->left = temp->right = nullptr;
    return temp;
};

// A utility function to insert a new Interval Search Tree Node
// This is similar to BST Insert.  Here the low value of interval
// is used tomaintain BST property
ITNode* SIPP::insert(ITNode *root, SIPPNode* n)
{
    // Base case: Tree is empty, new node becomes root
    if (root == nullptr)
        return newNode(n);

    // Get low value of interval at root
    int l = get<0>(root->n->interval);

    // If root's low value is smaller, then new interval goes to left subtree
    if (get<0>(n->interval) < l)
        root->left = insert(root->left, n);
    else  // Else, new node goes to right subtree.
        root->right = insert(root->right, n);

    // Update the max value of this ancestor if needed
    if (root->max < get<1>(n->interval))
        root->max = get<1>(n->interval);

    return root;
}

// A utility function to check if given two intervals overlap
bool SIPP::doOVerlap(SIPPNode* n1, SIPPNode* n2)
{
    return get<0>(n1->interval) <= get<1>(n2->interval) and
           get<0>(n2->interval) <= get<1>(n1->interval);
}

// The main function that searches a given interval i in a given
// Interval Tree.
void SIPP::overlapSearch(ITNode *root, SIPPNode* n, list<SIPPNode*>& overlaps)
{
    // Base Case, tree is empty
    if (root == nullptr)
        return;

    // If given interval overlaps with root
    if (doOVerlap(root->n, n))
        overlaps.push_back(root->n);

    // If left child of root is present and max of left child is
    // greater than or equal to given interval, then n may
    // overlap with an interval is left subtree
    if (root->left != nullptr && root->left->max >= get<0>(n->interval))
        overlapSearch(root->left, n, overlaps);
    // Same for the right subtree
    if (root->right != nullptr && root->right->max >= get<0>(n->interval))
        overlapSearch(root->right, n, overlaps);
}

void SIPP::deleteNodes(ITNode *root)
{
    if (root == nullptr) return;
    deleteNodes(root->left);
    deleteNodes(root->right);
    delete root->n;
    delete root;
}