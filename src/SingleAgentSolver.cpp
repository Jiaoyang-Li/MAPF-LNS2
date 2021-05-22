#include "SingleAgentSolver.h"
#include "SpaceTimeAStar.h"

list<int> SingleAgentSolver::getNextLocations(int curr) const // including itself and its neighbors
{
	list<int> rst = instance.getNeighbors(curr);
	rst.emplace_back(curr);
	return rst;
}


void SingleAgentSolver::compute_heuristics()
{
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}
		// the following is used to comapre nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
	};

	my_heuristic.resize(instance.map_size, MAX_TIMESTEP);

	// generate a heap that can save nodes (and a open_handle)
	boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;

	Node root(goal_location, 0);
	my_heuristic[goal_location] = 0;
	heap.push(root);  // add root to heap
	while (!heap.empty())
	{
		Node curr = heap.top();
		heap.pop();
		for (int next_location : instance.getNeighbors(curr.location))
		{
			if (my_heuristic[next_location] > curr.value + 1)
			{
				my_heuristic[next_location] = curr.value + 1;
				Node next(next_location, curr.value + 1);
				heap.push(next);
			}
		}
	}
}

// find the optimal no wait path by A* search
// Returns a path that minimizes cost, breaking ties by number of target locations visited (treated ad num_of_conflicts).
Path SingleAgentSolver::findNoWaitPath(vector<int>& goal_table,set<int>& A_target)
{
    // define typedefs and handles for heap
    typedef pairing_heap< AStarNode*, compare<AStarNode::compare_node> > heap_open_t;
    typedef pairing_heap< AStarNode*, compare<AStarNode::secondary_compare_node> > heap_focal_t;
    heap_open_t open_list;
    heap_focal_t focal_list;

    // define typedef for hash_map
    typedef unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
    hashtable_t allNodes_table;

    Path path;
    num_expanded = 0;
    num_generated = 0;

    // build constraint table
    auto t = clock();

    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0,
                               my_heuristic[start_location], nullptr, 0, 0, false);

    num_generated++;
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);
    min_f_val = (int) start->getFVal();
    // lower_bound = int(w * min_f_val));

    while (!open_list.empty())
    {
        // update FOCAL if min f-val increased
        auto open_head = open_list.top();
        if (open_head->getFVal() > min_f_val)
        {
            int new_min_f_val = (int)open_head->getFVal();
            for (auto n : open_list)
            {
                if (n->getFVal() >  w * min_f_val && n->getFVal() <= w * new_min_f_val)
                    n->focal_handle = focal_list.push(n);
            }
            min_f_val = new_min_f_val;
        }

        auto* curr = focal_list.top(); focal_list.pop();
        open_list.erase(curr->open_handle);
        curr->in_openlist = false;
        num_expanded++;

        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location  // arrive at the goal location
                )
        {
            LLNode* pt = curr;
            if (pt->is_goal)
                pt = curr->parent;
            path.reserve(pt->g_val + 1);
            while (pt != nullptr)
            {
                path.emplace_back(pt->location);
                pt = pt->parent;
            }
            std::reverse(path.begin(),path.end());

            for(auto p : path){
                if (goal_table[p.location] > -1 && p.location!=goal_location)
                    A_target.insert(goal_table[p.location]);
            }
            break;
        }

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep;


            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = my_heuristic[next_location];

            int num_of_visted_goals = curr->num_of_conflicts + goal_table[next_location] > -1? 1:0;



            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, num_of_visted_goals, false);

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                num_generated++;
                if (next->getFVal() <= w * min_f_val)
                    next->focal_handle = focal_list.push(next);
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
                (existing_next->getFVal() == next->getFVal() &&
                 existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
            {
                if (!existing_next->in_openlist) // if its in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    existing_next->open_handle = open_list.push(existing_next);
                    existing_next->in_openlist = true;
                    num_generated++;
                    if (existing_next->getFVal() <= w * min_f_val)
                        existing_next->focal_handle = focal_list.push(existing_next);
                }
                else
                {
                    bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
                    bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                    bool update_open = false;
                    if ((next_g_val + next_h_val) <= w * min_f_val)
                    {  // if the new f-val qualify to be in FOCAL
                        if (existing_next->getFVal() > w * min_f_val)
                            add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                        else
                            update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
                    }
                    if (existing_next->getFVal() > next_g_val + next_h_val)
                        update_open = true;

                    existing_next->copy(*next);	// update existing node

                    if (update_open)
                        open_list.increase(existing_next->open_handle);  // increase because f-val improved
                    if (add_to_focal)
                        existing_next->focal_handle = focal_list.push(existing_next);
                    if (update_in_focal)
                        focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    for (auto node: allNodes_table)
        delete node;
    return path;
}
