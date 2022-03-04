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
		// the following is used to compare nodes in the OPEN list
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
// Returns a path that minimizes the number of target locations visited, breaking ties by cost.
void SingleAgentSolver::findMinimumSetofColldingTargets(vector<int>& goal_table, set<int>& A_target)
{
    struct Node
    {
        int location;
        int g_val;
        int h_val;
        int num_of_targets;
        Node* parent;
        bool expanded = false;

        Node() = default;
        Node(int location, int g_val, int h_val, int num_of_targets, Node* parent) :
            location(location), g_val(g_val), h_val(h_val), num_of_targets(num_of_targets), parent(parent) {}

        // the following is used to compare nodes in the OPEN list
        struct compare_node
        {
            // returns true if n1 > n2(note -- this gives us *min*-heap).
            bool operator()(const Node* n1, const Node* n2) const
            {
                if (n1->num_of_targets == n2->num_of_targets)
                {
                    if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                    {
                        return n1->h_val >= n2->h_val;
                    }
                    return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
                }
                return n1->num_of_targets >= n2->num_of_targets;
            }
        };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

        pairing_heap<Node*,boost::heap::compare<compare_node>>::handle_type open_handle;
    };


    pairing_heap<Node*,boost::heap::compare<Node::compare_node>> open_list;
    vector<Node*> visited(goal_table.size(), nullptr);
    Path path;

    // generate start and add it to the OPEN
    auto start = new Node(start_location, 0, my_heuristic[start_location], 0, nullptr);
    start->open_handle = open_list.push(start);
    start->expanded = true;
    visited[start->location] = start;

    while (!open_list.empty())
    {
        auto curr = open_list.top(); open_list.pop();
        curr->expanded = true;

        // check if the popped node is a goal
        if (curr->location == goal_location)  // arrive at the goal location
        {
            auto pt = curr;
            path.reserve(pt->g_val + 1);
            while (pt != nullptr)
            {
                path.emplace_back(pt->location);
                pt = pt->parent;
            }
            std::reverse(path.begin(),path.end());

            for(auto& p : path)
            {
                if (goal_table[p.location] > -1 && p.location != goal_location)
                    A_target.insert(goal_table[p.location]);
            }
            break;
        }

        auto next_locations = instance.getNeighbors(curr->location);
        for (int next_location : next_locations)
        {
            int next_g_val = curr->g_val + 1;
            int num_of_visited_targets = curr->num_of_targets + goal_table[next_location] > -1? 1:0;
            if (visited[next_location] == nullptr)
            {
                // generate (maybe temporary) node
                auto next = new Node(next_location, next_g_val, my_heuristic[next_location],
                        num_of_visited_targets, curr);
                next->open_handle = open_list.push(next);
                visited[next_location] = next;
                continue;
            }
            else if (num_of_visited_targets < visited[next_location]->num_of_targets or
                    (num_of_visited_targets == visited[next_location]->num_of_targets and
                            next_g_val < visited[next_location]->g_val)) // update existing node's if needed (only in the open_list)
            {
                visited[next_location]->g_val = next_g_val;
                visited[next_location]->num_of_targets = num_of_visited_targets;
                visited[next_location]->parent = curr;
                if (visited[next_location]->expanded) // if its in the closed list (reopen)
                {
                    visited[next_location]->open_handle = open_list.push(visited[next_location]);
                    visited[next_location]->expanded = false;
                }
                else // it is in the open list
                {
                    open_list.increase(visited[next_location]->open_handle);  // increase because f-val improved
                }
            }
        }  // end for loop that generates successors
    }  // end while loop

    for (auto node: visited)
        if (node != nullptr)
            delete node;
}

std::ostream& operator<<(std::ostream& os, const LLNode& node)
{
    os << node.location << "@" << node.timestep << "(f=" << node.g_val << "+" << node.h_val << ")";
    return os;
}
