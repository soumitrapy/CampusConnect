from utils import memoize, PriorityQueue

class Order:
    def __init__(self, name, prefs) -> None:
        self.name = name
        self.prefs = prefs
        
    def update_pref(self, pref):
        self.prefs[pref[0]] = pref[1]

    def __str__(self) -> str:
        return str(self.name)+':'+str(self.prefs)

class Problem:
    def __init__(self, initial, goal=None):
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def goal_test(self, state):
        raise NotImplementedError

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def value(self, state):
        raise NotImplementedError

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        #self.orders = orders
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return len(self.state) < len(node.state)

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        s = 0
        for x in self.state.items():
            s += hash(x)
        return s
    

class CampusConnectProblem:
    def __init__(self, orders):
        self.orders = orders
        self.state = dict()
        self.all_prefs = set()
        for o in orders:
            for t in o.prefs:
                pref = (t, o.prefs[t])
                self.all_prefs.add(pref)
        

    def actions(self, state):
        possible_actions = set()
        for pref in self.all_prefs:
            if pref[0] not in self.state:
                possible_actions.add(pref)

        return possible_actions

    def result(self, state, action):
        newstate = state.copy()
        newstate[action[0]] = action[1]
        return newstate
    
    def number_of_order_possible(self, state):
        s = 0
        for o in self.orders:
            for t in o.prefs:
                if t in state and state[t]==o.prefs[t]:
                    s += 1
                    break
        return s
        
    def goal_test(self, state):
        return self.number_of_order_possible(state)==len(self.orders)
    
    def h(self, node):
        return len(self.orders)-self.number_of_order_possible(node.state)
    
    def path_cost(self, c, state1, action, state2):
        return c + 1
        
            
def best_first_graph_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.state)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        print(node)
        if problem.goal_test(node.state):
            if display:
                print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
            return node
        explored.add(node)
        for child in node.expand(problem):
            if child not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
    return None

def uniform_cost_search(problem, display=False):
    """[Figure 3.14]"""
    return best_first_graph_search(problem, lambda node: node.path_cost, display)

def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)

if __name__=='__main__':
    orders = {
    Order('s1',{9:'a',10:'b',11:'c'}),
    Order('s2',{12:'c'}),
    #Order('s3',{9:'a', 12:'c',10:'b',13:'d'}),
    Order('s4',{9:'d'}),
    Order('s5',{12:'d'}),
    }
    cp = CampusConnectProblem(orders)
    sol = astar_search(cp)
    print(f'solution is: {sol}')

