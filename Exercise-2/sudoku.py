
import sys
from collections import deque
from copy import deepcopy
import csv
import time


class Problem:
    s=9
    nexp=0
    ngen=0
    def __init__(self, initial):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
              
        
    def actions(self, state):
	
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        for i in range(0,self.s):
            for j in range(0,self.s):
                if(state[i][j]==0):
                    r,c=i,j
        poss_r=[1,2,3,4,5,6,7,8,9]    #repitition in row
        for i in range(0,9):
            if state[r][i] in poss_r:
                poss_r.remove(state[r][i])
        poss_c=[1,2,3,4,5,6,7,8,9]		#repitition in column
        for i in range(0,9):
            if state[i][c] in poss_c:
                poss_c.remove(state[i][c])
        poss_box=[1,2,3,4,5,6,7,8,9]	#repitition in same box
        for i in range(0,3):
            for j in range(0,3):
                tmp=state[i+r-r%3][j+c-c%3]
                if tmp in poss_box:
                    poss_box.remove(tmp)
        l=[]
        for i in poss_r:
            if(i in poss_c and i in poss_box):
                l.append(i)
        self.ngen=self.ngen+len(l)
        return l
				
		
		

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        l=[1,2,3,4,5,6,7,8,9]
        for i in range(0,9):
            for j in range(0,9):
                if(state[i][j]==0):
                    r,c=i,j
        new_s=deepcopy(state)
        if(action in l):
            new_s[r][c]=action
            return new_s

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        self.nexp=self.nexp+1
        if (state==self.initial):
            return False
        for i in range(0,9):
            for j in range(0,9):
                tmp=[1,2,3,4,5,6,7,8,9]
                l=[]
                for k in range(0,9):		#check row repitition
                    l.append(state[i][k])
                l.sort()
                if(tmp!=l):
                    return False
                tmp=[1,2,3,4,5,6,7,8,9]
                l=[]
                for k in range(0,9):		#check column repitition
                    l.append(state[k][j])
                l.sort()
                if(tmp!=l):
                    return False
                poss_box=[1,2,3,4,5,6,7,8,9]
                tmp=[]									#repitition in same box
                for x in range(0,3):
                    for y in range(0,3):
                        tmp.append(state[x+i-i%3][y+j-j%3])
                tmp.sort()
                if(tmp!=poss_box):
                    return False
        return True
				
				
								
					
    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1
    
    
    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError


# ______________________________________________________________________________


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
        return self.state < node.state

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
        return hash(self.state)


# ______________________________________________________________________________


class SimpleProblemSolvingAgentProgram:
    """
    [Figure 3.1]
    Abstract framework for a problem-solving agent.
    """

    def __init__(self, initial_state=None):
        """State is an abstract representation of the state
        of the world, and seq is the list of actions required
        to get to a particular state from the initial state(root)."""
        self.state = initial_state
        self.seq = []

    def __call__(self, percept):
        """[Figure 3.1] Formulate a goal and problem, then
        search for a sequence of actions to solve it."""
        self.state = self.update_state(self.state, percept)
        if not self.seq:
            goal = self.formulate_goal(self.state)
            problem = self.formulate_problem(self.state, goal)
            self.seq = self.search(problem)
            if not self.seq:
                return None
        return self.seq.pop(0)

    def update_state(self, state, percept):
        raise NotImplementedError

    def formulate_goal(self, state):
        raise NotImplementedError

    def formulate_problem(self, state, goal):
        raise NotImplementedError

    def search(self, problem):
        raise NotImplementedError


# ______________________________________________________________________________
# Uninformed Search algorithms


def breadth_first_tree_search(problem):
    """
    [Figure 3.7]
    Search the shallowest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Repeats infinitely in case of loops.
    """

    frontier = deque([Node(problem.initial)])  # FIFO queue

    while frontier:
        node = frontier.popleft()
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))
    return None


def depth_first_tree_search(problem):
    """
    [Figure 3.7]
    Search the deepest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Repeats infinitely in case of loops.
    """

    frontier = [Node(problem.initial)]  # Stack

    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))
    return None

def depth_limited_search(problem, limit=50):
    """[Figure 3.17]"""

    def recursive_dls(node, problem, limit):
        if problem.goal_test(node.state):
            return node
        elif limit == 0:
            return 'cutoff'
        else:
            cutoff_occurred = False
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit - 1)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            return 'cutoff' if cutoff_occurred else None

    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit)


def iterative_deepening_search(problem):
    """[Figure 3.18]"""
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result != 'cutoff':
            return result


# ______________________________________________________________________________
with open('sudoku.csv','r') as file:
    reader=csv.reader(file)
    next(reader)
    s_p=[]
    for row,c in zip(reader,range(100000)):
        prob=[]
        ind=0
        for i in range(0,9):
            l=[]
            for j in range(0,9):
                l.append(int(row[0][ind]))
                ind=ind+1
            prob.append(l)
        s_p.append(prob)
print("**********************************************************")
print("Breadth first search ")
node_e=0
node_g=0
total_time=0
print("Total instances: 100")
for i in range(0,100):
    s=Problem(s_p[i])
    start=time.time()
    temp=breadth_first_tree_search(s)
    end=time.time()
    total_time=end-start
    node_g=node_g+s.ngen
    node_e=node_e+s.nexp
print("Total nodes generated: ",node_g)
print("Total nodes Expanded: ",node_e)		    
print("Time taken: ",total_time)	
print("**********************************************************")
print("Depth first search ")
node_e=0
node_g=0
total_time=0
print("Total instances: 100")
for i in range(0,100):
    s=Problem(s_p[i])
    start=time.time()
    temp=depth_first_tree_search(s)
    end=time.time()
    total_time=end-start
    node_g=node_g+s.ngen
    node_e=node_e+s.nexp
print("Total nodes generated: ",node_g)
print("Total nodes Expanded: ",node_e)		    
print("Time taken: ",total_time)	
print("**********************************************************")
print("Depth limited search ")
node_e=0
node_g=0
total_time=0
print("Total instances: 100")
for i in range(0,100):
    s=Problem(s_p[i])
    start=time.time()
    temp=depth_limited_search(s)
    end=time.time()
    total_time=end-start
    node_g=node_g+s.ngen
    node_e=node_e+s.nexp
print("Total nodes generated: ",node_g)
print("Total nodes Expanded: ",node_e)		    
print("Time taken: ",total_time)	
print("**********************************************************")
print("Iterative deepening ")
node_e=0
node_g=0
total_time=0
print("Total instances: 100")
for i in range(0,100):
    s=Problem(s_p[i])
    start=time.time()
    temp=iterative_deepening_search(s)
    end=time.time()
    total_time=end-start
    node_g=node_g+s.ngen
    node_e=node_e+s.nexp
print("Total nodes generated: ",node_g)
print("Total nodes Expanded: ",node_e)		    
print("Time taken: ",total_time)	


""" Output
**********************************************************
Breadth first search 
Total instances: 100
Total nodes generated:  20690
Total nodes Expanded:  20790
Time taken:  0.03091740608215332
**********************************************************
Depth first search 
Total instances: 100
Total nodes generated:  12973
Total nodes Expanded:  12400
Time taken:  0.00797891616821289
**********************************************************
Depth limited search 
Total instances: 100
Total nodes generated:  13926
Total nodes Expanded:  13456
Time taken:  0.02693009376525879
**********************************************************
Iterative deepening 
Total instances: 100
Total nodes generated:  626600
Total nodes Expanded:  630842
Time taken:  1.033280849456787
"""