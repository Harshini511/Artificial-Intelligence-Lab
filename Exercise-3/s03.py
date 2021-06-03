
import time
from utils import *

class Point:
    def __init__(self,x,y):
        self.x=x
        self.y=y

def on_segment(p,q,r):
    "return true if point q is in segment pr"
    if((q.x<=max(p.x,r.x))and(q.x>=min(p.x,r.x))and(q.y<=max(p.y,r.y))and (q.y>=min(p.y,r.y))):
        return True
    return False

def orientation(p,q,r):
    "return orientation of three points"
    val=(float(q.y-p.y) * (r.x-q.x))-(float(q.x-p.x)*(r.y-q.y))
    if (val>0):
        return 1
    elif (val<0):
        return 2
    else:
        return 0
    
def do_intersect(p1,q1,p2,q2):
    
    o1=orientation(p1,q1,p2)
    o2=orientation(p1,q1,q2)
    o3=orientation(p2,q2,p1)
    o4=orientation(p2,q2,q1)
    if((o1!=o2)and (o3!=o4)):
        return True
    if((o1==0) and on_segment(p1,p2,q1)):
        return True
    if((o2==0) and on_segment(p1,q2,q1)):
        return True
    if((o3==0) and on_segment(p2,p1,q2)):
        return True
    if((o4==0) and on_segment(p2,q1,q2)):
        return True
    return False


class Problem:
    def __init__(self,initial,goal,states,polygons,edges):
        self.initial=initial
        self.goal=goal
        self.states=states
        self.polygons=polygons
        self.edges=edges
        self.expn=0
        self.genn=0
        
    def actions(self,state):
        self.expn=self.expn+1
        poss=[]
        x,y=state
        edges=self.edges
        for i in self.polygons:
            if(j in i):
                for p in edges:
                    if(p[0]==j):
                        poss.append(p[1])
                    elif(p[1]==j):
                        poss.append(p[0])
        for i in self.states:
            if(state==i):
                continue
            temp=[]
            for points in edges:
                if(state not in points and i not in points):
                    temp.append(do_intersect(Point(x,y),Point(i[0],i[1]),Point(points[0][0],points[0][1]),Point(points[1][0],points[1][1])))
            if(True not in temp and i not in poss):
                poss.append(i)
        return poss
    
    def result(self,state,action):
        self.genn=self.genn+1
        return action
    
    def goal_test(self,state):
        if isinstance(self.goal,list):
            return is_in(state,self.goal)
        else:
            return state==self.goal
    
    def path_cost(Self,c,state1,action,state2):
        x1,y1=state1
        x2,y2=state2
        dist=((((abs(x2-x1))**2)+((abs(y2-y1))**2)**0.5))
        return dist
    
    def h(self,node):
        x1,y1=node.state
        x2,y2=self.goal
        dist=((((abs(x2-x1))**2)+((abs(y2-y1))**2)**0.5))
        return dist
        
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




def depth_first_graph_search(problem):
    """
    [Figure 3.7]
    Search the deepest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Does not get trapped by loops.
    If two paths reach a state, only use the first one.
    """
    frontier = [(Node(problem.initial))]  # Stack

    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and child not in frontier)
    return None

def best_first_graph_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            if display:
                print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
    return None
def breadth_first_graph_search(problem):
    """[Figure 3.11]
    Note that this function can be implemented in a
    single line as below:
    return graph_search(problem, FIFOQueue())
    """
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = deque([node])
    explored = set()
    while frontier:
        node = frontier.popleft()
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
    return None



def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)

   

    
initial=(1,3)
polygons=[[(0,14),(6,19),(9,15),(7,8),(1,9)],
           [(2,6),(17,6),(17,1),(2,1)],
           [(12,15),(14,8),(10,8)],
           [(18,10),(23,6),(19,3)],
           [(14,13),(14,19),(18,20),(20,17)],
           [(22,19),(28,19),(28,9),(22,9)],
           [(32,8),(29,17),(31,19),(34,16)],
           [(28,1),(25,2),(25,6),(29,8),(31,6),(31,2)]]
goal=(34,19)
states=[]
for i in polygons:
    for j in i:
        states.append(j)
states.append(goal) 
edges=[]
for i in polygons:
        for j in range(len(i)):
            edges.append([i[j],i[j-1]])
            
soln=Problem(initial,goal,states,polygons,edges)

print("DFS")
start_time=time.time()
x=depth_first_graph_search(soln)
end_time=time.time()
total_time=end_time-start_time
for i in x.path():
    print(i.state)
print("cost: ",x.path_cost)
print("Total time taken: ",total_time)
print("no of nodes generated: ",soln.genn)
print("no of nodes expanded: ",soln.expn)

print("A*")
start_time=time.time()
x=astar_search(soln)
end_time=time.time()
total_time=end_time-start_time
for i in x.path():
    print(i.state)
print("cost: ",x.path_cost)
print("Total time taken: ",total_time)
print("no of nodes generated: ",soln.genn)
print("no of nodes expanded: ",soln.expn)


print("Best first greedy search")
start_time=time.time()
x=breadth_first_graph_search(soln)
end_time=time.time()
total_time=end_time-start_time
for i in x.path():
    print(i.state)
print("cost: ",x.path_cost)
print("Total time taken: ",total_time)
print("no of nodes generated: ",soln.genn)
print("no of nodes expanded: ",soln.expn)

""" OUTPUT
DFS
(1, 3)
(2, 1)
(17, 1)
(25, 6)
(31, 2)
(34, 16)
(34, 19)
cost:  3.0
Total time taken:  0.03284263610839844
no of nodes generated:  43
no of nodes expanded:  6
A*
(1, 3)
(2, 6)
(17, 6)
(19, 3)
(25, 6)
(29, 8)
(32, 8)
(34, 16)
(34, 19)
cost:  3.0
Total time taken:  0.031242847442626953
no of nodes generated:  112
no of nodes expanded:  14
Best first greedy search
(1, 3)
(0, 14)
(6, 19)
(18, 20)
(34, 19)
cost:  257.0
Total time taken:  0.06903982162475586
no of nodes generated:  222
no of nodes expanded:  29
"""