def bfs(node):
  path_cost=0
  explored=[]
  discovered=[node] 
  i=0
  while (len(discovered)>0 and i<10):
    i+=1
    node=discovered.pop(0)
    explored.append(node)
    
    for child in actions(node):
      
      if (child not in discovered) or (child not in explored):
        if child == goal:
          return
        discovered.append(child)
        
  
  print("Path cost = ",path_cost)

def GBFS(parent):
    global present_state,final_states
    visited={}
    frontier= PriorityQueue()
    frontier.put((h(present_state),present_state))
    visited[present_state]=1
    i=0

    while not frontier.empty():
        current=frontier.get()[1]
        if(current==final_states):
            return parent
        next_state=next_states(current)
        for st in next_state:
            if(st not in visited):
                parent[st]=current
                visited[st]=1
                if(st==final_states):
                    return parent
                frontier.put((h(st),st))

def ASTAR(parent):
    global present_state,final_states
    visited={}
    frontier= PriorityQueue()
    frontier.put((h(present_state),present_state))
    visited[present_state]=1
    i=0
    depth=0

    while not frontier.empty():
        current=frontier.get()[1]
        if(current==final_states):
            return parent
        next_state=next_states(current)
        for st in next_state:
            if(st not in visited):
                parent[st]=current
                visited[st]=1
                depth+=1
                if(st==final_states):
                    return parent
                frontier.put((h(st)+depth,st))