
def manhattan(a,c):
  return (abs(a[0]-c[0])+abs(a[1]-c[1]))


def heuristics(points,c):
  h=0
  for a in points:
    h+=manhattan(a,c)
  return h

def next_states(c):
  l=[(c[0],c[1]+1),
     (c[0],c[1]-1),
     (c[0]+1,c[1]),
     (c[0]-1,c[1])]
  return l

def min_successor(points,l,c):
  min_value=heuristics(points,c)
  state= c

  for i in l:
    h=heuristics(points,i)
    if h<min_value:
      min_value=h
      state=i

  return min_value,state

def hill_climbing(points,c):

  current=c

  while (1):

    succ=next_states(current)
    h_neighbour,neighbour = min_successor(points,succ,current)
    h_current=heuristics(points,current)

    if (h_neighbour>=h_current):
      print("Centre: ",current,"\nMinimum distance : ",h_current)
      return
    else:
      current=neighbour

import random
n=5
points=[]
for i in range(n):
  x=int(random.random()*10)
  y=int(random.random()*10)
  points+=[(x,y)]

x=int(random.random()*10)
y=int(random.random()*10)
center=(x,y)

print("Random points : ",points)

hill_climbing(points,center)

'''Sample Input/Output


Random points :  [(3, 2), (4, 1), (2, 7), (0, 3), (4, 2)]
Centre:  (3, 2) 
Minimum distance :  13'''
