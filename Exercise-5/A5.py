
import random

def random_parent():
  p=[]
  while(len(p)<8):
    c=random.randint(1,8)
    if c not in p:
      p.append(c)

  return p

def fitness_fn(x):
  #here fitness function will return the no. of nonattacking pairs of queen

  max=28
  conflict=0
  
  for i in range(8):
    for j in range(i+1,8):
      if(x[i] == x[j]):
        conflict += 1

      #checks if they are on the same diagonal
      if(x[i] + j - i == x[j] or x[i] + i - j == x[j]):
        conflict += 1

  return max-conflict

def random_pick(population,probabilities):
  z=zip(population,probabilities)
  total=sum(probabilities)

  r= random.uniform(0,total)
  upto=0

  for c, w in z:
    if upto+w>=r:
      return c
    upto+=w

  return None

#a crossover between 2 parents
def reproduce(x,y):
  c= random.randint(0,len(x))
  return x[:c]+y[c:n]

def mutate(x):
  c=random.randint(0,8-1)
  m=random.randint(1,8)
  x[c]=m

  return x

def gen_algo(population,fitness_fn):

  mutation_prob=0.03
  new_population=[]
  maxFitness=28 
  probabilities=[fitness_fn(x)/28 for x in population]

  for i in range(len(population)):
    x=random_pick(population,probabilities) #selecting 1st random parent
    y=random_pick(population,probabilities) #selecting 2nd random parent

    child=reproduce(x,y)

    if random.random()<mutation_prob:
      child=mutate(child)

    new_population.append(child)
    if fitness_fn(child)==maxFitness : break

  return new_population

population = [ random_parent() for i in range(100)]
generation=1
maxFitness=28


while maxFitness not in [fitness_fn(c) for c in population]:
  population=gen_algo(population,fitness_fn)
  generation+=1

print("Solved in Generation {}!".format(generation-1))

for c in population:
  if fitness_fn(c) == maxFitness:
    break

print("One of the solutions: ", list(c))
print("Fitness: ", fitness_fn(c))

chess=[]
print("Chess board: ")
for i in range(8):
  chess.append([" x "]*8)
for j in range(8):
  chess[j][int(c[j])-1]=" Q "

for i in range(8):
  for j in range(8):
    print(chess[i][j],end="")
  print()



''' Output sample 1: 

Solved in Generation 1570!
One of the solutions:  [5, 8, 4, 1, 7, 2, 6, 3]
Fitness:  28
Chess board: 
 x  x  x  x  Q  x  x  x 
 x  x  x  x  x  x  x  Q 
 x  x  x  Q  x  x  x  x 
 Q  x  x  x  x  x  x  x 
 x  x  x  x  x  x  Q  x 
 x  Q  x  x  x  x  x  x 
 x  x  x  x  x  Q  x  x 
 x  x  Q  x  x  x  x  x 


 Output sample 2:

 Solved in Generation 795!
One of the solutions:  [4, 7, 5, 2, 6, 1, 3, 8]
Fitness:  28
Chess board: 
 x  x  x  Q  x  x  x  x 
 x  x  x  x  x  x  Q  x 
 x  x  x  x  Q  x  x  x 
 x  Q  x  x  x  x  x  x 
 x  x  x  x  x  Q  x  x 
 Q  x  x  x  x  x  x  x 
 x  x  Q  x  x  x  x  x 
 x  x  x  x  x  x  x  Q 

Output sample 3:

 Solved in Generation 0!
One of the solutions:  [5, 7, 1, 3, 8, 6, 4, 2]
Fitness:  28
Chess board: 
 x  x  x  x  Q  x  x  x 
 x  x  x  x  x  x  Q  x 
 Q  x  x  x  x  x  x  x 
 x  x  Q  x  x  x  x  x 
 x  x  x  x  x  x  x  Q 
 x  x  x  x  x  Q  x  x 
 x  x  x  Q  x  x  x  x 
 x  Q  x  x  x  x  x  x 

 '''