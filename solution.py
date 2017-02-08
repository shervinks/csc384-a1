#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
import math
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    shortest = math.inf
    dis_sum = 0
    distance = 0
    for box in state.boxes:
      shortest = math.inf
      if state.restrictions == None:
        #find closest storage
        for storage in state.storage:
          distance = get_distance(box, storage)
          if  distance < shortest:
              shortest = distance
        dis_sum += shortest
      else:
        #find closest storage in restriction
        for storage in state.restrictions[state.boxes[box]]:
          distance = get_distance(box, storage)
          if  distance < shortest:
              shortest = distance
        dis_sum += shortest
    return dis_sum

def get_distance(t1, t2):
    return abs(t1[0] - t2[0]) + abs(t1[1] - t2[1])
  
def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    '''state.print_state()
    for b in state.boxes:
      
      print(no_goal_along_wall(state, b))

    return 0'''
    
    #for each box check for deadlocks
    for b in state.boxes:
      if state.restrictions == None:
        #check if box is already in storage
        if b not in state.storage:
          #check for deadlocks
          if touching_corner(state, b):
            return math.inf
          if along_wall(state, b):
            return math.inf
          if no_goal_along_wall(state, b):
            return math.inf
      else:
        #check if box is already in storage
        if b not in state.restrictions[state.boxes[b]]:
          #check for deadlocks
          if touching_corner(state, b):
            return math.inf
          if along_wall(state, b):
            return math.inf
          if no_goal_along_wall(state, b):
            return math.inf
           
    return heur_manhattan_distance(state)
      
def no_goal_along_wall(state, box):
    along_wall = False
    #touching west wall
    if box[0] == 0:
      along_wall = True
      if state.restrictions == None:
        for s in state.storage:
          if s[0] == 0:
            return False
      else:
        for s in state.restrictions[state.boxes[box]]:
          if s[0] == 0:
            return False
    #touching east wall
    if box[0] == state.width - 1:
      along_wall = True
      if state.restrictions == None:
        for s in state.storage:
          if s[0] == state.width - 1:
            return False
      else:
        for s in state.restrictions[state.boxes[box]]:
          if s[0] == state.width - 1:
            return False
    #touching north wall
    if box[1] == 0:
      along_wall = True
      if state.restrictions == None:
        for s in state.storage:
          if s[1] == 0:
            return False
      else:
        for s in state.restrictions[state.boxes[box]]:
          if s[1] == 0:
            return False
    #touching south wall
    if box[1] == state.height - 1:
      along_wall = True
      if state.restrictions == None:
        for s in state.storage:
          if s[1] == state.height - 1:
            return False
      else:
        for s in state.restrictions[state.boxes[box]]:
          if s[1] == state.height - 1:
            return False

    return along_wall
          
def along_wall(state, box):
    #touching west wall
    if box[0] == 0:
      for b in state.boxes:
        if b[0] == 0:
          if b[1] == box[1] - 1 or b[1] == box[1] + 1:
            return True
    #touching east wall
    if box[0] == state.width - 1:
      for b in state.boxes:
        if b[0] == state.width - 1:
          if b[1] == box[1] - 1 or b[1] == box[1] + 1:
            return True
    #touching north wall
    if box[1] == 0:
      for b in state.boxes:
        if b[1] == 0:
          if b[0] == box[0] - 1 or b[0] == box[0] + 1:
            return True
    #touching south wall
    if box[1] == state.height - 1:
      for b in state.boxes:
        if b[1] == state.height - 1:
          if b[0] == box[0] - 1 or b[0] == box[0] + 1:
            return True
    return False
  
def touching_corner(state, box):
    #base case map corners
    if box == (0, 0) or box == (0, state.height-1) or box == (state.width-1, 0) or box == (state.width-1, state.height-1):
      return True
    
    #corner with 1 wall 1 obstacle
    for o in state.obstacles:
      #touching west wall
      if box[0] == 0:
        if o[0] == 0:
          if o[1] == box[1] - 1 or o[1] == box[1] + 1:
            return True
      #touching east wall
      if box[0] == state.width - 1:
        if o[0] == state.width - 1:
          if o[1] == box[1] - 1 or o[1] == box[1] + 1:
            return True
      #touching north wall
      if box[1] == 0:
        if o[1] == 0:
          if o[0] == box[0] - 1 or o[0] == box[0] + 1:
            return True
      #touching south wall
      if box[1] == state.height - 1:
        if o[1] == state.height - 1:
          if o[0] == box[0] - 1 or o[0] == box[0] + 1:
            return True

    #corner with 2 obstacles
    for o in state.obstacles:
      #box touching west side of obstacle
      if o[0] > 2:
        if box[0] == o[0] - 1 and box[1] == o[1]:
          #check if any other obstacles is in a position to create a corner
          for o2 in state.obstacles:
            if (o2[0] == o[0] - 1 and o2[1] == o[1] - 1) or (o2[0] == o[0] - 1 and o2[1] == o[1] + 1):
              return True
      #box touching east side of obstacle
      if o[0] < state.width - 2:
        if box[0] == o[0] + 1 and box[1] == o[1]:
          #check if any other obstacles is in a position to create a corner
          for o2 in state.obstacles:
            if (o2[0] == o[0] + 1 and o2[1] == o[1] - 1) or (o2[0] == o[0] + 1 and o2[1] == o[1] + 1):
              return True
      #box touching north side of obstacle
      if o[1] > 2:
        if box[1] == o[1] - 1 and box[0] == o[0]:
          #check if any other obstacles is in a position to create a corner
          for o2 in state.obstacles:
            if (o2[0] == o[0] - 1 and o2[1] == o[1] - 1) or (o2[0] == o[0] + 1 and o2[1] == o[1] - 1):
              return True
      #box touching south side of obstacle
      if o[1] > 2:
        if box[1] == o[1] + 1 and box[0] == o[0]:
          #check if any other obstacles is in a position to create a corner
          for o2 in state.obstacles:
            if (o2[0] == o[0] - 1 and o2[1] == o[1] + 1) or (o2[0] == o[0] + 1 and o2[1] == o[1] + 1):
              return True
    return False
    
def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    
    return sN.gval + weight*sN.hval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    start_time = os.times()[0]
    stop_time = start_time + timebound
    goal = False

    g_bound = math.inf
    h_bound = math.inf
    gh_bound = math.inf
    
    while (os.times()[0] < stop_time):
      time_left = stop_time - os.times()[0]
      se = SearchEngine("best_first")
      se.init_search(initial_state, sokoban_goal_state, heur_fn)
      final = se.search(time_left, (g_bound, h_bound, gh_bound))
      if not final:
        break
      goal = final
      g_bound = final.gval-1

    return goal

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    start_time = os.times()[0]
    stop_time = start_time + timebound
    goal = False

    g_bound = math.inf
    h_bound = math.inf
    gh_bound = math.inf

    while (os.times()[0] < stop_time):
      time_left = stop_time - os.times()[0]
      se = SearchEngine("custom")
      se.init_search(initial_state, sokoban_goal_state, heur_fn, lambda sN: fval_function(sN, weight))
      final = se.search(time_left, (g_bound, h_bound, gh_bound))
      if not final:
        break
      goal = final
      gh_bound = final.gval-1    
    
    return goal

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star", "full")      

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 



