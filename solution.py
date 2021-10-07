#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems
#MY IMPORTS, CHECK WITH TAS!!!!!!!!!!!!!!!!
import operator
import heapq
import numpy as np


def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    sum_man_dist = 0
    boxes = set(state.boxes)
    storage = set(state.storage)

    for box in boxes:
        man_dist = float("inf")
        for store in storage:
            man_dist = min(man_dist, calc_manhattan_distance(box, store))
        sum_man_dist += man_dist

    return sum_man_dist

def calc_manhattan_distance(box, store):
    return abs(box[0] - store[0]) + abs(box[1] - store[1])

def calc_diag_manhattan_distance(box, robot):
    # distance assuming robots can move on diagonals (don't want to punish robot for "exploring" around a box)
    return max(abs(box[0] - robot[0]), abs(box[1] - robot[1]))

#SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count

def corner_stuck(box, state):
    stuck_x = (box[0] in [0, state.width-1]) or ((box[0] + 1, box[1]) in state.obstacles) or ((box[0] - 1, box[1]) in state.obstacles)
    stuck_y = (box[1] in [0, state.height-1]) or ((box[0], box[1] + 1) in state.obstacles) or ((box[0], box[1] - 1) in state.obstacles)
    return stuck_x and stuck_y

def edge_no_colinear(box, state):
    box_left, box_right, box_up, box_down = (box[0] == 0), (box[0] == state.width - 1), (box[1] == 0), (box[1] == state.height - 1)
    if box_left:
        left_goals = [storage for storage in state.storage if storage[0] == 0]
        if left_goals == []:
            return True
        left_obs = [obstacle for obstacle in state.obstacle if obstacle[0] == 0]

        left_goals_height = [height[1] for height in left_goals]
        left_obs_height = [height[1] for height in left_obs]

        box_height = box[1]

        #blocked_up

    return False


def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    # Initialize board info
    dim = (state.height, state.width)
    storage = list(state.storage)
    boxes = list(state.boxes)
    robots = list(state.robots)
    obstacles = list(state.obstacles)

    heur_value = 0

    # unfilled boxes and storage
    unfilled_storage = list(set(storage) - set(boxes))
    unstored_boxes = list(set(boxes) - set(storage))

    # check if boxes are stuck (against wall with no colinear goal, against corners)
    for box in unstored_boxes:
        if corner_stuck(box, state):
            return float("inf")
        #if edge_no_colinear(box, state):
            #return float("inf")



    #Use minimal pairs (pair off two sets such that the sum of difference between is minimized)
    # This problem is called least weight minimal pairing, and apparently very hard, my solution is just loop boxes and storage, pull out minimal minimal distance each loop until no more boxes/storage
    # left_boxes, left_storage = unstored_boxes, unfilled_storage
    left_boxes, left_storage = boxes, storage

    while (left_boxes != []) and (left_storage != []):
        '''
        remove_box, remove_store, min_min_dist = None, None, float("inf")
        for box in left_boxes:
            min_dist = float("inf")
            select_store = None

            for store in left_storage:
                dist_box_store = calc_manhattan_distance(box, store)
                if dist_box_store < min_dist:
                    select_store, min_dist = store, dist_box_store
            if min_dist < min_min_dist:
                remove_box, remove_store, min_min_dist = box, select_store, min_dist

        left_boxes, left_storage = list(set(left_boxes) - set([remove_box])), list(set(left_storage) - set([remove_store]))
        heur_value += min_min_dist
        '''

        remove_box, remove_store, max_min_dist = None, None, -1
        for box in left_boxes:
            min_dist = float("inf")
            select_store = None

            for store in left_storage:
                dist_box_store = calc_manhattan_distance(box, store)
                if dist_box_store < min_dist:
                    select_store, min_dist = store, dist_box_store
            if min_dist > max_min_dist:
                remove_box, remove_store, max_min_dist = box, select_store, min_dist

        left_boxes, left_storage = list(set(left_boxes) - set([remove_box])), list(
            set(left_storage) - set([remove_store]))
        heur_value += max_min_dist

        #print(f"Min Pair: box selected: {remove_box}, storage selected: {remove_store}")
        #print(f"heuristic = {heur_value}")

    # Calculate distance from non stored boxes to nearest robot, add to heuristic
    for box in unstored_boxes:
        #print(f'checking box {box}')
        closest_robot, min_dist = None, float("inf")

        for robot in robots:
            #print(f'checking robot {robot}')
            #dist_box_robot = calc_manhattan_distance(box, robot) # consider diagonal moves = 1, allow robot to explore more
            dist_box_robot = calc_diag_manhattan_distance(box, robot) # consider diagonal moves = 1, allow robot to explore more
            # print(f'Diag Dist: {dist_box_robot}')
            if dist_box_robot < min_dist:
                min_dist, closest_robot = dist_box_robot, robot
        #print(f"Box {box} paired with robot {closest_robot}")
        heur_value += min_dist
        #print(f'heuristic = {heur_value}')

    return heur_value


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

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
    return sN.gval + (weight*sN.hval)


def anytime_weighted_astar(initial_state, heur_fn, weight=10., timebound = 5):
    # IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of anytime weighted astar algorithm'''

    # heuristics override
    heur_fn = heur_alternate
    weight = 10

    start_time = current_time = os.times()[0]
    end_time = start_time + timebound
    engine = SearchEngine(strategy='custom', cc_level='full')
    engine.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
    costbound = (float('inf'), float('inf'), float('inf'))
    timebound = end_time - os.times()[0]

    # initial search
    best_path = engine.search(timebound, costbound)[0]
    if not best_path:
        return False
    # costbound = (float('inf'), float('inf'), best_path.gval)
    # If we are searching for paths with lower cost than the current path (let's say g_curr),
    # then given that the heuristic function is admissible,
    # (0 <= g-value <= g_curr) and (g-value + h-value <= g_curr), so (h-value <= g_curr).
    costbound = (best_path.gval, best_path.gval, best_path.gval)
    timebound = end_time - os.times()[0]

    # search again if time is left and frontier is not empty
    while timebound > 0 and not engine.open.empty():
        weight -= (weight - 1.0) / 2
        engine.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
        path = engine.search(timebound, costbound)[0]
        if not path:
            break
        if path.gval < best_path.gval:
            best_path = path
            # costbound = (float('inf'), float('inf'), best_path.gval)
            costbound = (best_path.gval, best_path.gval, best_path.gval)
        timebound = end_time - os.times()[0]

    return best_path


def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime greedy best-first search'''
  return False
