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
        left_obs = [obstacle for obstacle in state.obstacles if obstacle[0] == 0]
        if left_obs != []:
            print(state.print_state())
            print(f'Obstacles on Left: {left_obs}')
            left_goals_height = [height[1] for height in left_goals]
            left_obs_height = [height[1] for height in left_obs]
            box_height = box[1]

            if (box_height < min(obstacle for obstacle in left_obs_height if obstacle > box_height)) and (min(obstacle for obstacle in left_obs_height if obstacle > box_height)<min(goal for goal in left_goals_height if goal > box_height)):
                print(
                f"Stuck, Box On Left, Box Height: {box_height}, Goal Heights: {left_goals_height}, Obstacle Heights: {left_obs_height}")
                return True

            if (box_height > max(obstacle for obstacle in left_obs_height if obstacle < box_height)) and (max(obstacle for obstacle in left_obs_height if obstacle < box_height)<max(goal for goal in left_goals_height if goal < box_height)):
                print(
                f"Stuck, Box On Left, Box Height: {box_height}, Goal Heights: {left_goals_height}, Obstacle Heights: {left_obs_height}")
                return True
        return False

    if box_right:
        right_goals = [storage for storage in state.storage if storage[0] == state.width - 1]
        if right_goals == []:
            return True
        right_obs = [obstacle for obstacle in state.obstacles if obstacle[0] == state.width - 1]
        if right_obs != []:
            print(state.print_state())
            print(f'Obstacles on Right: {right_obs}')
            right_goals_height = [height[1] for height in right_goals]
            right_obs_height = [height[1] for height in right_obs]
            box_height = box[1]

            if (box_height < min(obstacle for obstacle in right_obs_height if obstacle > box_height)) and (
                    min(obstacle for obstacle in right_obs_height if obstacle > box_height) < min(
                    goal for goal in right_goals_height if goal > box_height)):
                print(
                    f"Stuck, Box On Right, Box Height: {box_height}, Goal Heights: {right_goals_height}, Obstacle Heights: {right_obs_height}")
                return True

            if (box_height > max(obstacle for obstacle in right_obs_height if obstacle < box_height)) and (
                    max(obstacle for obstacle in right_obs_height if obstacle < box_height) < max(
                    goal for goal in right_goals_height if goal < box_height)):
                print(
                    f"Stuck, Box On Right, Box Height: {box_height}, Goal Heights: {right_goals_height}, Obstacle Heights: {right_obs_height}")
                return True
        return False

    if box_up:
        up_goals = [storage for storage in state.storage if storage[1] == 0]
        if up_goals == []:
            return True
        up_obs = [obstacle for obstacle in state.obstacles if obstacle[1] == 0]
        if up_obs != []:
            print(state.print_state())
            print(f'Obstacles on Up: {up_obs}')
            up_goals_height = [height[0] for height in up_goals]
            up_obs_height = [height[0] for height in up_obs]
            box_height = box[0]

            if (box_height < min(obstacle for obstacle in up_obs_height if obstacle > box_height)) and (
                    min(obstacle for obstacle in up_obs_height if obstacle > box_height) < min(
                goal for goal in up_goals_height if goal > box_height)):
                print(
                    f"Stuck, Box On Up, Box Height: {box_height}, Goal Heights: {up_goals_height}, Obstacle Heights: {up_obs_height}")
                return True

            if (box_height > max(obstacle for obstacle in up_obs_height if obstacle < box_height)) and (
                    max(obstacle for obstacle in up_obs_height if obstacle < box_height) < max(
                goal for goal in up_goals_height if goal < box_height)):
                print(
                    f"Stuck, Box On Right, Box Height: {box_height}, Goal Heights: {up_goals_height}, Obstacle Heights: {up_obs_height}")
                return True
        return False

    if box_down:
        down_goals = [storage for storage in state.storage if storage[1] == state.height - 1]
        if down_goals == []:
            return True
        down_obs = [obstacle for obstacle in state.obstacles if obstacle[1] == state.height - 1]
        if down_obs != []:
            print(state.print_state())
            print(f'Obstacles on Up: {down_obs}')
            down_goals_height = [height[0] for height in down_goals]
            down_obs_height = [height[0] for height in down_obs]
            box_height = box[0]

            if (box_height < min(obstacle for obstacle in down_obs_height if obstacle > box_height)) and (
                    min(obstacle for obstacle in down_obs_height if obstacle > box_height) < min(
                goal for goal in down_goals_height if goal > box_height)):
                print(
                    f"Stuck, Box On Up, Box Height: {box_height}, Goal Heights: {down_goals_height}, Obstacle Heights: {down_obs_height}")
                return True

            if (box_height > max(obstacle for obstacle in down_obs_height if obstacle < box_height)) and (
                    max(obstacle for obstacle in down_obs_height if obstacle < box_height) < max(
                goal for goal in down_goals_height if goal < box_height)):
                print(
                    f"Stuck, Box On Right, Box Height: {box_height}, Goal Heights: {down_goals_height}, Obstacle Heights: {down_obs_height}")
                return True
        return False
    return False

def adj_edge_boxes(state):
    boxes_on_edge = [[],[],[],[]]
    storage = state.storage
    for box in state.boxes:
        box_left, box_right, box_up, box_down = (box[0] == 0), (box[0] == state.width - 1), (box[1] == 0), (box[1] == state.height - 1)
        if box_left:
            boxes_on_edge[0].append(box)
        if box_right:
            boxes_on_edge[1].append(box)
        if box_up:
            boxes_on_edge[2].append(box)
        if box_down:
            boxes_on_edge[3].append(box)
    if len(boxes_on_edge[0]) > 1:
        boxes = sorted(boxes_on_edge[0], key=lambda tup: tup[1])
        for i in range(0,len(boxes)-1):
            if boxes[i][1] +1 == boxes[i+1][1] and not ((boxes[i] in storage) and (boxes[i+1] in storage)):
                return True
    if len(boxes_on_edge[1]) > 1:
        boxes = sorted(boxes_on_edge[1], key=lambda tup: tup[1])
        for i in range(0,len(boxes)-1):
            if boxes[i][1] +1 == boxes[i+1][1] and not ((boxes[i] in storage) and (boxes[i+1] in storage)):
                return True
    if len(boxes_on_edge[2]) > 1:
        boxes = sorted(boxes_on_edge[2], key=lambda tup: tup[0])
        for i in range(0,len(boxes)-1):
            if boxes[i][0] +1 == boxes[i+1][0] and not ((boxes[i] in storage) and (boxes[i+1] in storage)):
                return True
    if len(boxes_on_edge[3]) > 1:
        boxes = sorted(boxes_on_edge[3], key=lambda tup: tup[0])
        for i in range(0,len(boxes)-1):
            if boxes[i][0] +1 == boxes[i+1][0] and not ((boxes[i] in storage) and (boxes[i+1] in storage)):
                return True
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
    if len(unstored_boxes) == 0:
        return 0

    # check if boxes are stuck (against wall with no colinear goal, against corners)
    for box in unstored_boxes:
        if corner_stuck(box, state):
            return float("inf")
        if edge_no_colinear(box, state):
            return float("inf")

    if adj_edge_boxes(state):
        return float("inf")

    # This problem is called least weight minimal pairing, and apparently very hard, my solution is just loop boxes and storage, pull out minimal minimal distance each loop until no more boxes/storage
    # left_boxes, left_storage = unstored_boxes, unfilled_storage
    left_boxes, left_storage = unstored_boxes, unfilled_storage

    # Penalize for not storing boxes
    #heur_value += 5*len(unstored_boxes)

    while (left_boxes != []) and (left_storage != []):

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

    # Calculate distance from non stored boxes to nearest robot, add to heuristic
    for box in unstored_boxes:
        closest_robot, min_dist = None, float("inf")

        for robot in robots:
            dist_box_robot = calc_manhattan_distance(box, robot) # consider diagonal moves = 1, allow robot to explore more
            #dist_box_robot = calc_diag_manhattan_distance(box, robot) # consider diagonal moves = 1, allow robot to explore more
            # print(f'Diag Dist: {dist_box_robot}')
            if dist_box_robot < min_dist:
                min_dist, closest_robot = dist_box_robot, robot
        heur_value += min_dist

    #Keep Robots Close To Unstored Boxes
    max_robot_dist_to_box = -1
    for robot in robots:
        min_dist = float("inf")
        for box in unstored_boxes:
            dist_box_robot = calc_manhattan_distance(box, robot) # consider diagonal moves = 1, allow robot to explore more
            if dist_box_robot < min_dist:
                min_dist = dist_box_robot
        if max_robot_dist_to_box < min_dist:
            max_robot_dist_to_box = min_dist

    heur_value += max(8, max_robot_dist_to_box)

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

    # overrides
    heur_fn = heur_alternate
    weight = 2

    start_time = os.times()[0]
    end_time = start_time + timebound
    engine = SearchEngine(strategy='custom', cc_level='full')
    engine.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
    cost = (float('inf'), float('inf'), float('inf'))
    timeout = end_time - os.times()[0]

    best_path = engine.search(timeout, cost)[0]
    if not best_path:
        return False

    cost = (best_path.gval, best_path.gval, best_path.gval)
    timeout = end_time - os.times()[0]

    # search again if time is left, frontier isn't empty and cut the weight in half
    while timeout > 0 and not engine.open.empty():
        weight = weight / 2
        engine.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
        path = engine.search(timebound, cost)[0]
        if not path:
            break
        if path.gval < best_path.gval:
            best_path = path
            cost = (best_path.gval, best_path.gval, best_path.gval)
        timeout = end_time - os.times()[0]

    return best_path


def anytime_gbfs(initial_state, heur_fn, timebound = 10):
    #IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of anytime greedy best-first search'''


    # override default parameters
    heur_fn = heur_alternate

    # initialization
    start_time = current_time = os.times()[0]
    end_time = start_time + timebound
    engine = SearchEngine(strategy='best_first', cc_level='full')
    engine.init_search(initial_state, sokoban_goal_state, heur_fn)
    cost = (float("inf"), float("inf"), float("inf"))
    timeout = end_time - os.times()[0]

    # initial search
    best_path = engine.search(timeout, cost)[0]
    if not best_path:
        return False

    cost = (best_path.gval, best_path.gval, best_path.gval)
    timeout = end_time - os.times()[0]

    # search again if time is left and frontier is not empty
    while timeout > 0 and not engine.open.empty():
        path = engine.search(timeout, cost)[0]
        if not path:
            break
        if path.gval < best_path.gval:
            best_path = path
            cost = (best_path.gval, best_path.gval, best_path.gval)
        timeout = end_time - os.times()[0]

    return best_path

