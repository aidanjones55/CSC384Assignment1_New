#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import csv  # for csv generation
import sys  # for 'inf' representation as sys.maxsize
import copy  # for Munkres import
from typing import Union, NewType, Sequence, Tuple, Optional, Callable  # for Munkres import
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems


def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True


def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    total_dist = 0
    for box in state.boxes:
        if box in state.storage:
            continue
        dist = float('inf')
        for storage in state.storage:
            dist = min(dist, abs(box[0] - storage[0]) + abs(box[1] - storage[1]))
        total_dist += dist
    return total_dist


# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count


def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    unassigned_storages = list(state.storage)  # cast the tuple of storages into a mutable list
    h_val = 0

    for box in state.boxes:
        # if a box is in a deadlock position, return max
        if (is_corner_deadlock(state, box) and box not in state.storage) or is_edge_deadlock(state, box):
            return float('inf')

        dist_assigned = float('inf')
        storage_assigned = None

        for storage in unassigned_storages:
            if is_edge_deadlock(state, box, storage):
                continue
            dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
            if dist < dist_assigned:
                dist_assigned = dist
                storage_assigned = storage

        # if a storage is not assigned, return max
        if not storage_assigned:
            return float('inf')

        h_val += dist_assigned
        unassigned_storages.remove(storage_assigned)

    ### an attempt to give incentives to move robots closer to boxes if none of them have been stored yet
    # is_storage_not_empty = False
    # for box in state.boxes:
    #   if box in state.storage:
    #     is_storage_not_empty = True
    #     break
    # h_val += (not is_storage_not_empty) * min_dist_between_robots_and_boxes(state.robots, state.boxes)
    # h_val += (not is_storage_not_empty) * sub_heuristic_start_macro(state)

    # sub_heuristic_goal_macro gives incentives to push a box into a corner storage_assigned
    # this helps when there are storage points clustered around corners
    h_val = max(h_val + sub_heuristic_goal_macro(state), 0)

    return h_val


### HELPER FUNCTIONS FOR heur_alternate
def assign_dist_metric(state, box, storage):
    # check_left_wall = (box[0] == 0)
    # check_right_wall = (box[0] == state.width-1)
    # check_up_wall = (box[1] == 0)
    # check_down_wall = (box[1] == state.height-1)

    # # if the box is not next to a wall, return Manhattan distance between the box and the storage
    # if not (check_left_wall or check_right_wall or check_up_wall or check_down_wall):
    #   return abs(box[0] - storage[0]) + abs(box[1] - storage[1])

    # # else, check if the box and the storage are on the same wall
    # if (storage[0] == 0 and check_left_wall) or (storage[0] == state.width-1 and check_left_wall) or (storage[1] == 0 and check_up_wall) or (storage[1] == state.height-1 and check_down_wall):
    # # if they are, return Manhattan distance between the box and the storage
    #   return abs(box[0] - storage[0]) + abs(box[1] - storage[1])
    # # else, return a very large number
    # return sys.maxsize
    return abs(box[0] - storage[0]) + abs(box[1] - storage[1])


def is_edge_deadlock(state: SokobanState, box: tuple, storage=None) -> bool:
    """
    Detect a deadlock if the box is adjacent in one of x or y direction to wall or an obstacle.
    Returns True if it is the case.

    @param SokobanState state: A Sokoban state
    @param tuple box: Coordinate of the box
    @param optional tuple storage (default: None): Coordinate of the storage
    @rtype: bool
    """

    if not storage:
        check_left_wall = (box[0] == 0 and box[0] not in (storage[0] for storage in state.storage))
        check_right_wall = (box[0] == state.width - 1 and box[0] not in (storage[0] for storage in state.storage))
        check_up_wall = (box[1] == 0 and box[1] not in (storage[1] for storage in state.storage))
        check_down_wall = (box[1] == state.height - 1 and box[1] not in (storage[1] for storage in state.storage))
        return check_left_wall or check_right_wall or check_up_wall or check_down_wall
    check_left_wall = (box[0] == 0 and storage[0] != 0)
    check_right_wall = (box[0] == state.width - 1 and storage[0] != state.width - 1)
    check_up_wall = (box[1] == 0 and storage[1] != 0)
    check_down_wall = (box[1] == state.height - 1 and storage[1] != state.height - 1)
    return check_left_wall or check_right_wall or check_up_wall or check_down_wall


def is_corner_deadlock(state: SokobanState, box: tuple) -> bool:
    """
    Detect a deadlock if the box is adjacent in both x and y direction to wall or an obstacle.
    Returns True if it is the case.

    @param SokobanState state: A Sokoban state
    @param tuple box: Coordinate of the box
    @rtype: bool
    """

    return is_immovable_x(state, box) and is_immovable_y(state, box)


def is_immovable_x(state: SokobanState, obj: tuple) -> bool:
    return (obj[0] == 0 or obj[0] == state.width - 1) or ((obj[0] + 1, obj[1]) in state.obstacles) or (
                (obj[0] - 1, obj[1]) in state.obstacles)


def is_immovable_y(state: SokobanState, obj: tuple) -> bool:
    return (obj[1] == 0 or obj[1] == state.height - 1) or ((obj[0], obj[1] + 1) in state.obstacles) or (
                (obj[0], obj[1] - 1) in state.obstacles)


def min_dist_between_robots_and_boxes(robots, boxes):
    min_dist = float('inf')
    for robot in robots:
        for box in boxes:
            min_dist = min(min_dist, abs(box[0] - robot[0]) + abs(box[1] - robot[1]))
    return min_dist


def sub_heuristic_goal_macro(state):
    sub_h_val = 0
    stored_boxes = tuple(set(state.boxes) & set(state.storage))
    for box in stored_boxes:
        sub_h_val -= (is_immovable_x(state, box) and is_immovable_y(state, box))
    return sub_h_val


def sub_heuristic_start_macro(state):
    total_dist = 0
    for box in state.boxes:
        if box in state.storage:
            continue
        dist = float('inf')
        for robot in state.robots:
            dist = min(dist, abs(box[0] - robot[0]) + abs(box[1] - robot[1]))
        total_dist += dist
    return total_dist


def sort_boxes(state, box):
    robots_midpoint = tuple([sum(x) / len(state.robots) for x in zip(*state.robots)])
    dist = abs(box[0] - robots_midpoint[0]) + abs(box[1] - robots_midpoint[1])
    return dist


#######################################


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)
    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    # Many searches will explore nodes (or states) that are ordered by their f-value.
    # For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    # You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    # The function must return a numeric f-value.
    # The value will determine your state's position on the Frontier list during a 'custom' search.
    # You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    return sN.gval + weight * sN.hval


def fval_function_XUP(sN, weight):
    # IMPLEMENT
    """
    Another custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the XUP form of weighted A*
    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    return (1 / (2 * weight)) * (
                sN.gval + sN.hval + ((sN.gval + sN.hval) ** 2 + 4 * weight * (weight - 1) * sN.hval ** 2) ** 0.5)


def fval_function_XDP(sN, weight):
    # IMPLEMENT
    """
    A third custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the XDP form of weighted A*
    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    return (1 / (2 * weight)) * (sN.gval + (2 * weight - 1) * sN.hval + (
                (sN.gval - sN.hval) ** 2 + 4 * weight * sN.gval * sN.hval) ** 0.5)


def compare_weighted_astars():
    # IMPLEMENT
    '''Compares various different implementations of A* that use different f-value functions'''
    '''INPUT: None'''
    '''OUTPUT: None'''
    """
    This function should generate a CSV file (comparison.csv) that contains statistics from
    4 varieties of A* search on 3 practice problems.  The four varieties of A* are as follows:
    Standard A* (Variant #1), Weighted A*  (Variant #2),  Weighted A* XUP (Variant #3) and Weighted A* XDP  (Variant #4).  
    Format each line in your your output CSV file as follows:
    A,B,C,D,E,F
    where
    A is the number of the problem being solved (0,1 or 2)
    B is the A* variant being used (1,2,3 or 4)
    C is the weight being used (2,3,4 or 5)
    D is the number of paths extracted from the Frontier (or expanded) during the search
    E is the number of paths generated by the successor function during the search
    F is the overall solution cost    
    Note that you will submit your CSV file (comparison.csv) with your code
    """

    with open('comparison.csv', mode='w') as datafile:
        data_writer = csv.writer(datafile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['Problem', 'A* variant', 'Weight', 'Extracted paths', 'Generated paths', 'Overall cost'])
        for i in range(0, 3):
            problem = PROBLEMS[i]
            engine = SearchEngine(strategy='astar', cc_level='full')
            engine.init_search(problem, sokoban_goal_state, heur_manhattan_distance)
            path, stats = engine.search()
            data_writer.writerow([i, 1, 1, stats.states_expanded, stats.states_generated, path.gval])
            for weight in [2, 3, 4, 5]:
                engine = SearchEngine(strategy='custom', cc_level='full')
                for func, func_val in [(fval_function, 2), (fval_function_XUP, 3), (fval_function_XDP, 4)]:
                    engine.init_search(problem, sokoban_goal_state, heur_manhattan_distance,
                                       (lambda sN: func(sN, weight)))
                    path, stats = engine.search()
                    data_writer.writerow(
                        [i, func_val, weight, stats.states_expanded, stats.states_generated, path.gval])


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound=10):
    # IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    # override default parameters
    heur_fn = heur_alternate
    # weight = 10.0

    # initialization
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


def anytime_gbfs(initial_state, heur_fn, timebound=10):
    # IMPLEMENT
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
        path = engine.search(timebound, costbound)[0]
        if not path:
            break
        if path.gval < best_path.gval:
            best_path = path
            # costbound = (best_path.gval, float('inf'), float('inf'))
            costbound = (best_path.gval, best_path.gval, best_path.gval)
        timebound = end_time - os.times()[0]

    return best_path


#############################################################################
# I have experimented with Hungarian-Munkres algorithm for the heuristic function.
# While it was able to solve some problems that weren't solvable before, it took too long in many cases.
#############################################################################
def heur_alternate_munkres(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    # Areas of improvements from vanilla Manhattan distance metric:
    # 1. Instead of calculating distances between boxes and their closest storage points,
    #    think of it as a maximum-weight matching problem of a bipartite graph.
    #    This can be solved using Hungarian (Munkres) algorithm with O(n^3) runtime.
    # 2. Detect deadlocks and assign 'inf' values to them.
    #    This prunes many paths during the search.
    # 3. Calculate the minimum distance between a box and a robot.
    #    This provides more layers in the frontier.

    for box in (box for box in state.boxes if box not in state.storage):
        # prune this state if there exists a corner deadlock where it is not a stroage ponit
        if (is_corner_deadlock(state, box) and box not in state.storage) or is_edge_deadlock(state, box):
            return float('inf')

    dist_matrix = []
    for box in state.boxes:
        dist_list = []
        for storage in state.storage:
            dist_list.append(assign_dist_metric(state, box, storage))
        dist_matrix.append(dist_list)

    m = Munkres()
    try:
        indexes = m.compute(dist_matrix)
    except:
        return float('inf')

    h_val = 0
    for r, c in indexes:
        x = dist_matrix[r][c]
        h_val += x

    # h_val = heur_manhattan_distance(state)
    h_val += min_dist_between_robots_and_boxes(state.robots, state.boxes)
    h_val = max(h_val + sub_heuristic_goal_macro(state), 0)
    return h_val


#############################################################################
# Munkres Library
# Codes below are not written by me.
# Citation: https://github.com/bmc/munkres
#############################################################################

"""
Introduction
============
The Munkres module provides an implementation of the Munkres algorithm
(also called the Hungarian algorithm or the Kuhn-Munkres algorithm),
useful for solving the Assignment Problem.
For complete usage documentation, see: https://software.clapper.org/munkres/
"""

# __docformat__ = 'markdown'

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------

# See the beginning of the file for Munkres's required library

# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------

# __all__     = ['Munkres', 'make_cost_matrix', 'DISALLOWED']

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

AnyNum = NewType('AnyNum', Union[int, float])
Matrix = NewType('Matrix', Sequence[Sequence[AnyNum]])


# Info about the module
# __version__   = "1.1.4"
# __author__    = "Brian Clapper, bmc@clapper.org"
# __url__       = "https://software.clapper.org/munkres/"
# __copyright__ = "(c) 2008-2020 Brian M. Clapper"
# __license__   = "Apache Software License"

# Constants
class DISALLOWED_OBJ(object):
    pass


DISALLOWED = DISALLOWED_OBJ()
DISALLOWED_PRINTVAL = "D"


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class UnsolvableMatrix(Exception):
    """
    Exception raised for unsolvable matrices
    """
    pass


# ---------------------------------------------------------------------------
# Classes
# ---------------------------------------------------------------------------

class Munkres:
    """
    Calculate the Munkres solution to the classical assignment problem.
    See the module documentation for usage.
    """

    def __init__(self):
        """Create a new instance"""
        self.C = None
        self.row_covered = []
        self.col_covered = []
        self.n = 0
        self.Z0_r = 0
        self.Z0_c = 0
        self.marked = None
        self.path = None

    def pad_matrix(self, matrix: Matrix, pad_value: int = 0) -> Matrix:
        """
        Pad a possibly non-square matrix to make it square.
        **Parameters**
        - `matrix` (list of lists of numbers): matrix to pad
        - `pad_value` (`int`): value to use to pad the matrix
        **Returns**
        a new, possibly padded, matrix
        """
        max_columns = 0
        total_rows = len(matrix)

        for row in matrix:
            max_columns = max(max_columns, len(row))

        total_rows = max(max_columns, total_rows)

        new_matrix = []
        for row in matrix:
            row_len = len(row)
            new_row = row[:]
            if total_rows > row_len:
                # Row too short. Pad it.
                new_row += [pad_value] * (total_rows - row_len)
            new_matrix += [new_row]

        while len(new_matrix) < total_rows:
            new_matrix += [[pad_value] * total_rows]

        return new_matrix

    def compute(self, cost_matrix: Matrix) -> Sequence[Tuple[int, int]]:
        """
        Compute the indexes for the lowest-cost pairings between rows and
        columns in the database. Returns a list of `(row, column)` tuples
        that can be used to traverse the matrix.
        **WARNING**: This code handles square and rectangular matrices. It
        does *not* handle irregular matrices.
        **Parameters**
        - `cost_matrix` (list of lists of numbers): The cost matrix. If this
          cost matrix is not square, it will be padded with zeros, via a call
          to `pad_matrix()`. (This method does *not* modify the caller's
          matrix. It operates on a copy of the matrix.)
        **Returns**
        A list of `(row, column)` tuples that describe the lowest cost path
        through the matrix
        """
        self.C = self.pad_matrix(cost_matrix)
        self.n = len(self.C)
        self.original_length = len(cost_matrix)
        self.original_width = len(cost_matrix[0])
        self.row_covered = [False for i in range(self.n)]
        self.col_covered = [False for i in range(self.n)]
        self.Z0_r = 0
        self.Z0_c = 0
        self.path = self.__make_matrix(self.n * 2, 0)
        self.marked = self.__make_matrix(self.n, 0)

        done = False
        step = 1

        steps = {1: self.__step1,
                 2: self.__step2,
                 3: self.__step3,
                 4: self.__step4,
                 5: self.__step5,
                 6: self.__step6}

        while not done:
            try:
                func = steps[step]
                step = func()
            except KeyError:
                done = True

        # Look for the starred columns
        results = []
        for i in range(self.original_length):
            for j in range(self.original_width):
                if self.marked[i][j] == 1:
                    results += [(i, j)]

        return results

    def __copy_matrix(self, matrix: Matrix) -> Matrix:
        """Return an exact copy of the supplied matrix"""
        return copy.deepcopy(matrix)

    def __make_matrix(self, n: int, val: AnyNum) -> Matrix:
        """Create an *n*x*n* matrix, populating it with the specific value."""
        matrix = []
        for i in range(n):
            matrix += [[val for j in range(n)]]
        return matrix

    def __step1(self) -> int:
        """
        For each row of the matrix, find the smallest element and
        subtract it from every element in its row. Go to Step 2.
        """
        C = self.C
        n = self.n
        for i in range(n):
            vals = [x for x in self.C[i] if x is not DISALLOWED]
            if len(vals) == 0:
                # All values in this row are DISALLOWED. This matrix is
                # unsolvable.
                raise UnsolvableMatrix(
                    "Row {0} is entirely DISALLOWED.".format(i)
                )
            minval = min(vals)
            # Find the minimum value for this row and subtract that minimum
            # from every element in the row.
            for j in range(n):
                if self.C[i][j] is not DISALLOWED:
                    self.C[i][j] -= minval
        return 2

    def __step2(self) -> int:
        """
        Find a zero (Z) in the resulting matrix. If there is no starred
        zero in its row or column, star Z. Repeat for each element in the
        matrix. Go to Step 3.
        """
        n = self.n
        for i in range(n):
            for j in range(n):
                if (self.C[i][j] == 0) and \
                        (not self.col_covered[j]) and \
                        (not self.row_covered[i]):
                    self.marked[i][j] = 1
                    self.col_covered[j] = True
                    self.row_covered[i] = True
                    break

        self.__clear_covers()
        return 3

    def __step3(self) -> int:
        """
        Cover each column containing a starred zero. If K columns are
        covered, the starred zeros describe a complete set of unique
        assignments. In this case, Go to DONE, otherwise, Go to Step 4.
        """
        n = self.n
        count = 0
        for i in range(n):
            for j in range(n):
                if self.marked[i][j] == 1 and not self.col_covered[j]:
                    self.col_covered[j] = True
                    count += 1

        if count >= n:
            step = 7  # done
        else:
            step = 4

        return step

    def __step4(self) -> int:
        """
        Find a noncovered zero and prime it. If there is no starred zero
        in the row containing this primed zero, Go to Step 5. Otherwise,
        cover this row and uncover the column containing the starred
        zero. Continue in this manner until there are no uncovered zeros
        left. Save the smallest uncovered value and Go to Step 6.
        """
        step = 0
        done = False
        row = 0
        col = 0
        star_col = -1
        while not done:
            (row, col) = self.__find_a_zero(row, col)
            if row < 0:
                done = True
                step = 6
            else:
                self.marked[row][col] = 2
                star_col = self.__find_star_in_row(row)
                if star_col >= 0:
                    col = star_col
                    self.row_covered[row] = True
                    self.col_covered[col] = False
                else:
                    done = True
                    self.Z0_r = row
                    self.Z0_c = col
                    step = 5

        return step

    def __step5(self) -> int:
        """
        Construct a series of alternating primed and starred zeros as
        follows. Let Z0 represent the uncovered primed zero found in Step 4.
        Let Z1 denote the starred zero in the column of Z0 (if any).
        Let Z2 denote the primed zero in the row of Z1 (there will always
        be one). Continue until the series terminates at a primed zero
        that has no starred zero in its column. Unstar each starred zero
        of the series, star each primed zero of the series, erase all
        primes and uncover every line in the matrix. Return to Step 3
        """
        count = 0
        path = self.path
        path[count][0] = self.Z0_r
        path[count][1] = self.Z0_c
        done = False
        while not done:
            row = self.__find_star_in_col(path[count][1])
            if row >= 0:
                count += 1
                path[count][0] = row
                path[count][1] = path[count - 1][1]
            else:
                done = True

            if not done:
                col = self.__find_prime_in_row(path[count][0])
                count += 1
                path[count][0] = path[count - 1][0]
                path[count][1] = col

        self.__convert_path(path, count)
        self.__clear_covers()
        self.__erase_primes()
        return 3

    def __step6(self) -> int:
        """
        Add the value found in Step 4 to every element of each covered
        row, and subtract it from every element of each uncovered column.
        Return to Step 4 without altering any stars, primes, or covered
        lines.
        """
        minval = self.__find_smallest()
        events = 0  # track actual changes to matrix
        for i in range(self.n):
            for j in range(self.n):
                if self.C[i][j] is DISALLOWED:
                    continue
                if self.row_covered[i]:
                    self.C[i][j] += minval
                    events += 1
                if not self.col_covered[j]:
                    self.C[i][j] -= minval
                    events += 1
                if self.row_covered[i] and not self.col_covered[j]:
                    events -= 2  # change reversed, no real difference
        if (events == 0):
            raise UnsolvableMatrix("Matrix cannot be solved!")
        return 4

    def __find_smallest(self) -> AnyNum:
        """Find the smallest uncovered value in the matrix."""
        minval = sys.maxsize
        for i in range(self.n):
            for j in range(self.n):
                if (not self.row_covered[i]) and (not self.col_covered[j]):
                    if self.C[i][j] is not DISALLOWED and minval > self.C[i][j]:
                        minval = self.C[i][j]
        return minval

    def __find_a_zero(self, i0: int = 0, j0: int = 0) -> Tuple[int, int]:
        """Find the first uncovered element with value 0"""
        row = -1
        col = -1
        i = i0
        n = self.n
        done = False

        while not done:
            j = j0
            while True:
                if (self.C[i][j] == 0) and \
                        (not self.row_covered[i]) and \
                        (not self.col_covered[j]):
                    row = i
                    col = j
                    done = True
                j = (j + 1) % n
                if j == j0:
                    break
            i = (i + 1) % n
            if i == i0:
                done = True

        return (row, col)

    def __find_star_in_row(self, row: Sequence[AnyNum]) -> int:
        """
        Find the first starred element in the specified row. Returns
        the column index, or -1 if no starred element was found.
        """
        col = -1
        for j in range(self.n):
            if self.marked[row][j] == 1:
                col = j
                break

        return col

    def __find_star_in_col(self, col: Sequence[AnyNum]) -> int:
        """
        Find the first starred element in the specified row. Returns
        the row index, or -1 if no starred element was found.
        """
        row = -1
        for i in range(self.n):
            if self.marked[i][col] == 1:
                row = i
                break

        return row

    def __find_prime_in_row(self, row) -> int:
        """
        Find the first prime element in the specified row. Returns
        the column index, or -1 if no starred element was found.
        """
        col = -1
        for j in range(self.n):
            if self.marked[row][j] == 2:
                col = j
                break

        return col

    def __convert_path(self,
                       path: Sequence[Sequence[int]],
                       count: int) -> None:
        for i in range(count + 1):
            if self.marked[path[i][0]][path[i][1]] == 1:
                self.marked[path[i][0]][path[i][1]] = 0
            else:
                self.marked[path[i][0]][path[i][1]] = 1

    def __clear_covers(self) -> None:
        """Clear all covered matrix cells"""
        for i in range(self.n):
            self.row_covered[i] = False
            self.col_covered[i] = False

    def __erase_primes(self) -> None:
        """Erase all prime markings"""
        for i in range(self.n):
            for j in range(self.n):
                if self.marked[i][j] == 2:
                    self.marked[i][j] = 0


# ---------------------------------------------------------------------------
# Functions
# ---------------------------------------------------------------------------

def make_cost_matrix(
        profit_matrix: Matrix,
        inversion_function: Optional[Callable[[AnyNum], AnyNum]] = None
) -> Matrix:
    """
    Create a cost matrix from a profit matrix by calling `inversion_function()`
    to invert each value. The inversion function must take one numeric argument
    (of any type) and return another numeric argument which is presumed to be
    the cost inverse of the original profit value. If the inversion function
    is not provided, a given cell's inverted value is calculated as
    `max(matrix) - value`.
    This is a static method. Call it like this:
        from munkres import Munkres
        cost_matrix = Munkres.make_cost_matrix(matrix, inversion_func)
    For example:
        from munkres import Munkres
        cost_matrix = Munkres.make_cost_matrix(matrix, lambda x : sys.maxsize - x)
    **Parameters**
    - `profit_matrix` (list of lists of numbers): The matrix to convert from
       profit to cost values.
    - `inversion_function` (`function`): The function to use to invert each
       entry in the profit matrix.
    **Returns**
    A new matrix representing the inversion of `profix_matrix`.
    """
    if not inversion_function:
        maximum = max(max(row) for row in profit_matrix)
        inversion_function = lambda x: maximum - x

    cost_matrix = []
    for row in profit_matrix:
        cost_matrix.append([inversion_function(value) for value in row])
    return cost_matrix


def print_matrix(matrix: Matrix, msg: Optional[str] = None) -> None:
    """
    Convenience function: Displays the contents of a matrix.
    **Parameters**
    - `matrix` (list of lists of numbers): The matrix to print
    - `msg` (`str`): Optional message to print before displaying the matrix
    """
    import math

    if msg is not None:
        print(msg)

    # Calculate the appropriate format width.
    width = 0
    for row in matrix:
        for val in row:
            if val is DISALLOWED:
                val = DISALLOWED_PRINTVAL
            width = max(width, len(str(val)))

    # Make the format string
    format = ('%%%d' % width)

    # Print the matrix
    for row in matrix:
        sep = '['
        for val in row:
            if val is DISALLOWED:
                val = DISALLOWED_PRINTVAL
            formatted = ((format + 's') % val)
            sys.stdout.write(sep + formatted)
            sep = ', '
        sys.stdout.write(']\n')

## MINE ##

#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files






#SOKOBAN HEURISTICS