"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import sys
from time import sleep

from game import Directions

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def depthFirstSearch(problem):
    path = []
    visited = []
    stack = util.Stack()
    stack.push((problem.getStartState(), path))

    while not stack.is_empty():
        (it, path) = stack.pop()

        if (problem.isGoalState(it)):
            break

        successors = problem.getSuccessors(it)
        for (pos, direction, expand) in successors:
            if pos not in visited:
                visited.append(pos)
                stack.push((pos, path + [direction]))
    return path      


def breadthFirstSearch(problem):
    path = []
    visited = []
    queue = util.Queue()
    queue.enqueue((problem.getStartState(), path))
    
    while not queue.is_empty():
        (it, path) = queue.dequeue()
        
        if problem.isGoalState(it):
            break

        successors = problem.getSuccessors(it)
        for (pos, direction, expand) in successors:
            if pos not in visited:
                visited.append(pos)
                queue.enqueue((pos, path + [direction]))
    return path 


def uniformCostSearch(problem):
    pq = util.PriorityQueue()
    visited = []
    path = []
    visited.append(problem.getStartState())
    pq.update(problem.getStartState(), problem.getCostOfActions(path), path)
    while not pq.isEmpty():
        (pri, it, path) = pq.pop()
        if problem.isGoalState((it)):
            break
        successors = problem.getSuccessors(it)
        for (pos, direction, expand) in successors:
            if pos not in visited:
                visited.append(pos)
                pq.update(pos, pri+expand, path+[direction])
    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(position, problem):
    pac_man, food_pos = problem.getStartState()
    x1, y1 = position[0]
    food_lst = food_pos.asList(key =True)
    food_distance = []
    for (x, y) in food_lst:
        distance = abs( x1 - x ) + abs( y1 - y )
        food_distance.append(distance)
    return min(food_distance)

def canberraHeuristic(position, problem):
    pac_man, food_pos = problem.getStartState()
    x1, y1 = position[0]
    food_lst = food_pos.asList(key =True)
    food_distance = []
    for (x, y) in food_lst:
        distance = float(abs(x1-x) / (abs(x1) + abs(x)) + abs(y1-y) / (abs(y1) + abs(y)))
        food_distance.append(distance)
    return min(food_distance)

def euclideanHeuristic(position, problem):
    pac_man, food_pos = problem.getStartState()
    x1, y1 = position[0]
    food_lst = food_pos.asList(key =True)
    food_distance = []
    for (x, y) in food_lst:
        distance = ((x1 - x) ** 2 + (y1 - y) ** 2) ** 0.5
        food_distance.append(distance.real)
    return min(food_distance)

def chebyshevHeuristic(position, problem):
    pac_man, food_pos = problem.getStartState()
    x1, y1 = position[0]
    food_lst = food_pos.asList(key =True)
    food_distance = []
    for (x, y) in food_lst:
        distance = max(abs(y - y1), abs(x - x1))
        food_distance.append(distance.real)
    #print(food_distance)
    return min(food_distance)


# minkowskiHeuristic is euclideanHeuristic if p = 2 and is manhattanHeuristic if p = 1

# def minkowskiHeuristic(position, problem, p=2):
#     pac_man, food_pos = problem.getStartState()
#     x1, y1 = position[0]
#     food_lst = food_pos.asList(key =True)
#     food_distance = []
#     for (x, y) in food_lst:
#         distance = ((x1 - x) ** p + (y1 - y) ** p) ** (1/p)
#         food_distance.append(distance.real)
#     return min(food_distance)

def aStarSearch(problem, heuristic=euclideanHeuristic):
    # the component of tree is ((state, the path to the state), priority)
    tree = util.PriorityQueueWithFunction(heuristic, problem)
    tree.push(problem.getStartState(), [])

    # store the visited position
    visited = []

    while(not tree.isEmpty()):
        (pri, it, path) = tree.pop()
        if(problem.isGoalState(it)):
            break

        successors = problem.getSuccessors(it)
        for (pos, direction, expand) in successors:
            if(pos not in visited):  # any state has been visited doesn't need to be visited again
                visited.append(pos)
                tree.update(pos, path + [direction])

    return path
   

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch