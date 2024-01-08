# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # stack -> last in first out
    stack = util.Stack()
    expanded = set()
    stack.push((problem.getStartState(),[]))
    
    point = problem.getStartState()
    action = []
    goal_point = ()
    find = 0

    if problem.isGoalState(point) == True:
        return action

    while not find:
        for p in problem.getSuccessors(point):
            expanded.add(point)
            if p[0] not in expanded:
                stack.push([p[0],action + [p[1]]])
        next_point_find = 0
        while not stack.isEmpty() and next_point_find == 0:
            next = stack.pop()
            if next[0] not in expanded:
                point = next[0]
                action = next[1]
                next_point_find = 1

        if problem.isGoalState(point):
            goal_point = point
            find = 1
            return action

    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # queue -> first in first out
    queue = util.Queue()
    expanded = set()
    queue.push((problem.getStartState(),[]))
    
    point = problem.getStartState()
    action = []
    goal_point = ()
    find = 0

    if problem.isGoalState(point) == True:
        return action

    while not find:
        for p in problem.getSuccessors(point):
            expanded.add(point)
            if p[0] not in expanded:
                queue.push([p[0],action + [p[1]]])
        next_point_find = 0
        while not queue.isEmpty() and next_point_find == 0:
            next = queue.pop()
            if next[0] not in expanded:
                point = next[0]
                action = next[1]
                next_point_find = 1

        if problem.isGoalState(point):
            goal_point = point
            find = 1
            return action


    util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # queue cost
    queue = util.PriorityQueue()
    expanded = set()
    
    point = problem.getStartState()
    action = []
    goal_point = ()
    find = 0
    cost = 0
    if problem.isGoalState(point) == True:
        return action
    while not find:
        for p in problem.getSuccessors(point):
            expanded.add(point)
            if p[0] not in expanded:
                queue.update([p[0],action + [p[1]], p[2] +cost], p[2]+cost)
        next_point_find = 0
        while not queue.isEmpty() and next_point_find == 0:
            next = queue.pop()
            if next[0] not in expanded:
                point = next[0]
                action = next[1]
                cost = next[2]
                next_point_find = 1

        if problem.isGoalState(point):
            goal_point = point
            find = 1
            return action

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    queue = util.PriorityQueue()
    expanded = []
    
    point = problem.getStartState()
    action = []
    costs = {}
    costs[point] = 50000000000
    goal_point = ()
    find = 0
    find_goal = 0
    goal_action = []
    cost = 0
    count = 0
    goal_cost = 50000000000
    if problem.isGoalState(point) == True:
        return action
    
    while not find:
        for p in problem.getSuccessors(point):
            expanded.append(point)
            if problem.isGoalState(p[0]):
                if cost + heuristic(p[0], problem) +p[2] <= goal_cost:
                    find_goal = 1
                    goal_point = p[0]
                    goal_action = action+ [p[1]]
                    goal_cost = cost + heuristic(p[0], problem) +p[2]
            if p[0] not in costs:
                queue.update([p[0],action + [p[1]], cost+p[2]], heuristic(p[0], problem)+cost+p[2])
                costs[p[0]] =  heuristic(p[0], problem)+cost+p[2]
            elif costs[p[0]] > cost + heuristic(p[0], problem) + p[2]:
                queue.update([p[0],action + [p[1]], heuristic(p[0], problem)+cost+p[2]], heuristic(p[0], problem)+cost+p[2])
        next_point_find = 0
        while not queue.isEmpty() and next_point_find == 0:
            next = queue.pop()
            if next[0] not in expanded:
                point = next[0]
                action = next[1]
                cost = next[2]
                next_point_find = 1
        if problem.isGoalState(point):
            if goal_cost < cost:
                return goal_action
            else:
                goal_point = point
                find = 1
                return action
        if find_goal == 1 and goal_cost < cost:
            return goal_action
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
