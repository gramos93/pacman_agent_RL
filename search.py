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

def depthFirstSearch(problem):
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
    # Parents tracking list
    explored = list()

    # Frontier keeps track of the branches of the graph
    # We use a stack LIFO for the DFS algorithm (p.85)
    frontier = util.Stack()
    
    node = (problem.getStartState(), "")
    branch = list([node])
    
    while not problem.isGoalState(node[0]):
        if node[0] not in explored:
            for child_state in problem.getSuccessors(node[0]):
                new_branch = list(branch)
                new_branch.append(child_state)
                frontier.push(new_branch)

            explored.append(node[0])
        if frontier.isEmpty():
            # No solution found
            return []

        branch = frontier.pop()
        node = branch[-1]

    solution = [action[1] for action in branch]
    return solution[1:]

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    # Parents tracking dictionary
    explored = list()

    # Frontier keeps track of the branches of the graph
    # We use a stack FIFO for the DFS algorithm (p.81)
    frontier = util.Queue()
    node = (problem.getStartState(), "")
    branch = list([node])
    
    while not problem.isGoalState(node[0]):
        if node[0] not in explored:
            for child_state in problem.getSuccessors(node[0]):
                new_branch = list(branch)
                new_branch.append(child_state)
                frontier.push(new_branch)

            explored.append(node[0])
        if frontier.isEmpty():
            # No solution found
            return []

        branch = frontier.pop()
        node = branch[-1]

    solution = [action[1] for action in branch]
    return solution[1:]

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    # Parents tracking list
    explored = list()
    
    # Frontier keeps track of the branches of the graph 
    # We use a priority queue for the UCS algorithm (p.84)
    frontier = util.PriorityQueue()
    node = (problem.getStartState(), "", 0.)
    branch = list([[node], 0.])
    
    while not problem.isGoalState(node[0]):
        if node[0] not in explored:
            for child_state in problem.getSuccessors(node[0]):
                new_branch = list(branch[0])
                new_branch.append(child_state)
                frontier.push([new_branch, branch[-1] + child_state[-1]], 
                               branch[-1] + child_state[-1])

            explored.append(node[0])

        if frontier.isEmpty():
            # No solution found
            return []
       
        branch = frontier.pop()
        node = branch[0][-1]

    solution = [action[1] for action in branch[0]]
    return solution[1:]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    # Parents tracking list
    explored = list()

    # Frontier keeps track of the branches of the graph
    # We use a priority queue with h(n)+c(n) as sorting key.
    frontier = util.PriorityQueue()
    node = (problem.getStartState(), "", 0.)
    branch = list([[node], 0.])
    
    while not problem.isGoalState(node[0]):
        if node[0] not in explored:
            for child_state in problem.getSuccessors(node[0]):
                h = heuristic(child_state[0], problem)
                new_branch = list(branch[0])
                new_branch.append(child_state)
                frontier.push([new_branch, branch[-1] + child_state[-1]], 
                               h + branch[-1] + child_state[-1])

            explored.append(node[0])

        if frontier.isEmpty():
            # No solution found
            return []
       
        branch = frontier.pop()
        node = branch[0][-1]

    solution = [action[1] for action in branch[0]]
    return solution[1:]

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
