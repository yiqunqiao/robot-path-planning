"""Portable multi-target A* adapter for the Task 2 heuristic.

The public Task 1 base replaces the course-only compiled planner. Search
states include orientation and the set of targets still to visit.
"""
import heapq
import itertools

from environment import Action, Direction
from task1.agent import BaseAgent


class MultiTargetAgent(BaseAgent):
    def a_star(self, start, goals):
        goals = frozenset(tuple(goal) for goal in goals)
        initial = (start[0], start[1], int(self.env.robots[0].direction), goals - {start})
        sequence = itertools.count()
        queue = [(0, next(sequence), 0, initial)]
        costs = {initial: 0}
        parents = {}
        while queue:
            _, _, cost, state = heapq.heappop(queue)
            if cost != costs[state]:
                continue
            x, y, direction, remaining = state
            if not remaining:
                actions = []
                while state in parents:
                    state, action = parents[state]
                    actions.append(action)
                return actions[::-1]
            successors = [
                (Action.ROTATE_CW, x, y, (direction+1) % 4),
                (Action.ROTATE_CCW, x, y, (direction-1) % 4),
            ]
            dx, dy = [(0,-1), (1,0), (0,1), (-1,0)][direction]
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height and self.env.grid[ny,nx] != 1:
                successors.append((Action.MOVE_FORWARD, nx, ny, direction))
            for action, nx, ny, nd in successors:
                todo = remaining - {(nx,ny)}
                successor = (nx,ny,nd,todo)
                new_cost = cost+1
                if new_cost >= costs.get(successor, float("inf")):
                    continue
                costs[successor] = new_cost
                parents[successor] = (state,action)
                observation = {"robots": [(0,nx,ny,Direction(nd),None)]}
                estimate = self.task_heuristic_single_agent(observation, sorted(todo))
                heapq.heappush(queue, (new_cost+estimate, next(sequence), new_cost, successor))
        return []
