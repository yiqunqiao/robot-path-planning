import numpy as np
from queue import PriorityQueue
from typing import List, Tuple, Dict, Optional, Set, Any, Callable
from collections import deque
from environment import MultiAgentPathfindingEnv, Robot, Task, Errand, Action, Direction
import random
from dataclasses import dataclass, field
from task1.agent import SinglePathPlanningAgent

def manhattan_heuristic(state: Dict[str, Any], goal: Tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm using Manhattan distance.
    
    Args:
        state: Current state of the environment
        goal: Goal position (x, y)
    
    Returns:
        Estimated cost from current state to goal
    """
    robot_position = state['robots'][0][1:3]  # Assuming single agent for this class
    return abs(robot_position[0] - goal[0]) + abs(robot_position[1] - goal[1])

class SingleTaskSchedulingAgent(SinglePathPlanningAgent):

    def _costs_to_target(self, start: Tuple[int, int], start_direction: int,
                         goal: Tuple[int, int], cache: Dict) -> Dict[int, int]:
        """Return the shortest action cost for each direction at ``goal``."""
        key = (start[0], start[1], int(start_direction), goal[0], goal[1])
        if key in cache:
            return cache[key]

        width = self.env.grid_width
        height = self.env.grid_height
        grid = self.env.grid

        if (goal[0] < 0 or goal[0] >= width or
                goal[1] < 0 or goal[1] >= height or
                grid[goal[1], goal[0]] == 1):
            cache[key] = {}
            return cache[key]

        start_state = (start[0], start[1], int(start_direction))
        frontier = deque([(start_state, 0)])
        visited = {start_state}
        goal_costs = {}
        moves = [(0, -1), (1, 0), (0, 1), (-1, 0)]

        while frontier and len(goal_costs) < 4:
            (x, y, direction), cost = frontier.popleft()

            if (x, y) == goal and direction not in goal_costs:
                goal_costs[direction] = cost

            next_states = [
                (x, y, (direction + 1) % 4),
                (x, y, (direction - 1) % 4)
            ]

            dx, dy = moves[direction]
            next_x, next_y = x + dx, y + dy
            if (0 <= next_x < width and 0 <= next_y < height and
                    grid[next_y, next_x] != 1):
                next_states.append((next_x, next_y, direction))

            for next_state in next_states:
                if next_state not in visited:
                    visited.add(next_state)
                    frontier.append((next_state, cost + 1))

        cache[key] = goal_costs
        return goal_costs

    def _task_options(self, task: Task, start: Tuple[int, int],
                      start_direction: int, path_cache: Dict,
                      task_cache: Dict) -> Tuple[Optional[Tuple[int, int]], Dict[int, int]]:
        """Estimate a task while keeping its errands in their required order."""
        key = (task.id, start[0], start[1], int(start_direction),
               task.current_errand_idx)
        if key in task_cache:
            return task_cache[key]

        errands = task.errands[task.current_errand_idx:]
        if not errands:
            task_cache[key] = (None, {})
            return task_cache[key]

        position = start
        direction_costs = {int(start_direction): 0}

        for errand in errands:
            goal = (errand.target_x, errand.target_y)
            next_costs = {}

            for direction, current_cost in direction_costs.items():
                target_costs = self._costs_to_target(
                    position, direction, goal, path_cache)

                for end_direction, travel_cost in target_costs.items():
                    segment_cost = max(1, travel_cost)
                    total_cost = current_cost + segment_cost
                    if (end_direction not in next_costs or
                            total_cost < next_costs[end_direction]):
                        next_costs[end_direction] = total_cost

            if not next_costs:
                task_cache[key] = (None, {})
                return task_cache[key]

            position = goal
            direction_costs = next_costs

        task_cache[key] = (position, direction_costs)
        return task_cache[key]

    def choose_action(self, obs: np.ndarray) -> Any:
        """
        Choose an action based on the current observation.
        """
        if not hasattr(self, 'current_actions'):
            self.current_actions = []

        robot = self.env.robots[0]  
        current_errand = None
        if robot.task_id is None or robot.task_id < 0:
            unassigned_tasks = [self.env.tasks[i] for i in self.env.get_unassigned_tasks()]
            task_id = self.assign_tasks_single_agent((robot.x, robot.y), unassigned_tasks)
            if task_id != -1:
                current_errand = self.env.tasks[task_id].get_current_errand()
        else:
            task_id = -1
            current_task = self.env.tasks[robot.task_id]
            current_errand = current_task.get_current_errand()

        if current_errand and not self.current_actions:
            goal = (current_errand.target_x, current_errand.target_y)
            start = (robot.x, robot.y)
        

            actions = self.a_star(start, goal, manhattan_heuristic)
            if actions:
                self.current_actions = actions
                action = self.current_actions.pop(0)
                return ([action], [task_id])  
                
        elif self.current_actions:
            action = self.current_actions.pop(0)
            return ([action], [task_id])  
        return ([Action.WAIT], [task_id]) 
    def assign_tasks_single_agent(self, position: Tuple[int, int], available_tasks: List[Task]) -> int:
        """
        Assign tasks to a single robot to minimize total travel time.
        
        Args:
            position: Current position of the robot
            available_tasks: List of tasks to consider for assignment

        Returns:
            id of the assigned task. Use -1 if no task can be assigned.
        """
        if not available_tasks or not self.env.robots:
            return -1

        remaining_time = max(
            0, self.env.max_timesteps - self.env.timestep)
        if remaining_time == 0:
            return -1

        start = (int(position[0]), int(position[1]))
        start_direction = int(self.env.robots[0].direction)
        path_cache = {}
        task_cache = {}
        tasks = []
        for task in available_tasks:
            end_position, costs = self._task_options(
                task, start, start_direction, path_cache, task_cache)
            if (end_position is not None and costs and
                    min(costs.values()) <= remaining_time):
                tasks.append(task)

        if not tasks:
            return -1
        exact_limit = 8
        if len(tasks) <= exact_limit:
            memo = {}

            def search(current_position: Tuple[int, int], direction: int,
                       remaining: Tuple[int, ...], time_left: int):
                key = (current_position, direction, remaining, time_left)
                if key in memo:
                    return memo[key]
                best = (0, 0, -1)

                for local_index in remaining:
                    task = tasks[local_index]
                    end_position, end_costs = self._task_options(
                        task, current_position, direction,
                        path_cache, task_cache)
                    if end_position is None:
                        continue

                    next_remaining = tuple(
                        index for index in remaining if index != local_index)

                    for end_direction, task_cost in end_costs.items():
                        if task_cost > time_left:
                            continue

                        completed, later_cost, _ = search(
                            end_position, end_direction, next_remaining,
                            time_left - task_cost)
                        candidate = (
                            completed + 1,
                            task_cost + later_cost,
                            local_index
                        )

                        if (candidate[0] > best[0] or
                                (candidate[0] == best[0] and
                                 candidate[1] < best[1]) or
                                (candidate[0] == best[0] and
                                 candidate[1] == best[1] and
                                 tasks[candidate[2]].id <
                                 tasks[best[2]].id)):
                            best = candidate

                memo[key] = best
                return best

            task_indices = tuple(range(len(tasks)))
            _, _, first_index = search(
                start, start_direction, task_indices, remaining_time)
            return tasks[first_index].id if first_index != -1 else -1

    
        best_task_id = -1
        best_cost = float('inf')

        for task in tasks:
            _, costs = self._task_options(
                task, start, start_direction, path_cache, task_cache)
            task_cost = min(costs.values())
            if (task_cost < best_cost or
                    (task_cost == best_cost and
                     (best_task_id == -1 or task.id < best_task_id))):
                best_cost = task_cost
                best_task_id = task.id

        return best_task_id

if __name__ == "__main__":
    env = MultiAgentPathfindingEnv(num_robots=1,seed=2) 
    agent = SingleTaskSchedulingAgent(env)
    agent.run(iterations=1, display=True)
  
