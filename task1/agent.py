
import numpy as np
from queue import PriorityQueue
from typing import List, Tuple, Dict, Optional, Set, Any, Callable
from collections import deque
from environment import MultiAgentPathfindingEnv, Robot, Task, Errand, Action, Direction
import random
from dataclasses import dataclass, field

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

class BaseAgent:    
    """
    Base class for multi-agent pathfinding agents.
    
    This class provides basic functionality and should not be instantiated directly.
    Instead, use the PathfindingAgent subclass for actual implementations.
    """
    def __init__(self, env: MultiAgentPathfindingEnv):
        """
        Initialize the base agent with the environment.
        
        Args:
            env: The multi-agent pathfinding environment
        """
        self.env = env
        self.grid_width = env.grid_width
        self.grid_height = env.grid_height
        self.current_actions = []

    def run(self, iterations: int = 10, display: bool = False):
        """
        Run the agent's main loop.
        """
        for i in range(iterations):
            if i>0:
                obs,info = self.env.reset()
            else:
                obs = self.env._get_observation()
           
            terminated = False
            truncated = False
            while not terminated and not truncated:
                if display:
                    self.env.render()
                # Choose an action
                action = self.choose_action(obs)

                # Take a step in the environment
                obs, reward, terminated, truncated, info = self.env.step(action)
        
        return self.env.completed_tasks

    def choose_action(self, obs: np.ndarray) -> Any:
        """
        Choose an action based on the current observation.
        """
        # Not implemented in the base class
        raise NotImplementedError("This method should be overridden by subclasses.")

class SinglePathPlanningAgent(BaseAgent):
    """
    Agent for solving multi-agent pathfinding problems.
    
    This class provides a scaffold for implementing algorithms of increasing complexity.
    Students should implement methods in the order they appear.
    """
 
class SinglePathPlanningAgent(BaseAgent):
    """
    Agent for solving single-agent pathfinding problems.
    
    """
    def choose_action(self, obs: np.ndarray) -> Any:
        """
        Choose an action based on the current observation.
        """
        # Check if agent has an assigned task
        robot = self.env.robots[0]  # Assuming single agent for this class
        current_errand = None
        task_id = -1
        
        if robot.task_id is None or robot.task_id < 0:
            # No current task, assign the first pending task
            available_tasks = self.env.get_unassigned_tasks()
            if available_tasks:
                task_id = available_tasks[0]
                task = self.env.tasks[task_id]
                current_errand = task.get_current_errand()
        else:
            task_id = -1
            current_errand = self.env.tasks[robot.task_id].get_current_errand()
        

        if current_errand and not self.current_actions:
            goal = (current_errand.target_x, current_errand.target_y)
            start = (robot.x, robot.y)
            
            # Get path using A* (should return list of actions)
            actions = self.a_star(start, goal, manhattan_heuristic)
            if actions:
                self.current_actions = actions
                action = self.current_actions.pop(0)
                return ([action], [task_id])  # Return first action in path
        elif self.current_actions:
            action = self.current_actions.pop(0)
            return ([action], [task_id])  # Return next action in path

        # Default action if no task or path
        return ([Action.WAIT], [task_id])  # Wait and no task assignment    
    
    # ============================================================================
    # TASK 1: SINGLE AGENT PATH PLANNING
    # ============================================================================
    # Implement path planning for a single agent

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int], heuristic: Callable[[Dict[str, Any], Tuple[int, int]], float]) -> List[Action]:
        """Return an optimal action sequence from ``start`` to ``goal`` using A*."""

        # A position alone is not enough: the available forward move also depends
        # on the robot's current direction.
        start_x, start_y = start
        goal_x, goal_y = goal

        if not (0 <= start_x < self.grid_width and 0 <= start_y < self.grid_height):
            return []
        if not (0 <= goal_x < self.grid_width and 0 <= goal_y < self.grid_height):
            return []
        if self.env.grid[start_y, start_x] == 1 or self.env.grid[goal_y, goal_x] == 1:
            return []
        if start == goal:
            return []

        start_state = (start_x, start_y, self.env.robots[0].direction)
        base_environment_state = self.env.get_state()

        def heuristic_value(search_state: Tuple[int, int, Direction]) -> float:
            """Build the state format expected by the supplied heuristic."""
            x, y, direction = search_state
            state_for_heuristic = base_environment_state.copy()
            robots = list(base_environment_state['robots'])
            robot_id, _, _, _, task_id = robots[0]
            robots[0] = (robot_id, x, y, direction, task_id)
            state_for_heuristic['robots'] = robots
            return heuristic(state_for_heuristic, goal)

        def successors(search_state: Tuple[int, int, Direction]):
            """Generate the applicable actions and their successor states."""
            x, y, direction = search_state

            # MOVE_FORWARD is applicable only when the next cell is free.
            dx, dy = [(0, -1), (1, 0), (0, 1), (-1, 0)][direction]
            next_x, next_y = x + dx, y + dy
            if (0 <= next_x < self.grid_width and
                    0 <= next_y < self.grid_height and
                    self.env.grid[next_y, next_x] == 0):
                yield Action.MOVE_FORWARD, (next_x, next_y, direction)

            # Keep the same action order as the supplied Action enumeration.
            yield Action.ROTATE_CW, (x, y, Direction((direction + 1) % 4))
            yield Action.ROTATE_CCW, (x, y, Direction((direction - 1) % 4))
            yield Action.WAIT, search_state

        # Each node stores: search state, parent node, action from parent, and g.
        nodes = [(start_state, None, None, 0)]
        frontier = PriorityQueue()
        closed: Set[Tuple[int, int, Direction]] = set()
        best_g: Dict[Tuple[int, int, Direction], int] = {}

        start_h = heuristic_value(start_state)
        if start_h == float('inf'):
            return []

        # Queue order is f = g + h, then smaller h, then insertion order.
        counter = 0
        frontier.put((start_h, start_h, counter, 0))

        while not frontier.empty():
            _, _, _, node_id = frontier.get()
            state, parent_id, action_from_parent, path_cost = nodes[node_id]

            # Check duplicates when a node is removed from the open list.
            # Re-open a state if this node reaches it with a better g-value.
            if state in closed and path_cost >= best_g[state]:
                continue

            closed.add(state)
            best_g[state] = path_cost

            x, y, _ = state
            if (x, y) == goal:
                path = []
                while parent_id is not None:
                    path.append(action_from_parent)
                    node_id = parent_id
                    state, parent_id, action_from_parent, path_cost = nodes[node_id]
                path.reverse()
                return path

            for action, successor_state in successors(state):
                successor_g = path_cost + 1
                successor_h = heuristic_value(successor_state)

                if successor_h == float('inf'):
                    continue

                nodes.append((successor_state, node_id, action, successor_g))
                successor_id = len(nodes) - 1
                counter += 1
                frontier.put((successor_g + successor_h,
                              successor_h,
                              counter,
                              successor_id))

        # The open list is empty, so no path exists.
        return []


if __name__ == "__main__":
    env = MultiAgentPathfindingEnv(num_robots=1,seed=1, max_timesteps=100)
    agent = SinglePathPlanningAgent(env)
    agent.run(iterations=1, display=True)  
