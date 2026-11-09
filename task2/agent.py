import numpy as np
from itertools import permutations
from typing import List, Tuple, Dict, Any
from environment import MultiAgentPathfindingEnv, Action, Direction
from hiddenagent import HiddenAgent

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

class SinglePathPlanningAgent(HiddenAgent):

    def choose_action(self, obs: np.ndarray) -> Any:
        """
        Choose an action based on the current observation.
        """
        # Check if agent has an assigned task
        robot = self.env.robots[0]  # Assuming single agent for this class
        task_ids = self.env.get_unassigned_tasks()
        tasks = [
            (self.env.tasks[task_id].get_current_errand().target_x,
             self.env.tasks[task_id].get_current_errand().target_y)
            for task_id in task_ids
        ]

        if not hasattr(self, 'current_actions'):
            self.current_actions = self.a_star((robot.x, robot.y), tasks)
        
        for task_id, task_pos in zip(task_ids, tasks):
            if (robot.x, robot.y) == task_pos:
                return ([Action.WAIT], [task_id])

        if self.current_actions:
            action = self.current_actions.pop(0)
            return ([action], [-1])

      
        # Default action if no task or path
        return ([Action.WAIT], [-1])  # Wait and no task assignment

  # ============================================================================
    # TASK 2: SINGLE AGENT HEURISTIC DESIGN
    # ============================================================================
    # Implement a heuristic for multiple tasks for a single agent

    def task_heuristic_single_agent(self, state: Dict[str, Any], available_tasks: List[Tuple[int,int]]) -> int:
        """
        Design a heuristic to estimate total travel time to complete multiple tasks.
        
        Args:
            state: Current environment state
            available_tasks: List of tasks to consider for the heuristic as (x,y) tuples.

        Returns:
            estimated value of the cost to reach all tasks.
        """
        if not available_tasks:
            return 0

        _, robot_x, robot_y, robot_direction, _ = state['robots'][0]

        min_x = max_x = available_tasks[0][0]
        min_y = max_y = available_tasks[0][1]
        for task_x, task_y in available_tasks[1:]:
            min_x = min(min_x, task_x)
            max_x = max(max_x, task_x)
            min_y = min(min_y, task_y)
            max_y = max(max_y, task_y)

        # Relax obstacles and count only the horizontal and vertical movement
        # that is necessary to cover the task coordinates.
        horizontal_moves = ((max_x - min_x) +
                            min(abs(robot_x - min_x), abs(robot_x - max_x)))
        vertical_moves = ((max_y - min_y) +
                          min(abs(robot_y - min_y), abs(robot_y - max_y)))

        # Reaching coordinates on each side of the robot requires moving in
        # the corresponding direction at least once.  Count the cheapest
        # possible rotations needed to use those directions.
        required_directions = []
        if max_x > robot_x:
            required_directions.append(Direction.EAST)
        if min_x < robot_x:
            required_directions.append(Direction.WEST)
        if max_y > robot_y:
            required_directions.append(Direction.SOUTH)
        if min_y < robot_y:
            required_directions.append(Direction.NORTH)

        minimum_rotations = 0
        if required_directions:
            minimum_rotations = float('inf')
            for order in permutations(required_directions):
                rotations = 0
                direction = int(robot_direction)
                for next_direction in order:
                    difference = abs(direction - int(next_direction))
                    rotations += min(difference, 4 - difference)
                    direction = int(next_direction)
                minimum_rotations = min(minimum_rotations, rotations)

        return int(horizontal_moves + vertical_moves + minimum_rotations)

if __name__ == "__main__":
    env = MultiAgentPathfindingEnv(num_robots=1,seed=3,max_errands_per_task=1,task_reveal_rate=0, tasks_per_robot=7, max_timesteps=40,show_all_tasks=True) 
    agent = SinglePathPlanningAgent(env)
    agent.run(iterations=1, display=True)
  
