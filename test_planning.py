"""Behaviour checks for portable agents, using independent BFS oracles."""
from collections import deque
from itertools import combinations
import unittest
import numpy as np
from environment import MultiAgentPathfindingEnv, Robot, Task, Errand, Action, Direction
from task1.agent import SinglePathPlanningAgent, manhattan_heuristic
from task2.agent import SinglePathPlanningAgent as HeuristicAgent
from task3.agent import SingleTaskSchedulingAgent


def world(width=4, height=4):
    env = MultiAgentPathfindingEnv(grid_size=(width,height), num_robots=1, seed=42, obstacle_density=0, task_reveal_rate=0, max_timesteps=60)
    env.grid[:] = 0
    env.robots = [Robot(0,0,0,Direction.EAST)]
    env.tasks = []
    return env


def bfs_cost(env, start, direction, goals):
    initial = (*start, int(direction), frozenset(goals)-{start})
    queue = deque([(initial,0)])
    seen = {initial}
    while queue:
        (x,y,d,todo), cost = queue.popleft()
        if not todo:
            return cost
        successors = [(x,y,(d+1)%4),(x,y,(d-1)%4)]
        dx,dy = [(0,-1),(1,0),(0,1),(-1,0)][d]
        nx,ny=x+dx,y+dy
        if 0 <= nx < env.grid_width and 0 <= ny < env.grid_height and env.grid[ny,nx]!=1:
            successors.append((nx,ny,d))
        for nx,ny,nd in successors:
            state=(nx,ny,nd,todo-{(nx,ny)})
            if state not in seen:
                seen.add(state);queue.append((state,cost+1))
    return None


class PlanningTest(unittest.TestCase):
    def test_astar_matches_bfs_with_orientation_and_obstacles(self):
        env=world();env.grid[1,1]=env.grid[2,1]=1
        agent=SinglePathPlanningAgent(env)
        for d in Direction:
            env.robots[0].direction=d
            for goal in [(0,0),(3,0),(0,3),(3,3),(2,1)]:
                with self.subTest(direction=d,goal=goal):
                    self.assertEqual(len(agent.a_star((0,0),goal,manhattan_heuristic)),bfs_cost(env,(0,0),d,[goal]))

    def test_unreachable_and_out_of_bounds_goals(self):
        env=world();env.grid[0,1]=env.grid[1,0]=1
        agent=SinglePathPlanningAgent(env)
        self.assertEqual(agent.a_star((0,0),(3,3),manhattan_heuristic),[])
        self.assertEqual(agent.a_star((0,0),(9,9),manhattan_heuristic),[])

    def test_multi_target_heuristic_is_admissible_on_small_cases(self):
        env=world(3,3);agent=HeuristicAgent(env)
        for d in Direction:
            env.robots[0].direction=d
            for goals in combinations([(0,2),(2,0),(2,2)],2):
                exact=bfs_cost(env,(0,0),d,goals)
                h=agent.task_heuristic_single_agent(env.get_state(),list(goals))
                self.assertLessEqual(h,exact)
                self.assertEqual(len(agent.a_star((0,0),goals)),exact)

    def test_agents_complete_actual_environment_episodes(self):
        for kind in [SinglePathPlanningAgent,HeuristicAgent,SingleTaskSchedulingAgent]:
            with self.subTest(agent=kind.__name__):
                env=world()
                env.tasks=[Task(0,[Errand(2,0)],revealed=True),Task(1,[Errand(2,2)],revealed=True)]
                env.revealed_tasks=2
                kind(env).run(iterations=1,display=False)
                self.assertEqual(env.completed_tasks,2)

    def test_idle_and_budget_exhausted(self):
        env=world()
        self.assertEqual(SinglePathPlanningAgent(env).choose_action(env._get_observation()),([Action.WAIT],[-1]))
        env.timestep=env.max_timesteps
        agent=SingleTaskSchedulingAgent(env)
        self.assertEqual(agent.assign_tasks_single_agent((0,0),[Task(0,[Errand(1,0)],revealed=True)]),-1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
