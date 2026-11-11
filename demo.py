"""Reproduce an orientation-aware A* route and its grid visualisation."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from environment import MultiAgentPathfindingEnv, Robot, Direction, Action
from task1.agent import SinglePathPlanningAgent, manhattan_heuristic

ROOT=Path(__file__).resolve().parent


def main():
    env=MultiAgentPathfindingEnv(grid_size=(8,6),num_robots=1,seed=42,obstacle_density=0,task_reveal_rate=0)
    env.grid[:]=0
    env.grid[:4,3]=1;env.grid[2:,5]=1
    env.robots=[Robot(0,0,0,Direction.EAST)]
    goal=(7,5)
    actions=SinglePathPlanningAgent(env).a_star((0,0),goal,manhattan_heuristic)
    x=y=0;direction=int(Direction.EAST);route=[(x,y)]
    for action in actions:
        if action==Action.ROTATE_CW:direction=(direction+1)%4
        elif action==Action.ROTATE_CCW:direction=(direction-1)%4
        elif action==Action.MOVE_FORWARD:
            dx,dy=[(0,-1),(1,0),(0,1),(-1,0)][direction]
            x+=dx;y+=dy;route.append((x,y))
            assert env.grid[y,x]!=1
    assert (x,y)==goal
    fig,ax=plt.subplots(figsize=(9,6),layout="constrained")
    ax.imshow(env.grid,cmap=ListedColormap(["#eef3f8","#24354b"]),vmin=0,vmax=1)
    xx,yy=zip(*route)
    ax.plot(xx,yy,"-o",color="#079b87",linewidth=3,markersize=4)
    ax.scatter([0],[0],s=170,color="#1385d6",zorder=4,label="Start (facing east)")
    ax.scatter([7],[5],s=210,color="#ec9c34",marker="*",zorder=4,label="Goal")
    ax.set_xticks(range(8));ax.set_yticks(range(6))
    ax.set_xticks(np.arange(-.5,8,1),minor=True);ax.set_yticks(np.arange(-.5,6,1),minor=True)
    ax.grid(which="minor",color="white",linewidth=2);ax.tick_params(which="minor",length=0)
    turns=sum(a in [Action.ROTATE_CW,Action.ROTATE_CCW] for a in actions)
    ax.set_title(f"A* path planning\n{len(actions)} actions · {len(route)-1} forward moves · {turns} rotations",loc="left",fontweight="bold",fontsize=16,pad=18)
    ax.set_xlabel("Grid column");ax.set_ylabel("Grid row");ax.legend(loc="upper right",fontsize=8)
    (ROOT/"docs").mkdir(exist_ok=True)
    fig.savefig(ROOT/"docs/path-demo.png",dpi=150);fig.savefig(ROOT/"docs/path-demo.svg");plt.close(fig)
    result={"actions":[a.name for a in actions],"route":route,"goal":goal}
    (ROOT/"docs/path-demo.json").write_text(json.dumps(result,indent=2)+"\n")
    print(f"Reached {goal}: {len(actions)} actions, {turns} rotations. See docs/path-demo.png")


if __name__=="__main__":main()
