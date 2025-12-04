import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
from path_planner_interface import Configuration
from test_planners import generate_random_obstacles
from RRT import RRTPlanner

def plot_rrt(planner, title="RRT (10000 iterations, 100 obstacles)"):

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title(title)

    (xmin, xmax), (ymin, ymax) = planner.bounds
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)

    patches_list = []
    for obs in planner.obstacles:
        t = obs.get("type", "circle")
        if t == "circle":
            cx = obs.get("cx", 0.0)
            cy = obs.get("cy", 0.0)
            r = obs.get("r", obs.get("radius", 0.5))
            c = patches.Circle((cx, cy), r, color="gray", alpha=0.5)
            patches_list.append(c)

        elif t == "rect":
            x_min = obs["xmin"]
            x_max = obs["xmax"]
            y_min = obs["ymin"]
            y_max = obs["ymax"]
            width = x_max - x_min
            height = y_max - y_min
            r = patches.Rectangle((x_min, y_min),width,height,color="gray",alpha=0.5,)
            patches_list.append(r)

        elif t == "polygon":
            verts = obs.get("vertices", obs.get("points", []))
            if verts:
                poly = patches.Polygon(verts, closed=True,color="gray", alpha=0.5)
                patches_list.append(poly)

    if patches_list:
        pc = PatchCollection(patches_list, match_original=True)
        ax.add_collection(pc)


    xs = []
    ys = []
    for n in planner.nodes:
        if hasattr(n, "x"):
            xs.append(n.x)
            ys.append(n.y)
        else:
            xs.append(n[0])
            ys.append(n[1])
    if xs:
        ax.scatter(xs, ys, s=5, color="blue", label="Tree nodes")
    if hasattr(planner, "edges") and planner.edges:
        for i, j in planner.edges:
            ni = planner.nodes[i]
            nj = planner.nodes[j]
            x1 = ni.x if hasattr(ni, "x") else ni[0]
            y1 = ni.y if hasattr(ni, "y") else ni[1]
            x2 = nj.x if hasattr(nj, "x") else nj[0]
            y2 = nj.y if hasattr(nj, "y") else nj[1]
            ax.plot([x1, x2], [y1, y2], color="lightblue", linewidth=0.5)

    path = getattr(planner, "path", None)
    if path and len(path) > 1:
        px = [p.x for p in path]
        py = [p.y for p in path]
        ax.plot(px, py, color="red", linewidth=3, label="Final path")

    ax.scatter(planner.start.x, planner.start.y,
               color="green", s=60, label="Start")
    ax.scatter(planner.goal.x, planner.goal.y,
               color="red", s=60, label="Goal")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
   
    bounds = ((0.0, 10.0), (0.0, 10.0))
    start = Configuration(1.0, 1.0)
    goal = Configuration(9.0, 9.0)

    print("Generating 100 obstacles...")
    obstacles = generate_random_obstacles(bounds, 100, start, goal)

    print("Running RRT with 10000 iterations...")
    rrt = RRTPlanner(start=start,goal=goal,bounds=bounds,step_size=1.0,max_iterations=10000, )
    rrt.set_obstacles(obstacles)
    
    success = rrt.plan()
    print("Success:", success)
    print("Planning time:", rrt.get_planning_time())
    print("Number of nodes:", rrt.get_num_nodes())

    plot_rrt(rrt)
