import random
import time
import math
from configuration3d import Configuration3D
def steer(q_near: Configuration3D, q_rand: Configuration3D, step: float) -> Configuration3D:
    dx = q_rand.x - q_near.x
    dy = q_rand.y - q_near.y
    dz = q_rand.z - q_near.z
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist == 0.0:
        return q_near
    if dist <= step:
        return q_rand
    ux = dx / dist
    uy = dy / dist
    uz = dz / dist

    return Configuration3D(q_near.x + step * ux,q_near.y + step * uy,q_near.z + step * uz,)

def in_bounds(q: Configuration3D, bounds) -> bool:
    (xmin, xmax), (ymin, ymax), (zmin, zmax) = bounds
    return (xmin <= q.x <= xmax and
            ymin <= q.y <= ymax and
            zmin <= q.z <= zmax)

def rrt_3d(start: Configuration3D,
           goal: Configuration3D,
           bounds,
           step: float = 0.5,
           max_iter: int = 5000) -> dict:
    tree = [start]
    parent = {0: None}
    success = False
    t0 = time.time()
    for i in range(max_iter):
        rx = random.uniform(bounds[0][0], bounds[0][1])
        ry = random.uniform(bounds[1][0], bounds[1][1])
        rz = random.uniform(bounds[2][0], bounds[2][1])
        q_rand = Configuration3D(rx, ry, rz)

 
        nearest_index = 0
        nearest_dist = float("inf")
        for idx, node in enumerate(tree):
            d = node.distance_to(q_rand)
            if d < nearest_dist:
                nearest_dist = d
                nearest_index = idx

        q_near = tree[nearest_index]
        q_new = steer(q_near, q_rand, step)

        if not in_bounds(q_new, bounds):
            continue
        new_index = len(tree)
        tree.append(q_new)
        parent[new_index] = nearest_index

        if q_new.distance_to(goal) < step:
            goal_index = len(tree)
            tree.append(goal)
            parent[goal_index] = new_index
            success = True
            break
    t1 = time.time()
    path = []
    idx = len(tree) - 1
    while idx is not None:
        path.append(tree[idx])
        idx = parent[idx]
    path.reverse()

    return {"success": success,"path": path,"nodes": tree,"time": t1 - t0,}

