import random
import time
import math
from path_planner_interface import PathPlanner, Configuration

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0 


class RRTPlanner(PathPlanner):
    def __init__(self, start, goal, bounds,step_size=1.0,max_iterations=5000,goal_sample_rate=0.05):

        PathPlanner.__init__(self, start, goal, bounds)
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_sample_rate = goal_sample_rate

        self.nodes = []  
        self.edges = []   

        self._planning_time = 0.0
        self._num_nodes = 0

    def _sample_random_point(self):

        if random.random() < self.goal_sample_rate:
            return self.goal.x, self.goal.y

        (xmin, xmax), (ymin, ymax) = self.bounds
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        return x, y

    def _nearest_node_index(self, x, y):
        best_idx = 0
        best_dist = float("inf")
        for i, node in enumerate(self.nodes):
            dx = node.x - x
            dy = node.y - y
            d = dx * dx + dy * dy
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _steer(self, from_node, x_rand, y_rand):
        dx = x_rand - from_node.x
        dy = y_rand - from_node.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= self.step_size:
            return Node(x_rand, y_rand)
        if dist == 0.0:
            return Node(from_node.x, from_node.y)
        scale = self.step_size / dist
        x_new = from_node.x + scale * dx
        y_new = from_node.y + scale * dy
        return Node(x_new, y_new)

    def _build_path(self, goal_node_index):
        path = []
        cur = self.nodes[goal_node_index]
        while cur is not None:
            cfg = Configuration(cur.x, cur.y)
            path.append(cfg)
            cur = cur.parent
        path.reverse()
        self.path = path

    def plan(self):
        start_time = time.perf_counter()

        self.nodes = []
        self.edges = []

        start_node = Node(self.start.x, self.start.y)
        self.nodes.append(start_node)

        for i in range(self.max_iterations):
            x_rand, y_rand = self._sample_random_point()
            idx_nearest = self._nearest_node_index(x_rand, y_rand)
            nearest = self.nodes[idx_nearest]

            new_node = self._steer(nearest, x_rand, y_rand)
            q1 = Configuration(nearest.x, nearest.y)
            q2 = Configuration(new_node.x, new_node.y)
            if not self.is_collision_free(q1, q2):
                continue
            new_node.parent = nearest
            self.nodes.append(new_node)
            self.edges.append((idx_nearest, len(self.nodes) - 1))

            q_goal = Configuration(self.goal.x, self.goal.y)
            q_new = Configuration(new_node.x, new_node.y)
            if q_new.distance_to(q_goal) <= self.step_size:
                if self.is_collision_free(q_new, q_goal):
                    goal_node = Node(self.goal.x, self.goal.y)
                    goal_node.parent = new_node
                    self.nodes.append(goal_node)
                    self.edges.append((len(self.nodes) - 2, len(self.nodes) - 1))

                    self._build_path(len(self.nodes) - 1)
                    self._planning_time = time.perf_counter() - start_time
                    self._num_nodes = len(self.nodes)
                    return True

        self.path = None
        self._planning_time = time.perf_counter() - start_time
        self._num_nodes = len(self.nodes)
        return False
    def get_planning_time(self):
        return self._planning_time
    def get_num_nodes(self):
        return self._num_nodes

