import random
import time
import math
import heapq
from path_planner_interface import PathPlanner, Configuration

class PRMPlanner(PathPlanner):
    def __init__(self, start, goal, bounds,num_samples=5000,k_neighbors=10):
        PathPlanner.__init__(self, start, goal, bounds)
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.nodes = []   
        self.edges = [] 
        self.adj = []  
        self._planning_time = 0.0
        self._num_nodes = 0
    def _sample_free_point(self):
        (xmin, xmax), (ymin, ymax) = self.bounds
        while True:
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            if not self._point_in_any_obstacle(x, y):
                return x, y

    def _build_roadmap(self):
        self.nodes = []
        for i in range(self.num_samples):
            x, y = self._sample_free_point()
            self.nodes.append((x, y))

        self.nodes.append((self.start.x, self.start.y))
        self.nodes.append((self.goal.x, self.goal.y))

        n = len(self.nodes)
        self.adj = [[] for _ in range(n)]
        self.edges = []

        for i in range(n):
            xi, yi = self.nodes[i]
            dists = []
            for j in range(n):
                if i == j:
                    continue
                xj, yj = self.nodes[j]
                dx = xi - xj
                dy = yi - yj
                d = math.sqrt(dx * dx + dy * dy)
                dists.append((d, j))
            dists.sort(key=lambda t: t[0])
            neighbors = dists[:self.k_neighbors]

            for d, j in neighbors:

                x1, y1 = self.nodes[i]
                x2, y2 = self.nodes[j]
                q1 = Configuration(x1, y1)
                q2 = Configuration(x2, y2)
                if self.is_collision_free(q1, q2):
                    self.adj[i].append((j, d))
                    self.adj[j].append((i, d))
                    if i < j:
                        self.edges.append((i, j))

    def _dijkstra(self, start_idx, goal_idx):
        n = len(self.nodes)
        dist = [float("inf")] * n
        prev = [-1] * n

        dist[start_idx] = 0.0
        pq = []
        heapq.heappush(pq, (0.0, start_idx))

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == goal_idx:
                break
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if dist[goal_idx] == float("inf"):
            return None

        path_idx = []
        cur = goal_idx
        while cur != -1:
            path_idx.append(cur)
            cur = prev[cur]
        path_idx.reverse()
        return path_idx
    
    def plan(self):
        start_time = time.perf_counter()
        self._build_roadmap()
        n = len(self.nodes)
        start_idx = n - 2
        goal_idx = n - 1
        path_idx = self._dijkstra(start_idx, goal_idx)
        if path_idx is None:
            self.path = None
            self._planning_time = time.perf_counter() - start_time
            self._num_nodes = len(self.nodes)
            return False
        path = []
        for i in path_idx:
            x, y = self.nodes[i]
            path.append(Configuration(x, y))

        self.path = path
        self._planning_time = time.perf_counter() - start_time
        self._num_nodes = len(self.nodes)
        return True

    def get_planning_time(self):
        return self._planning_time

    def get_num_nodes(self):
        return self._num_nodes
