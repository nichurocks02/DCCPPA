import math
import random

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class PRM:
    def __init__(self, start, goal, obstacles, num_samples, connection_radius):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.num_samples = num_samples
        self.connection_radius = connection_radius
        self.vertices = [start, goal]
        self.edges = []

    def generate_samples(self):
        for _ in range(self.num_samples):
            x = random.uniform(0, 10)
            y = random.uniform(0, 10)
            sample = Obstacle(x, y, 0)
            if all(not self.is_collision(sample, obstacle) for obstacle in self.obstacles):
                self.vertices.append(sample)

    def is_collision(self, point, obstacle):
        return math.dist((point.x, point.y), (obstacle.x, obstacle.y)) <= obstacle.radius

    def find_neighbors(self, point):
        return [v for v in self.vertices if math.dist((point.x, point.y), (v.x, v.y)) <= self.connection_radius]

    def build_roadmap(self):
        for v in self.vertices:
            neighbors = self.find_neighbors(v)
            for n in neighbors:
                if all(not self.is_collision(v, obstacle) for obstacle in self.obstacles) and \
                   all(not self.is_collision(n, obstacle) for obstacle in self.obstacles):
                    self.edges.append((v, n))

    def find_path(self):
        self.generate_samples()
        self.build_roadmap()
        return len(self.edges)

class RRT:
    def __init__(self, start, goal, obstacles, num_iterations, step_size):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.num_iterations = num_iterations
        self.step_size = step_size
        self.vertices = [start]
        self.edges = []

    def random_point(self):
        if random.uniform(0, 1) < 0.1:
            return self.goal
        else:
            x = random.uniform(0, 10)
            y = random.uniform(0, 10)
            return Obstacle(x, y, 0)

    def nearest_neighbor(self, point):
        return min(self.vertices, key=lambda v: math.dist((point.x, point.y), (v.x, v.y)))

    def steer(self, from_point, to_point):
        distance = math.dist((from_point.x, from_point.y), (to_point.x, to_point.y))
        if distance < self.step_size:
            return to_point
        else:
            angle = math.atan2(to_point.y - from_point.y, to_point.x - from_point.x)
            x = from_point.x + self.step_size * math.cos(angle)
            y = from_point.y + self.step_size * math.sin(angle)
            return Obstacle(x, y, 0)

    def is_collision(self, point, obstacle):
        return math.dist((point.x, point.y), (obstacle.x, obstacle.y)) <= obstacle.radius

    def build_tree(self):
        for _ in range(self.num_iterations):
            random_point = self.random_point()
            nearest = self.nearest_neighbor(random_point)
            new_point = self.steer(nearest, random_point)
            if all(not self.is_collision(new_point, obstacle) for obstacle in self.obstacles):
                self.vertices.append(new_point)
                self.edges.append((nearest, new_point))

    def find_path(self):
        self.build_tree()
        return len(self.edges)

# Example usage
start = Obstacle(1, 1, 0)
goal = Obstacle(9, 9, 0)
obstacles = [Obstacle(2, 4, 1),
    Obstacle(4, 6, 1),
    Obstacle(6, 2, 1),]
num_samples = 200
connection_radius = 2.0
num_iterations = 1000
step_size = 0.5

# PRM
prm = PRM(start, goal, obstacles, num_samples, connection_radius)
prm_steps = prm.find_path()

# RRT
rrt = RRT(start, goal, obstacles, num_iterations, step_size)
rrt_steps = rrt.find_path()

print(f"PRM Steps: {prm_steps}")
print(f"RRT Steps: {rrt_steps}")