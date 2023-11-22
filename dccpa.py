import math
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class DCCPPA:
    def __init__(self, start, goal, obstacles, curvature_threshold):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.curvature_threshold = curvature_threshold

    def evaluate_curvature(self, point):
        total_curvature = sum(1 / (obstacle.radius + math.dist((point.x, point.y), (obstacle.x, obstacle.y)))
                             for obstacle in self.obstacles)
        return total_curvature

    def local_search(self, current_point):
        dx = self.goal.x - current_point.x
        dy = self.goal.y - current_point.y
        norm = math.sqrt(dx**2 + dy**2)
        step_size = min(0.1, norm)
        new_x = current_point.x + step_size * dx / norm
        new_y = current_point.y + step_size * dy / norm
        new_point = Obstacle(new_x, new_y, 0)
        return new_point

    def global_search(self, current_point):
        for _ in range(100):
            new_point = Obstacle(random.uniform(0, 10), random.uniform(0, 10), 0)
            if self.evaluate_curvature(new_point) <= self.curvature_threshold:
                return new_point

    def plan_path(self):
        current_point = self.start
        path = [current_point]

        while math.dist((current_point.x, current_point.y), (self.goal.x, self.goal.y)) > 0.1:
            curvature = self.evaluate_curvature(current_point)
            if curvature > self.curvature_threshold:
                new_point = self.global_search(current_point)
            else:
                new_point = self.local_search(current_point)

            path.append(new_point)
            current_point = new_point

        return path

# Example usage
start = Obstacle(1, 1, 0)
goal = Obstacle(9, 9, 0)
obstacles = [Obstacle(2, 4, 1),
              Obstacle(4, 6, 1),
              Obstacle(6, 2, 1),]
curvature_threshold = 0.5

dccppa = DCCPPA(start, goal, obstacles, curvature_threshold)
path = dccppa.plan_path()

# Plotting
obstacle_x = [obstacle.x for obstacle in obstacles]
obstacle_y = [obstacle.y for obstacle in obstacles]

# Create a scatter plot for obstacles, start and goal points
plt.scatter(obstacle_x, obstacle_y, color='red', label='Obstacles')
plt.scatter(start.x, start.y, color='green', marker='o', label='Start Point')
plt.scatter(goal.x, goal.y, color='purple', marker='x', label='Goal Point')

# Initialize the plot for the path
line, = plt.plot([], [], marker='o', linestyle='-', color='blue', label='DCCPPA Path')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('DCCPPA Path Planning')
plt.legend()

def update(frame):
    line.set_data([point.x for point in path[:frame+1]], [point.y for point in path[:frame+1]])
    return line,

# Animate the path
animation = FuncAnimation(plt.gcf(), update, frames=len(path), interval=200, blit=True)
plt.show()
