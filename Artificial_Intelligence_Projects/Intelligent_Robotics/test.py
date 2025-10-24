import matplotlib.pyplot as plt
import numpy as np


# def plot_points(points, title="Plot of Points", xlabel="X-axis", ylabel="Y-axis"):
#     """
#     Plots a list of points.

#     Parameters:
#     - points: List of tuples, where each tuple contains two NumPy arrays representing x and y coordinates.
#               Example: [(array([x1]), array([y1])), (array([x2]), array([y2])), ...]
#     - title: Title of the plot.
#     - xlabel: Label for the X-axis.
#     - ylabel: Label for the Y-axis.
#     """
#     # Extract x and y values from the list of points
#     x_values = [point[0][0] for point in points]  # Extract scalar from array
#     y_values = [point[1][0] for point in points]  # Extract scalar from array

#     # Create a new figure and axis
#     plt.figure(figsize=(8, 6))
#     plt.scatter(x_values, y_values, marker='o', linestyle='-', color='b', label='Path')

#     # Add labels and title
#     plt.title(title)
#     plt.xlabel(xlabel)
#     plt.ylabel(ylabel)

#     # Add grid
#     plt.grid(True)

#     # Add legend
#     plt.legend()

#     # Show the plot
#     plt.show()


# def generate_line_points(x, y, xEnd, yEnd, n_points=10):
#     # Calculate the step size for x and y directions
#     x_step = (xEnd - x) / (n_points - 1)
#     y_step = (yEnd - y) / (n_points - 1)

#     # Generate the points
#     points = []
#     for i in range(n_points):
#         new_x = x + i * x_step
#         new_y = y + i * y_step
#         points.append((new_x, new_y))
    
#     return points




# class Circle:
    
#     def __init__(self, c_x, c_y, radius,name):
#         self.center = np.array([c_x, c_y])
#         self.radius = radius
#         self.name = name

#     def is_in_collision_with_points(self, points):
#         collisionPoint = [0,0]

#         dist = []
#         for point in points:
#             dx = self.center[0] - point[0]
#             dy = self.center[1] - point[1]

#             dist.append(dx * dx + dy * dy)
#         if np.min(dist) <= (self.radius+0.03)**2:
#             # print(np.argmin(dist))
#             # print(points[np.argmin(dist)])
#             # input("check above")
#             collisionPoint = points[np.argmin(dist)]
#         # if np.min(dist) <= self.radius:
#             return True, collisionPoint

#         return False,collisionPoint  # safe
    

    
# # x, y = 0, 0
# # xEnd, yEnd = 1, 1

# # obs = Circle(1,1,0.8,"None")

# # # Generate points on the line
# # points = generate_line_points(x, y, xEnd, yEnd, n_points=5)

# # print(obs.is_in_collision_with_points(points))


# points = [
#     (np.array([-0.033135]), np.array([0.29327])),
#     (np.array([-0.033706]), np.array([0.29405])),
#     (np.array([-0.034276]), np.array([0.29482])),
#     (np.array([-0.034847]), np.array([0.2956])),
#     (np.array([-0.035417]), np.array([0.29638])),
#     (np.array([-0.035988]), np.array([0.29716])),
#     (np.array([-0.036558]), np.array([0.29794])),
#     (np.array([-0.037129]), np.array([0.29872])),
#     (np.array([-0.037699]), np.array([0.29949])),
#     (np.array([-0.03827]), np.array([0.30027])),
#     (np.array([-0.42145114312862453]), np.array([1.046422875320986]))
# ]

# # {'y': 1.046422875320986, 'x': -0.42145114312862453}

# # Call the plot_points function
# plot_points(points, title="Robot Path", xlabel="X Position", ylabel="Y Position")



z = -0.00000000001



for i in range(1000):
    if z >= -1 and z <= 1:
        z = np.exp(z) - np.exp(-z)
    elif z > 1:
        z = np.exp(1) - np.exp(-1)
    elif z <-1:
        z = np.exp(-1) - np.exp(1)
        
import numpy as np
import matplotlib.pyplot as plt

# Define the functions exp(x) - exp(-x) and y = x
def func(x):
    return np.exp(x) - np.exp(-x)

def linear_func(x):
    return x

# Create a range of x values
x = np.linspace(-1, 1, 400)

# Calculate y values for both functions
y = func(x)
y_linear = linear_func(x)

# Plot both functions on the same graph
plt.plot(x, y, label=r'$e^x - e^{-x}$')
plt.plot(x, y_linear, label=r'$y = x$', linestyle='--', color='red')

# Add labels and title
plt.xlabel('x')
plt.ylabel('y')
plt.title('Plot of $e^x - e^{-x}$ and $y = x$')

# Show grid and legend
plt.grid(True)
plt.legend()

# Display the plot
plt.show()
