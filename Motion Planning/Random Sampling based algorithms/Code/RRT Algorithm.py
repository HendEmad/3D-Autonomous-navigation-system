import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
import time

np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22


# Treenode class
class treeNode:
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None


# RRT algorithm class
class RRTAlgorithm:
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])  # Tree root
        self.goal = treeNode(goal[0], goal[1])  # Tree goal
        self.nearestNode = None  # Nearest node
        self.iterations = min(numIterations, 250)  # Number of iterations to be done
        self.grid = grid  # The map to find path on
        self.rho = stepSize  # Length of each branch(edge)
        self.path_distance = 0  # Total path distance (cost)
        self.nearestDist = 10000  # Distance to the nearest node(not to be exceeded)
        self.numWaPoints = 0  # Number of wayPoints
        self.wayPoints = []  # The wayPoints list
        self.serializedTree = []  # The serialized tree as list of lists

    # Function to add the node to the nearest node, and add goal if necessary
    def addChild(self, locationX, locationY):
        # If we reach a goal
        if locationX == self.goal.locationX:
            # Append goal to the nearestNode's children
            self.nearestNode.children.append(self.goal)
            # And set goal's parent to nearestNode
            self.goal.parent = self.nearestNode
        else:  # If not the goal
            # Create a tree node from locationX, locationY
            tempNode = treeNode(locationX, locationY)
            # Append this node to nearestNode's children
            self.nearestNode.children.append(tempNode)
            # Set the parent to nearestNode
            tempNode.parent = self.nearestNode

    # Function to perform sampling
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point

    # Function to steer a distance stepSize from start to end locations with taking the grid limits into consideration
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho * self.unitVector(locationStart, locationEnd)  # Vector
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        # Check if the point doesn't exceed the grid
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1] - 1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        return point

    # Function to check if obstacle lies between the start and end points of each edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = min(grid.shape[1] - 1, locationStart.locationX + i * u_hat[0])
            testPoint[1] = min(grid.shape[0] - 1, locationStart.locationY + i * u_hat[1])
            # If an obstacle, the value of the grid at y,x will correspond to 1
            if self.grid[round(testPoint[1]), round(testPoint[0])] == 1:  # [y, x]
                return True
        else:
            return False

    # Function to find the unitVector between 2 locations (delta_x, delta_y)
    def unitVector(self, locationStart, locationEnd):  # locationStart and locationEnd are arrays
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v / np.linalg.norm(v)  # Norm is the magnitude
        return u_hat

    # Function to find the nearest node from a given unconnected point to a root (Euclidean point)
    def findNearest(self, root, point):
        if not root:
            return
        # Find distance between root and point using distance function
        dist = self.distance(root, point)  # root will be updated to the children nodes (recursion process)
        # If the dist is lower than or equal to nearestDist
        if dist <= self.nearestDist:
            # update nearestNode to root
            self.nearestNode = root
            # Update nearestDist to dist
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)

    # Function to find the euclidean distance between a node object and a xy point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist

    # Function to check if the goal is found (check if the goal is within the stepSize rho from a point)
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        pass

    # Function to reset nearestNode to None and the nearestDist to 10000
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    # Function to trace the path from goal to start root by backwards from the goal
    def retraceRRTPath(self, goal):
        if goal.locationX == self.randomTree.locationX:
            return  # Return if it is at the start, algorithm termination
        # add 1 to numWayPoints
        self.numWaPoints += 1
        # Extract x and y locations from the goal point
        currentPoint = np.array([goal.locationX, goal.locationY])
        # Insert this point to wayPoints from the beginning
        self.wayPoints.insert(0, currentPoint)
        # Add rho to path_distance
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)  # Call the function again with the parent


# Load the grid, set the start and the goal (x, y) positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])
numIterations = 200
stepSize = 100
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill=False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

# Initialize the class; the beginning
rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(2)

# RRT Algorithm iteration
startTime = time.time()
for i in range(rrt.iterations):
    # 1. Reset the nearest values, call the resetNearestValues method
    rrt.resetNearestValues()
    print("Iteration: ", i)
    # 2. Sample the point; the algorithm starts here
    point = rrt.sampleAPoint()
    # 3. Find the nearest node with respect to the point
    rrt.findNearest(rrt.randomTree, point)  # (root, point)
    # 4. Steer to a point
    new = rrt.steerToPoint(rrt.nearestNode, point)
    # 5. Check if obstacles
    if not rrt.isInObstacle(rrt.nearestNode, new):
        # Add new to the nearestNode (addChild)
        rrt.addChild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle='--')
        # If goal is found; new is within the goal region
        if rrt.goalFound(new):
            # Append goal ti path
            rrt.addChild(goal[0], goal[1])
            rrt.retraceRRTPath(rrt.goal)
            print("Goal is reached!")
            break

endTime = time.time()
print("Time of calculating the free path = ", endTime - startTime)

# Add start to wayPoints
rrt.wayPoints.insert(0, start)
print("Number of wayPoints: ", rrt.numWaPoints)
print("Path Distance (m): ", rrt.path_distance)
print("WayPoints: ", rrt.wayPoints)

# Plot wayPoints
for i in range(len(rrt.wayPoints) - 1):
    plt.plot([rrt.wayPoints[i][0], rrt.wayPoints[i + 1][0]], [rrt.wayPoints[i][1], rrt.wayPoints[i + 1][1]], 'ro',
             linestyle='--')
    plt.pause(0.10)

plt.pause(100)
