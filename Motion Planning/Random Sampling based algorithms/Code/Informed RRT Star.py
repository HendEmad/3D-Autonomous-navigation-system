import numpy as np
import matplotlib.pyplot as plt
import random
import time
from matplotlib.pyplot import rcParams

np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 15


# tree Node class
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX  # X Location
        self.locationY = locationY  # Y Location
        self.children = []  # children list
        self.parent = None  # parent node reference


# Informed RRT Star Algorithm class
class InformedRRTStarAlgorithm():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])  # The RRT (root position) (has 0 cost)
        self.goal = treeNode(goal[0], goal[1])  # goal position (initialize to a high cost)
        self.nearestNode = None  # nearest node
        self.iterations = min(numIterations, 1400)  # number of iterations to run
        self.grid = grid  # the map
        self.rho = stepSize  # length of each branch
        self.nearestDist = 10000  # distance to nearest node (initialize with large)
        self.numWaypoints = 0  # number of waypoints
        self.Waypoints = []  # the waypoints
        self.serializedTree = []  # the serialized tree as list of lists
        self.searchRadius = self.rho * 2  # the radius to search for finding neighbouring vertices
        self.neighbouringNodes = []  # neighbouring nodes
        self.goalArray = np.array([goal[0], goal[1]])  # goal as an array
        self.goalCosts = [10000]  # the costs to the goal (ignore first value)
        self.initialPathFound = False  # trigger when initial path obtained
        self.ellipseAngle = np.arctan2(goal[1] - start[1], goal[0] - start[0])
        self.xCenterEllipse = 0.5 * (start[0] + goal[0])  # x-center of ellipse
        self.yCenterEllipse = 0.5 * (start[1] + goal[1])  # y-center of ellipse
        self.c_min = np.sqrt((goal[1] - start[1]) ** 2 + (goal[0] - start[0]) ** 2)
        self.a = np.linspace(0, 2 * np.pi, 100)  # angle for parametric ellipse plots

    # Function to add the sampled node to the nearest node, and add goal if necessary
    def addChild(self, treeNode):
        if treeNode.locationX == self.goal.locationX:
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            self.nearestNode.children.append(treeNode)
            treeNode.parent = self.nearestNode

    # Function to sample random point within grid limits
    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point

    # Function to sample a random point within ellipse limits
    def checkInEllipse(self, pointX, pointY, c_best):
        rad_x = c_best / 2
        rad_y = np.sqrt(c_best ** 2 - self.c_min ** 2) / 2
        if (((pointX - self.xCenterEllipse) * np.cos(-self.ellipseAngle) + (pointY - self.yCenterEllipse) * np.sin(
                -self.ellipseAngle)) ** 2 / rad_x ** 2 + (
                    (pointX - self.xCenterEllipse) * np.sin(-self.ellipseAngle) + (
                    pointY - self.yCenterEllipse) * np.cos(-self.ellipseAngle)) ** 2 / rad_y ** 2) < 1:
            return True
        return False

    # Function to plot the ellipse
    def plotEllipse(self, c_best):
        rad_x = c_best / 2
        rad_y = np.sqrt(c_best ** 2 - self.c_min ** 2) / 2
        plt.plot(
            rad_x * np.cos(self.a) * np.cos(self.ellipseAngle) - rad_y * np.sin(self.a) * np.sin(
                self.ellipseAngle) + self.xCenterEllipse,
            rad_x * np.cos(self.a) * np.sin(self.ellipseAngle) - rad_y * np.sin(self.a) * np.cos(
                self.ellipseAngle) + self.yCenterEllipse)

    # Function to steer a distance stepSize from start location to end location with keeping the grid limits under consideration
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1] - 1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        return point

    # Function to check if obstacles lies between the start and end points of the edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        test_point = np.array([0.0, 0.0])
        distance = int(self.distance(locationStart, locationEnd))
        for i in range(distance):
            test_point[0] = min(1799, locationStart.locationX + i * u_hat[0])
            test_point[1] = min(1119, locationStart.locationY + i * u_hat[1])
            if self.grid[round(test_point[1]), round(test_point[0])] == 1:
                return True
        return False

    # Function to find the unit vector between two points
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        v_norm = np.linalg.norm(v)
        if v_norm < 1:  # Avoid division by Zero
            v_norm = 1
        u_hat = v / v_norm
        return u_hat

    # Function to find the nearest node from ag given unconnected point (Euclidean distance)
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist and root.locationX != self.goal.locationX:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)

    # Function to find the neighbouring nodes
    def findNeighbouringNodes(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.searchRadius:
            self.neighbouringNodes.append(root)
        for child in root.children:
            self.findNeighbouringNodes(child, point)

    # Function to find the distance between a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist

    # Function to check if the goal is within stepSize (rho) distance from point
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False

    # Function to reset the nearestNode to None, and nearestDistance to 10000 and neighbouringNodes to empty array
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        self.neighbouringNodes = []

    # Function to find the path from goal to start, since have to reset if called many times
    def retracePath(self):
        self.numWayPoints = 0
        self.wayPoints = []
        goalCost = 0
        goal = self.goal
        while goal.locationX != self.randomTree.locationX:
            self.numWayPoints += 1
            currentPoint = np.array([goal.locationX, goal.locationY])
            self.wayPoints.insert(0, currentPoint)
            goalCost += self.distance(goal, np.array([goal.parent.locationX, goal.parent.locationY]))
            goal = goal.parent
        self.goalCosts.append(goalCost)

        # Find unique path length from root of the node --> Cost
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += self.distance(currentNode, np.array([currentNode.parent.locationX, currentNode.parent.locationY]))
            currentNode = currentNode.parent
        return costFromRoot


# Load the grid, get start and goal <x, y> positions, number of iterations and step size
grid = np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])
numIterations = 700
stepSize = 75
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill=False)

fig = plt.figure("Informed RRT Star Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

# Begin
iRRTStar = InformedRRTStarAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(4)

# Informed RRT Stat algorithm
start_time = time.time()
for i in range(iRRTStar.iterations):
    iRRTStar.resetNearestValues()
    print("Iteration: ", i)

    # Algorithm begins here
    point = iRRTStar.sampleAPoint()
    # 1. If an initial path had been found
    if iRRTStar.initialPathFound:
        # Get c_best, the last element of the goalCosts
        c_best = iRRTStar.goalCosts[-1]
        # If the point is not in ellipse, go to the next iteration of the loop
        if not iRRTStar.checkInEllipse(point[0], point[1], c_best):
            continue

    # 2. Find the nearest node with respect to the point
    iRRTStar.findNearest(iRRTStar.randomTree, point)

    # 3. Steer to point
    new = iRRTStar.steerToPoint(iRRTStar.nearestNode, point)
    # If not in obstacle
    if not iRRTStar.isInObstacle(iRRTStar.nearestNode, new):
        iRRTStar.findNeighbouringNodes(iRRTStar.randomTree, new)
        min_cost_node = iRRTStar.nearestNode
        min_cost = iRRTStar.findPathDistance(min_cost_node)
        min_cost += iRRTStar.distance(iRRTStar.nearestNode, new)

        # Connect along the minimum path cost
        # For each node in neighbouringNodes
        for node in iRRTStar.neighbouringNodes:
            # Find the cost from the root
            nodeCost = iRRTStar.findPathDistance(node)
            # Add the distance between the node and the new point to the resulted node
            nodeCost += iRRTStar.distance(node, new)
            # If node and new are obstacles free and the cost is lower than the min_cost
            if not iRRTStar.isInObstacle(node, new) and nodeCost < min_cost:
                # Set the min_cost_node to this node and the min_cost this cost
                min_cost_node = node
                min_cost = nodeCost
        # Update nearestNode to min_cost_node, create a treeNode object from the new point
        iRRTStar.nearestNode = min_cost_node
        newNode = treeNode(new[0], new[1])

        # If neighbouringNodes is empty, it'll add to the original nearestNode obstacle free
        iRRTStar.addChild(newNode)

        # Plot for display
        plt.pause(0.01)
        plt.plot([iRRTStar.nearestNode.locationX, new[0]], [iRRTStar.nearestNode.locationY, new[1]], 'go', linestyle="--")

        # rewire step
        for node in iRRTStar.neighbouringNodes:
            nodeCost = min_cost
            nodeCost += iRRTStar.distance(node, new)
            if not iRRTStar.isInObstacle(node, new) and nodeCost < iRRTStar.findPathDistance(node):
                node.parent = newNode

        # If goal is found, and the projected cost is lower, then append it to path
        point = np.array([newNode.locationX, newNode.locationY])
        if iRRTStar.goalFound(point):
            projectedCost = iRRTStar.findPathDistance(newNode) + iRRTStar.distance(iRRTStar.goal, point)
            if projectedCost < iRRTStar.goalCosts[-1]:
                iRRTStar.initialPathFound = True
                iRRTStar.addChild(iRRTStar.goal)
                plt.plot([iRRTStar.nearestNode.locationX, iRRTStar.goalArray[0]], [iRRTStar.nearestNode.locationY, iRRTStar.goalArray[1]], 'go', linestyle="--")
                iRRTStar.retracePath()

                print("Goal Cost: ", iRRTStar.goalCosts)
                plt.pause(0.25)
                iRRTStar.wayPoints.insert(0, start)

                # Plot the wayPoints
                for i in range (len(iRRTStar.wayPoints) - 1):
                    plt.plot([iRRTStar.wayPoints[i][0], iRRTStar.wayPoints[i+1][0]], [iRRTStar.wayPoints[i][1], iRRTStar.wayPoints[i+1][1]], 'ro', linestyle="--")
                    plt.pause(0.01)

                # Plot ellipse
                c_best = iRRTStar.goalCosts[-1]
                iRRTStar.plotEllipse(c_best)

end_time = time.time()
print(iRRTStar.goalCosts)
print("Algorithm execution time: ", end_time - start_time)
print("The wayPoints: \n")
print(iRRTStar.wayPoints)
plt.pause(100)
