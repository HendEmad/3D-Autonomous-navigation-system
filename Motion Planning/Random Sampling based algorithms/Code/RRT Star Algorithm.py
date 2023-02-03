import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams
import time

np.set_printoptions(precision=3, suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 15


# Tree node class
class treeNode:
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None


# RRT star algorithm
class RRTStarAlgorithm:
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numIterations, 700)  # More iterations as we need to generate multiple paths
        self.grid = grid
        self.rho = stepSize
        self.nearestDist = 10000
        self.numWayPoints = 0
        self.wayPoints = []
        self.serializedTree = []
        self.searchRadius = self.rho * 2  # Always larger than stepSize
        self.neighbouringNodes = []
        self.goalArray = np.array([goal[0], goal[1]])
        self.goalCosts = [10000]  # To keep the minimum path

    def addChild(self, treeNode):
        if treeNode.locationX == self.goal.locationX:
            # Append goal to the nearestNode's children
            self.nearestNode.children.append(self.goal)
            # Set the goal's parent to nearestNode
            self.goal.parent = self.nearestNode
        else:
            # Append this node to nearestNode's children
            self.nearestNode.children.append(treeNode)
            # Set the parent to nearestNode
            treeNode.parent = self.nearestNode

    def sampleAPoint(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point

    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho * self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1] - 1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        return point

    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        dist = self.distance(locationStart, locationEnd)
        for i in range(int(dist)):
            testPoint[0] = min(grid.shape[1] - 1, locationStart.locationX + i * u_hat[0])
            testPoint[1] = min(grid.shape[0] - 1, locationStart.locationY + i * u_hat[1])
            if self.grid[round(testPoint[1]), round(testPoint[0])] == 1:
                return True
        return False

    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        v_norm = np.linalg.norm(v)
        if v_norm < 1:
            v_norm = 1
        u_hat = v / v_norm
        return u_hat

    def findNearest(self, root, point):
        if not root:
            return
        # Find distance between root and point
        dist = self.distance(root, point)
        # If it is lower than or equal to the nearestDist
        if dist <= self.nearestDist:
            # Update nearestNode to root
            self.nearestNode = root
            # update nearestDist to the resulted dist
            self.nearestDist = dist
        # Recursive call
        for child in root.children:
            self.findNearest(child, point)

    # A function to find the neighbouring nodes ==> Specific for RRT Star algo.
    def findNeighbouringNodes(self, root, point):
        if not root:
            return
        # find distance between the root and point(dist)
        dist = self.distance(root, point)
        # Add root to neighbouringNodes if dist is less than or equal to searchRadius
        if dist <= self.searchRadius:
            self.neighbouringNodes.append(root)
        # Recursive call
        for child in root.children:
            self.findNeighbouringNodes(child, point)

    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist

    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000
        self.neighbouringNodes = []  # Because everytime we iterate, the neighbouring nodes are found from scratch(for each sampled point)

    def retracePath(self):
        self.numWayPoints = 0
        self.wayPoints = []
        goalCost = 0
        goal = self.goal
        # While goal is not the start
        while goal.locationX != self.randomTree.locationX:
            # Add 1 to numWayPoints
            self.numWayPoints += 1
            # extract the X Y location of goal in a numpy array
            currentPoint = np.array([goal.locationX, goal.locationY])
            # Insert this array to wayPoints (from the beginning)
            self.wayPoints.insert(0, currentPoint)
            # add distance between the node and its parent to goalCost
            goalCost += self.distance(goal, np.array([goal.parent.locationX, goal.parent.locationY]))
            goal = goal.parent  # set the goal to its parent
        self.goalCosts.append(goalCost)

    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += self.distance(currentNode,
                                          np.array([currentNode.parent.locationX, currentNode.parent.locationY]))
            currentNode = currentNode.parent
        return costFromRoot


# Load the grid, set the start and the goal (x, y) positions, number of iterations, step size
grid = np.load('cspace.npy')
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 750.0])
numIterations = 700
stepSize = 75
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill=False)

fig = plt.figure("RRT Star Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

# Begin
rrtStar = RRTStarAlgorithm(start, goal, numIterations, grid, stepSize)
plt.pause(2)

# RRT Star algorithm
start_time = time.time()
for i in range(rrtStar.iterations):
    # Reset the nearest values, call the resetNearestValues method
    rrtStar.resetNearestValues()
    print("Iteration: ", i)

    # Algorithm begins here
    # 1. Sample a point
    point = rrtStar.sampleAPoint()
    # 2. Find the nearest node with respect to the point
    rrtStar.findNearest(rrtStar.randomTree, point)
    # 3. Steer to point, set the returned variable to 'new'
    new = rrtStar.steerToPoint(rrtStar.nearestNode, point)
    # 4. Check if obstacle
    # if not in obstacle
    if not rrtStar.isInObstacle(rrtStar.nearestNode, new):
        rrtStar.findNeighbouringNodes(rrtStar.randomTree, new)
        min_cost_node = rrtStar.nearestNode
        min_cost = rrtStar.findPathDistance(min_cost_node)
        min_cost = min_cost + rrtStar.distance(rrtStar.nearestNode, new)

        # Connect along minimum cost path
        # for each node in neighbouringNodes
        for node in rrtStar.neighbouringNodes:
            # Find the cost from the root(findPathDistance)
            nodeCost = rrtStar.findPathDistance(node)
            # Add the distance between the node and the new point('new') to the above resulted cost
            nodeCost += rrtStar.distance(node, new)
        # If node and new are obstacles free and the cost is lower than min_cost
        if not rrtStar.isInObstacle(node, new) and nodeCost < min_cost:
            # Set the min_cost node to this node
            min_cost_node = node
            # Set the min_cost to this cost
            min_cost = nodeCost
        # Update the nearest node to min_cost_node and create a treeNode object from new point (call this newNode('new[0], new[1]'))
        rrtStar.nearestNode = min_cost_node
        newNode = treeNode(new[0], new[1])
        # If neighbouringNodes is empty, it'll add to the original the nearest node
        rrtStar.addChild(newNode)

        # Plot fot display
        plt.pause(0.01)
        plt.plot([rrtStar.nearestNode.locationX, new[0]], [rrtStar.nearestNode.locationY, new[1]], 'go', linestyle="--")

        # Rewire the tree
        # for each node in neighbouringNodes
        for node in rrtStar.neighbouringNodes:
            # set a variable: 'cost' to min_cost
            nodeCost = min_cost
            # add the distance between 'new' and node to cost
            nodeCost += rrtStar.distance(node, new)
            # if node and new are obstacles free and the cost is lower than the distance from the root and the node
            if not rrtStar.isInObstacle(node, new) and nodeCost < rrtStar.findPathDistance(node):
                # set the parent of the node to 'newNode'
                node.parent = newNode

        # if goal found and the projected cost is lower, then append to path let it sample more
        point = np.array([newNode.locationX, newNode.locationY])
        if rrtStar.goalFound(point):
            projectedCost = rrtStar.findPathDistance(newNode) + rrtStar.distance(rrtStar.goal, point)
            if projectedCost < rrtStar.goalCosts[-1]:
                rrtStar.addChild(rrtStar.goal)
                plt.plot([rrtStar.nearestNode.locationX, rrtStar.goalArray[0]], [rrtStar.nearestNode.locationY, rrtStar.goalArray[1]])
                # retrace and plot, this method finds wayPoints and cost from root
                rrtStar.retracePath()

                print("Goal cost: ", rrtStar.goalCosts)
                plt.pause(0.25)

                # Plot the wayPoints
                for i in range(len(rrtStar.wayPoints) - 1):
                    plt.plot([rrtStar.wayPoints[i][0], rrtStar.wayPoints[i+1][0]], [rrtStar.wayPoints[i][1], rrtStar.wayPoints[i+1][1]], 'ro', linestyle="--")
                    plt.pause(0.01)

end_time = time.time()
print("Goal costs: ", rrtStar.goalCosts[1:-1])
print("Time of algorithm execution: ", end_time - start_time)
print("Way points: ", rrtStar
      .wayPoints)
plt.pause(100)
