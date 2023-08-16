# Graph-based Planning Methods
![image](https://github.com/HendEmad/Autonomous-navigation-system/assets/91827137/6ad327b1-5c16-48e8-9c4f-f122af00e02b)

Since the algorithm explores the cells symmetrically from both the start'A' and goal node'B', it will explore the same no.of cells and edges in both cases.

![image](https://github.com/HendEmad/Autonomous-navigation-system/assets/91827137/36c68451-4d71-410b-971c-d33c42e2fc1b)

Since the algorithm is deterministic and explores the cells symmetrically from both the start node'A' and the goal node'B'. The path will be the same, but in the opposite direction.

![image](https://github.com/HendEmad/Autonomous-navigation-system/assets/91827137/f2b353db-759f-4279-a83a-7c0190ceaf15)

The grassfire algorihm explores the grid in breadth first manner, considering the neighboring nodes at each level before moving on the next level. As the grid increases, the no.of possible neighbors increases exponentially. For example, in 2D grid, no.of neighbors are 4 (up, down, left, right) and in 3D grid, no.of neighbors are 6 (left, right, up, down, forward, backward) and in 4D grid, no.of neighbors are 8, and so on..

In the context of grassfire algorith, the `level` refers to the distance of number of steps away from the starting node. Each level consists of the nodes that are exactly that distance away from the starting nodes. 
For example, in a 2D grid, the starting node would be at level 1. The nodes adjacent ti those level 1 nodes would be at level 2, and so on..

![image](https://github.com/HendEmad/Autonomous-navigation-system/assets/91827137/bdb39419-b0ad-4563-b9a4-69f3d16c1730)

In general, breadth-first search is faster than Dijkstra’s algorithm for path planning problems on a discrete grid or graph. Breadth-first search is a graph traversal algorithm that visits all the vertices of a graph or all the nodes of a tree level by level. It is used to find the shortest path between two nodes in an unweighted graph. Dijkstra’s algorithm is used to find the shortest path between two nodes in a weighted graph. It is slower than breadth-first search because it has to calculate the distance between each node and its neighbors before it can determine the shortest path
