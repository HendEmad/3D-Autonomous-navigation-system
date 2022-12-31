import math
import pygame


# Define the image method specifying the map dimensions as arguments
class buildEnvironment:
    def __init__(self, MapDimensions):
        # Initiate the PyGame instance
        pygame.init()
        self.pointCloud = []  # declare a list to store the point cloud map
        # Upload the map file
        self.externalMap = pygame.image.load('Map 1.png')
        self.mapHeight, self.mapWidth = MapDimensions
        self.MapWindowName = 'RRT Path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapWidth, self.mapHeight))  # 2D
        # Overlay the external map on the pygame created window
        self.map.blit(self.externalMap, (0, 0))
        # colors
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.white = (255, 255, 255)

    # A method to convert the sensor raw angle distance data to cartesian coordinates
    def AD2pos(self, distance, angle, robotPosition):
        x = distance * math.cos(angle) + robotPosition[0]
        y = distance * math.sin(angle) + robotPosition[1]
        return int(x), int(y)

    # A method to take the raw data and use the AD2pos method to convert them into cartesian coordinates, and then store that points in the point cloud list after checking for duplicates
    def dataStorage(self, data):
        print(len(self.pointCloud))
        for element in data:
            point = self.AD2pos(element[0], element[1], element[2])
            if point not in self.pointCloud:
                self.pointCloud.append(point)

    # Show sensor data
    def show_sensorData(self):
        self.infomap = self.map.copy()
        # draw our data on a new map
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]), int(point[1])), (255, 0, 0))