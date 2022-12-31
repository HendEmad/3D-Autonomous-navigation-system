import pygame
import math
import numpy as np


# Add noise to the simulated sensor
# This is done by taking a random value in the vicinity of the actual measurement
# For example, if the sensor output was 5 meters, we sample a random value from a gaussian distribution where:
# - The 5 meters is the mean & - variance is the uncertainty value we provide
def uncertainty_add(distance, angle, sigma):
    # arrange the data in numpy array
    mean = np.array([distance, angle])
    # Use sigma to create the covariance matrix which is equal to 0 except for the diagonal since the noise of the distance and angles are not correlated.
    # Don't be correlated means that they don't affect each other.
    covariance = np.diag(sigma ** 2)
    # pass the mean and covariance in the multivariate normal method in order to get the noisy values
    distance, angle = np.random.multivariate_normal(mean, covariance)
    # make sure we don't get negative values using max function
    distance = max(distance, 0)
    angle = max(angle, 0)
    # Return measurements as array
    return [distance, angle]


class LaserSensor:
    def __init__(self, Range, map, uncertainty):
        self.Range = Range
        self.speed = 4  # in case we wanted to set a fixed speed for sensor rotations(rounds/second)
        self.sigma = np.array(
            [uncertainty[0], uncertainty[1]])  # Sensor measurement noise, 0 for distance and 1 is for angle
        self.position = (0, 0)  # Robot initial position
        self.map = map
        # Getting window dimensions in the top right corner
        self.width, self.height = pygame.display.get_surface().get_size()
        # A list to store the points cloud
        self.sensedObstacles = []

    # Compute the distance to the obstacle
    def distance(self, obstaclePosition):
        px = (obstaclePosition[0] - self.position[0]) ** 2
        py = (obstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(px + py)

    # Sense nearby obstacles
    ''' This is done by using sampling. We will extend from the position of the robot a straight line of segment of length equals to the sensor range and along this line
    segment, we will take a number of samples from the environment. If the color of this sample is black, this means an obstacle is reached reached which means we don't 
    need to sample the rest of the line segment. If no obstacles was found, we consider this direction within the sensor range to be obstacle free.
    We repeat this operation while increasing the angle of the line segment until a rotation (360/ ....) is achieved.'''

    # Each point is represented with a tuple containing its distance from the robot and the angle in which it was found
    # For 360° lidar
    def sense_obstacles(self):
        data = []
        # Coordinates of the robot (sensor)
        x1, y1 = self.position[0], self.position[1]
        for angle in np.linspace(0, 2 * math.pi, 60, False):
            # Coordinates that represent the end of the line segment
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            # Sampling step --> getting a random points between each two points
            for i in range(0, 100):
                u = i / 100
                # simple interpolation formula
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                # if the calculated point is within the window
                if 0 < x < self.width and 0 < y < self.height:
                    # extract the color of that exact point from the map
                    color = self.map.get_at((x, y))
                    # if the color is black, calculate the distance of that point from the robot
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance((x, y))
                        # add noise to the output
                        output = uncertainty_add(distance, angle, self.sigma)
                        # append the new position to the output array
                        output.append(self.position)
                        # Append the output list to the data list
                        # Store the measurements
                        data.append(output)
                        break
        # When the sensor complete its turn, return the data to be drawn in the map
        if len(data) > 0:
            return data
        else:
            return False

# Drawing the sensor data is the responsibility of the build environment class in the env file
# In order to draw the data as a part of a map, we will build the point cloud map in the first place which will be represented by a list of 2d coordinates.
