import numpy as np
import math
from fractions import Fraction
from scipy.odr import *

# A list to store the landmarks for data association purpose
Landmarks = []


# First group
class featuresDetection:
    def __int__(self):
        # variables in pixel units
        self.EPSILON = 10  # Minimum orthogonal distance of a point to its line
        self.DELTA = 20  # maximum non-orthogonal distance of a point to its line
        self.SNUM = 6
        self.PMIN = 20  # number of points a line segment has to have at least
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1  # Total number of laser points
        self.LMIN = 20  # Minimum length of line segment
        self.LR = 0  # Real length of a line segment
        self.PR = 0  # The number of laser points contained in the line segment
        self.FEATURES = []

    @staticmethod
    def dist_point2point(point1, point2):
        px = (point1[0] - point2[0]) ** 2
        py = (point1[1] - point2[1]) ** 2
        return math.sqrt(px + py)

    # The distance from point to line
    @staticmethod
    def dist_point2line(params, point):
        A, B, C = params
        distance = abs(A * point[0] + B * point[1] + C) / math.sqrt(A ** 2 + B ** 2)
        return distance

    # A method takes the parameters of a line presented under the slope intercept form and using substitution would return the x and y coordinates of the two points from
    # this line in an array
    # extract two points from the line
    @staticmethod
    def line_2points(m, b):
        x = 5
        y = m * x + b
        x2 = 2000
        y2 = m * x2 + b
        return [(x, y), (x2, y2)]

    # A method that transform a line represented in the general form to the slope intercept form
    # General form to slope_intercept form
    @staticmethod
    def lineForm_G2SI(A, B, C):
        m = -A / B
        B = -C / B
        return m, B

    # slope_intercept form to general form
    @staticmethod
    def lineForm_SI2G(m, B):
        A, B, C = -m, 1, -B
        if A < 0:
            A, B, C = -A, -B, -C
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        A = A * lcm
        B = B * lcm
        C = C * lcm
        return A, B, C

    # A method to find the intersection between two lines under the general form
    # Assume the two lines will definitely intercept
    @staticmethod
    def line_intersect_general(params1, params2):
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)
        return x, y

    # Extract the line equation of the line that goes through these two extracted points
    @staticmethod
    def points_2line(point1, point2):
        # assume the slope m and the intercept b of the line are both equal to 0
        m, b = 0, 0
        # check whether the lines have the same x coordinate
        if point2[0] == point1[0]:
            # the resulting line will be vertical, that means that it can't be represented by the slope intercept form anyways
            pass
        # if it is not the case
        else:
            # calculate the slope and intercept (y2 - y1) / (x2 - x1)
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            # the intercept is done by substituting the second point in the line equation
            b = point2[1] - m * point2[0]
        return m, b

    # A method for orthogonal position of a point onto a line dealing
    # To return the point of the projection as a tuple
    @staticmethod
    def projection_point2line(point, m, b):
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = - (b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y

    # A method takes the lidar data provided as a distance and an angle and convert them to x and y coordinates
    @staticmethod
    def AD2pos(distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return int(x), int(y)

    # A method that will use the AD2pos method to know the position of the vehicle from which the lidar measurements were taken
    def laser_points_set(self, data):
        self.LASERPOINTS = []
        if not data:
            pass
        else:
            for point in data:
                # convert the received lidar data to the cartesian coordinates
                coordinates = self.AD2pos(point[0], point[1], point[2])
                # Append the coordinates to the LASERPOINTS list as an array containing two elements
                # The first point is the x and y coordinates as a tuple
                # The second is the angle of that point
                self.LASERPOINTS.append([coordinates, point[1]])
        self.NP = len(self.LASERPOINTS) - 1  # NP is the total number of laser points, that's the lidar measurements

    # For the line fitting task
    # Using the scipy module, we will implement the orthogonal line fitting
    # The orthogonal distance regression aka-odr
    # A linear function to fit against
    @staticmethod
    def linear_func(p, x):
        m, b = p
        return m * x + b

    # declare the odr function, pass the laser points as a list
    def odr_fit(self, laser_points):
        # separate the x and y coordinates
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        # create a model for fitting the data using the model method from the scipy odr file and pass the linear function to it
        linear_model = Model(self.linear_func)

        # create the real data object using our initiated data from above
        data = RealData(x, y)

        # set up the ODR with the data and the model we have created
        odr_model = ODR(data, linear_model, beta0=[0., 0.])  # beta is the initial parameters values

        # Run the regression
        # Return the m and b parameters of the line that was fitted using the data we provided
        out = odr_model.run()
        # Return the m and b parameters of the line that was fitted using the data we provided
        m, b = out.beta
        return m, b

    # A method to predict point, pass the line parameters to the point provided by the lidar measurements and the robot position at the moment these measurements were taken
    def predictPoint(self, line_params, sensed_points, robotpos):
        # get the line parameters or laser beam at this point
        m, b = self.points_2line(robotpos, sensed_points)
        # Calculate the intersection between the laser beam and the line we have fitted
        # The predicted point will be the intersection point
        # get the general form of the laser beam line since the fitted line will be provided in the general form too
        params1 = self.lineForm_SI2G(m, b)
        predx, predy = self.line_intersect_general(params1, line_params)
        return predx, predy

    # -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Second group
    '''
    1. This is done be storing all the points in a single lidar sweep as a list. So, each point have an index. We start the process from the first element of the list and we check
    whether the first endpoints make a good seed segment or not 'using the seed segment detection method'.
    2. Expand the seed segment into a line segment which is just a larger set of points. This is done using the 'seed segment growing method'.
    3. After we get our line segment, we take the last point of this segment and then, we start the process again from this point. this point will be called the 'breakpoint'.
    '''

    def seed_segment_detection(self, robot_position, break_point_ind):
        flag = True
        # Make sure that the self np variable is not negative
        self.NP = max(0, self.NP)
        # Initializing the seed segment list
        # store the seed segments we detect inside the list;; break_pint_end = 0 at the beginning of the storing
        for i in range(break_point_ind,
                       (self.NP - self.PMIN)):  # PMIN is the minimum number of points a seed segment should have
            # A list for storing the predicted points
            predicted_points_to_draw = []  # for drawing in PyGame library
            j = i + self.SNUM  # SNUM is the number of points in our seed segment
            # Fit a line through the points from i and j in the laserpoints list and return the line parameters of this loop in slope-intercept form
            m, c = self.odr_fit(self.LASERPOINTS[i:j])
            # Convert the line equation to the general form
            params = self.lineForm_SI2G(m, c)
            # Check whether every point in the seed segments of the points from i to j is satisfying the conditions for being in this seed segment
            for k in range(i, j):
                # extract the predicted position of a point
                predicted_point = self.predictPoint(params, self.LASERPOINTS[k][0], robot_position)
                # Append this position to this list
                predicted_points_to_draw.append(predicted_point)
                # measure the distance between the actual point and its predicted position
                d1 = self.dist_point2point(predicted_point, self.LASERPOINTS[k][0])
                # if d1 is larger than delta, set the flag to false
                if d1 > self.DELTA:
                    flag = False
                    break
                # If not, check the second condition which is the distance between the actual point and the fitted line
                d2 = self.dist_point2line(params, self.LASERPOINTS[k][0])
                if d2 > self.EPSILON:
                    flag = False
                    break
                # If this inner loop is over and flag is still true, store the line in the line parameter variable
            if flag:
                self.LINE_PARAMS = params
                # return a list containing all the information we need: the detected seed segment, the predicted points, the start and end indices of the seed segment i & j
                return [self.LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]
        # If the seed doesn't being extracted for some reason, return false
        return False

    # A method which takes the indices of the detected seed segment as arguments with the break_point
    def seed_segment_growing(self, indices, break_point):
        # the fitted line
        line_eq = self.LINE_PARAMS
        # the i and j indices are separated
        i, j = indices
        # The beginning(PB) and the final(PF) points in the line segment
        PB, PF = max(break_point, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        # Start growing the left side
        # While the distance between the new point PF and the line is less than EPSILON, we keep expanding
        while self.dist_point2line(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            # if PF is not bigger than the least length, fit another line. But, add the point PF to the original seed segment
            if PF > self.NP - 1:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_SI2G(m, b)
                POINT = self.LASERPOINTS[PF][0]

            PF += 1
            NEXT_POINT = self.LASERPOINTS[PF][0]

            # Calculate the distance between the current PF and the previous one
            # If the distance is bigger than GMAX, there is a gap in our seed segment which means it's probably a door or a window
            # Here, we will need to break our growing at that point
            if self.dist_point2point(POINT, NEXT_POINT) > self.GMAX:
                break
        PF -= 1

        # Grow the seed segment the right side and when it finishes. increment the PB by one
        while self.dist_point2line(line_eq, self.LASERPOINTS[PB][0]) < self.EPSILON:
            if PB < break_point:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB: PF])
                line_eq = self.lineForm_SI2G(m, b)
                POINT = self.LASERPOINTS[PB][0]
            PB -= 1
            NEXT_POINT = self.LASERPOINTS[PB][0]
            if self.dist_point2point(POINT, NEXT_POINT) > self.GMAX:
                break
        PB += 1

        # LR: The length of line segment
        LR = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        # PR: The number of points the line segment contains
        PR = len(self.LASERPOINTS[PB:PF])

        if (LR >= self.LMIN) and (PR >= self.PMIN):
            # The line segment is successfully created
            self.LINE_PARAMS = line_eq
            m, b = self.lineForm_G2SI(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_2points(m, b)
            self.LINE_SEGMENTS.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))
            # Return line segments, the two points from the final fitted line
            return [self.LASERPOINTS[PB:PF], self.two_points,
                    (self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]), PF, line_eq, (m, b)]
        else:
            return False

    # A method to convert the line feature representation to point representation
    def lineFeats2point(self):
        new_rep = []  # The new representation of the features
        for feature in self.FEATURES:
            projection = self.projection_point2line((0, 0), feature[0][0], feature[0][1])
            new_rep.append([feature[0], feature[1], projection])

        return new_rep


# A function that takes the features that are called landmarks
def landmark_association(landmarks):
    thresh = 10  # The threshold for matching new landmarks in pixels
    for l in landmarks:
        flag = False  # Indicates whether this is never seen landmark
        for i, Landmark in enumerate(Landmarks):
            # Calculate the distance between the new observed landmark and every one of those
            dist = featuresDetection.dist_point2point(l[2], Landmark[2])
            # If the distance is less than the threshold
            if dist < thresh:
                # check if the two landmarks are overlapped
                if not is_overlap(l[1], Landmark[1]):
                    continue
                else:
                    # Pop the landmark from this list and insert the new one in its place
                    Landmarks.pop(i)
                    Landmarks.insert(i, l)
                    flag = True
                    break
        # If the loop is ended without finding any match for our observed landmark
        if not flag:
            # This means that this is a new never seen landmark, and we have to append it to the list
            Landmarks.append(l)


'''
If two segments are on the same line, then if the distance between the centers of this segment is greater than the sum of each half segment. In that case, they are not overlapped. No matter what the size of the two segments is r the line slope
'''


def is_overlap(seg1, seg2):
    # calculate the centers of the two line segments, the distance between them, and the length of each one
    length1 = featuresDetection.dist_point2point(seg1[0], seg1[1])
    length2 = featuresDetection.dist_point2point(seg2[0], seg2[1])
    center1 = ((seg1[0][0] + seg1[1][0]) / 2, (seg1[0][1] + seg1[1][1]) / 2)
    center2 = ((seg2[0][0] + seg2[1][0]) / 2, (seg2[0][1] + seg2[1][1]) / 2)

    dist = featuresDetection.dist_point2point(center1, center2)
    if dist > (length1 + length2) / 2:
        return False
    else:
        return True