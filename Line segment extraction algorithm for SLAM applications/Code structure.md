![image](https://user-images.githubusercontent.com/91827137/210156800-b78760f3-c750-46a0-b77f-6461cdb08f90.png)

The code includes a class called `featuresDetection`. This class contains the methods needed to perform the feature extraction. They have two groups:
## 1. First group:
The helper methods that perform the mathematical tasks which are:
- Calculating the distance between two points.
- Calculating the distance between a point and a line.
- Extracting point coordinates from a line given the line equation.
- Transforming a line representation under the general form to the slope-intercept foem, and also the opposite.
- Calculating the intersection between two lines.
- AD2Pos method will transform the lidar sensor readings from the angle and distance measurements to the x,y coordinates. Further mode, another method to store x,y coordinates.

***These two methods are related to the line fitting task that will be essential in the feature extraction.***

- Finally, the predicted position of a point need to be extracted in order to test the quality of the seed segment and whether it is good enough to be used or not.

![image](https://user-images.githubusercontent.com/91827137/210156778-b6f827d8-5392-4fbd-8bfd-206db672c2e3.png)

## 2. The second group:
It contains only 2 methods which are the main methods to be used:
1. The seed segment detection.
2. The seed segment growing.

***These two methods will use the methods from the first group.***
