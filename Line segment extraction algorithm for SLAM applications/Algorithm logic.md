The process begins with the batch of 2D points obtained from a single 360-lidar scan.
# The first step
![Screenshot (1068)](https://user-images.githubusercontent.com/91827137/210156028-682991d6-1ad7-4438-8611-508eb2a4774e.png)

Detect some seed segments which are a group of successive points that have the ability to produce a line segment. There is some strict conditions have to be placed: 

At first, we fit a straignt line through these points. For a point to be included in out seed segment, it has to satisfy these Two requirements:

![Screenshot (1069)](https://user-images.githubusercontent.com/91827137/210156055-73206278-a4fa-46f6-8895-48f39a8972eb.png)

1. The distance from any point in the seed segment to the fitted straight line should be less than a given threshold EPSILON.

![Screenshot (1070)](https://user-images.githubusercontent.com/91827137/210156079-9b132b29-fec2-4fcb-97f4-e3d2bde179d0.png)


2. The distance from every point in the seed segment to its predicted position also should be less thsn a given threshold DELTA.

_**Knowing that the predicted position of the point is the intersection between the ftting line and the lidar sensor beam.**_

![image](https://user-images.githubusercontent.com/91827137/210156458-aefcfbcc-2c4d-4bb9-a611-2c16ac73469a.png)

Points that satisfy the two requirements will be considered as a seed segment.

# The second step
![image](https://user-images.githubusercontent.com/91827137/210156516-8bbe21d1-e9c4-4ddd-bc53-1df938eb3996.png)

This step is to grow the seed segemtn to include more points. To achieve this:
- The seeded region algorithm is used which is a widely used algorithm in image segmentation. However, it can be more effictive if used with lidar data producing the satisfactory performance.

_**Typically, region growing examins neighbouring pixels of initial seed points and determines whether the pixel neighbours should be added to the region**_

- The points in the same line segment tend to have the same distance from it, this property can be utilized. The seed segment will be grown typically from both ends while measuring the distance of every newely added node to the line and then refitting that line after adding the point.

_**This process is done from both sides until one termination condition is met**_

- There are three termination conditions in that algorithm:
    * The first condition --> Over growing: If the region is over grown from one side, then the growing process will be halted in that side and will continue in the other side.
      ![image](https://user-images.githubusercontent.com/91827137/210156543-cc6ea866-cec5-498c-8a65-57b5fe14363b.png)
    * The second one --> Distance threshold: if the distance from a point to the line is bigger than the threshold, then growing process will stop at the corresponding side.
    ![image](https://user-images.githubusercontent.com/91827137/210156580-17cb5afb-914b-4852-bbbf-93ecdf30929a.png)
    *  The third: When the line segment satisfies the minimum length threshold, then the whole process of the region growing will be stopped and the line segment will be returned.
    *  ![image](https://user-images.githubusercontent.com/91827137/210156612-4c0101fe-d10e-4fd6-af43-91312e23a8b3.png)

# The third step:
Unify the overlapping line segments to produce a larger one.

# The last step:
The end points generation, this is achieved ny ptojecting the outermost points to to the fitting line and that will be the endpoints of the line segment.

-------
The whole process can be iterated until the iseful features in that specific lidar scan are extracted.

Some of the line segments that we extracted can be used as landmarks that will help in the localization process of the autonomous vehichle in its environment. BUT this will need some filterations (fusion).

There is also, a data association problem to deal with because it doesn't matter if you detect a landmark without being able to recognize it in another observation.
