![image](https://user-images.githubusercontent.com/91827137/205767588-88e1715d-f0cc-40c2-a6e6-c689f4deeb0f.png)

![image](https://user-images.githubusercontent.com/91827137/205767676-51db937e-e391-41da-8867-5946597638a0.png)

![image](https://user-images.githubusercontent.com/91827137/205769991-f3b2a50a-2394-42c5-ad6a-b7fb76ae9310.png)

![image](https://user-images.githubusercontent.com/91827137/205770486-f7b6f404-9349-4753-8598-43126ccddd04.png)

![image](https://user-images.githubusercontent.com/91827137/205770887-7d3b1642-c1a2-4dc3-a107-3be2af909c2c.png)

## Regarding the global to local transformation problem:
There's no need to worry about angular velocity in this problem, because the yaw of the robot in global coordinates is available in navdata.rotZ.
One of the homework problems is to see if the velocity is given in local or global coordinates. Once you have the velocity in local coordinates you can use a rotation to convert to the velocity in the global coordinates. (I think it is best to see the velocity in the local frame as the motion of the local frame expressed in the local frame's own coordinate system).
When you have the velocity in global coordinates, multiply the velocity by the time step (dt) and this gives you the distance the robot moves in the global frame.
Add this distance to self.position and you are done!
The local frame moves with respect to the global frame, and the global frame moves with respect to the local frame, and there is a symmetric relationship between the two motions.
