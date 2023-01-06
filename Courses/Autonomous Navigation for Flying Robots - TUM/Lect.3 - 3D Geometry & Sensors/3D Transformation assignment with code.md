# Question
![image](https://user-images.githubusercontent.com/91827137/211112899-e484b195-113f-4036-a948-aad6518fa4c5.png)

# Answer:
![IMG20230107005811](https://user-images.githubusercontent.com/91827137/211115418-92935f56-87e3-4a04-adba-f68a4611ced7.jpg)

# Code:
```
import numpy as np

class Pose3D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation
        
    def inv(self):
        inv_rotation = self.rotation.transpose()
        inv_translation = -np.dot(inv_rotation, self.translation)
        
        return Pose3D(inv_rotation, inv_translation)
    
    def __mul__(self, other):
        return Pose3D(np.dot(self.rotation, other.rotation), np.dot(self.rotation, other.translation) + self.translation)
    
    def __str__(self):
        return "rotation:\n" + str(self.rotation) + "\ntranslation:\n" + str(self.translation.transpose())

def compute_quadrotor_pose(global_marker_pose, observed_marker_pose):
    global_quadrotor_pose = global_marker_pose * observed_marker_pose.inv()

    return global_quadrotor_pose
```
