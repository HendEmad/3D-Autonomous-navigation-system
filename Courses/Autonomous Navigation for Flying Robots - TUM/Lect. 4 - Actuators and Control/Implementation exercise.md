# The question
![image](https://user-images.githubusercontent.com/91827137/211391707-6311392f-78c1-4127-8c54-26e3d9d6c49f.png)

# The equation
![image](https://user-images.githubusercontent.com/91827137/211391894-3efb26f6-40aa-4455-a7ec-7b99d8cce1a3.png)

# The code
```
class UserCode:
    def __init__(self):
        self.Kp = 5
        self.Kd = 5
        self.last = 0
            
    def compute_control_command(self, t, dt, x_measured, x_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to compute_control_command
        :param x_measured: measured position (scalar)
        :param x_desired: desired position (scalar)
        :return - control command u
        '''
        vel = (x_measured - self.last) / dt
        u = self.Kp * (x_desired - x_measured) + self.Kd * (0 - vel)
        
        self.last = x_measured;
        
```
