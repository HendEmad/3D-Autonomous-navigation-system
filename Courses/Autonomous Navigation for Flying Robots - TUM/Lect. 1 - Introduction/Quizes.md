# Quiz 1
![image](https://user-images.githubusercontent.com/91827137/205477714-fb11a77d-5e0a-4235-87cf-960bfcdfabf8.png)

![image](https://user-images.githubusercontent.com/91827137/205477725-8b9a67eb-0ec0-4023-b845-b046c0e1b4e2.png)

![image](https://user-images.githubusercontent.com/91827137/205477734-a677a04c-d9fd-4017-8d89-7ad466c4c2e8.png)

# Exercise 1
### Question

![image](https://user-images.githubusercontent.com/91827137/205477995-423ecb76-99a2-4cde-9f0f-ea62bbb1a262.png)

### Answer
```
import quadrotor.command as cmd
from math import sqrt

def plan_mission(mission):

    # this is an example illustrating the different motion commands,
    # replace them with your own commands and activate all beacons
    commands  = [
        cmd.down(0.5),
        cmd.right(1),
        cmd.turn_left(45),
        cmd.forward(sqrt(2)),
        cmd.turn_right(45),
        cmd.right(1),
        cmd.turn_left(45),
        cmd.forward(sqrt(0.5)),
        cmd.turn_left(90),
        cmd.forward(sqrt(0.5)),
        cmd.turn_left(45),
        cmd.forward(1),
        cmd.turn_right(45),
        cmd.backward(sqrt(2)),
        cmd.turn_left(45),
        cmd.forward(1),
    ]

    mission.add_commands(commands)

```

### Output
```
updating setpoint, position: [[0 0 -0.5]] yaw: 0.0
updating setpoint, position: [[0 -1 -0.5]] yaw: 0.0
updating setpoint, position: [[0 -1 -0.5]] yaw: 0.785398163397
updating setpoint, position: [[1.0099 -0.010029 -0.5]] yaw: 0.785398163397
updating setpoint, position: [[1.0099 -0.010029 -0.5]] yaw: 0.0
updating setpoint, position: [[1.0199 -1.01 -0.5]] yaw: 0.0
updating setpoint, position: [[1.0199 -1.01 -0.5]] yaw: 0.785398163397
updating setpoint, position: [[1.5249 -0.515 -0.5]] yaw: 0.785398163397
updating setpoint, position: [[1.5249 -0.515 -0.5]] yaw: 2.35619449019
updating setpoint, position: [[1.0299 -0.010024 -0.5]] yaw: 2.35619449019
updating setpoint, position: [[1.0299 -0.010024 -0.5]] yaw: 3.14159265359
updating setpoint, position: [[0.029966 -3.1025e-5 -0.5]] yaw: 3.14159265359
updating setpoint, position: [[0.029966 -3.1025e-5 -0.5]] yaw: 2.35619449019
updating setpoint, position: [[1.0399 -0.98999 -0.5]] yaw: 2.35619449019
updating setpoint, position: [[1.0399 -0.98999 -0.5]] yaw: 3.14159265359
updating setpoint, position: [[0.039956 -0.98 -0.5]] yaw: 3.14159265359
done
done 145.977s 0.005067238267150073s/step
```

![image](https://user-images.githubusercontent.com/91827137/205478028-d502cc57-0908-4230-bb39-b965f4dc7786.png)
