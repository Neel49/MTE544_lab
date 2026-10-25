from email import message
import numpy as np
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # Trajectories implemented here
    def trajectory_planner(self):
        trajectory = "sigmoid"

        if trajectory == "parabola":
            x = np.linspace(0.0, 1.5, 61)
            y = x ** 2
         
        elif trajectory == "sigmoid":
            x = np.linspace(0.0, 2.5, 101)
            y = 2.0 / (1+np.exp(-2.0*x)) - 1.0
        else:
            raise ValueError(f"Trajectory expected parabola or sigmoid, but recieved {trajectory}.")
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return [[xi,yi] for xi, yi in zip(x,y)]