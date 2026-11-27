
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        if hasattr(self, "m_utilites"):
            self.m_utilites.destroy_node()
        self.m_utilites=mapManipulator(filename_=self.mapName, laser_sig=0.45)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # Convert the cell pixels into the cartesian coordinates
        
        # stop this attempt if either point is outside the usable map
        width, height = self.costMap.shape[1], self.costMap.shape[0]
        for x, y in [startPose, endPose]:
            if not (0 <= x < width and 0 <= y < height) or self.costMap[y, x] > 0.8:
                return None
        cells = search(self.costMap, startPose, endPose)
        if cells is None or cells[-1] != endPose:
            return None
        Path = list(map(self.m_utilites.cell_2_position, cells))



        # return the path as list of [x,y]
        return Path




if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
