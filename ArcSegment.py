import Robot
import numpy as np
import config
from math import atan2

class ArcSegment:
    def __init__(self, goal, turnRadius, clockwise):
        self.goal = goal
        self.turnRadius = turnRadius
        if clockwise:
            self.clockwise = 1
        else:
            self.clockwise = -1
        self.omega = config.CONSTANT_VELOCITY / self.turnRadius 

    def update(self, dt):
        goalFromRobot = Robot.transformPointToRobot(self.goal)
        Robot.sendDifferentialDriveCmd(config.CONSTANT_VELOCITY, self.omega*self.clockwise)
        return abs(goalFromRobot[1]) > config.ARC_SEGMENT_FINISHED
