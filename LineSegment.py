import Robot
import numpy as np
from math import atan2, sqrt
import config

class LineSegment:
    def __init__(self, start, stop, velocity):
        self.start = start
        self.stop = stop

        self.setVelocityConstraints(velocity, velocity, 0)
    
    def __init__(self, start, stop, vi, vf, acc):
        self.start = start
        self.stop = stop

        self.setVelocityConstraints(vi, vf, acc)

    def setVelocityConstraints(self, vi, vf, acc):
        self.vel = vi
        self.vf = vf
        self.acc = acc

    def vector(self):
        return self.stop - self.start

    def update(self, dt):

        goalFromRobot = Robot.transformPointToRobot(self.stop)
        
        if self.acc is not 0:
            remainingLength = sqrt(goalFromRobot[0]**2 + goalFromRobot[1]**2)
            deltaVel = self.acc * dt 
            # If we're within one timestep of the goal velocity, just jump to that velocity
            # Prevents oscilations because we move above the goal within one timestep
            if abs(self.vf-self.vel) <= deltaVel:
                self.vel = self.vf
            # If we're under the target velocity
            elif self.vel < self.vf:
                self.vel += deltaVel

            if self.vf == 0:
                # How fast can we go while respecting acceleration limits for slowing down in time
                maxVelocity = sqrt(2*self.acc*remainingLength)
                # Clamp current velocity to the max possible
                if self.vel > maxVelocity:
                    self.vel = maxVelocity

        Robot.sendDifferentialDriveCmd(self.vel, goalFromRobot[1]*config.KP_HEADING)
        return abs(goalFromRobot[0]) > 0.01

