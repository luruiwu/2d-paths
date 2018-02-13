import numpy as np
import Robot
from LineSegment import LineSegment
from ArcSegment import ArcSegment
import config

class Path:
    def __init__(self):
        self.segments = []
        self.step = 0

    def addSegment(self, point, turnRadius):
        point = np.array(point)
        if len(self.segments) is 0:
            self.segments.append(LineSegment(np.array([0,0]), point, 0, config.CONSTANT_VELOCITY, config.ACCEL))
        else:
            prev = self.segments[-1]
            # We don't want to change velocity constraints on the first segment, but
            # any addition segments need to have velocity constraints adjusted because
            # they are added with a vf of 0
            if len(self.segments) is not 1:
                prev.setVelocityConstraints(config.CONSTANT_VELOCITY, config.CONSTANT_VELOCITY, 0)

            vecB = point - prev.start

            # Arc to the goal point with a set turn radius
            # AB = The previous path vector segment 
            # C = new goal point
            # Then the sign of AB (cross) AC determines the direction of the arc
            self.segments.append(ArcSegment(point, turnRadius, np.cross(prev.vector(), vecB) > 0))
            # Add this segment assuming its the last one in the path
            self.segments.append(LineSegment(prev.stop, point, config.CONSTANT_VELOCITY, 0, config.ACCEL))

    def finished(self):
        return self.step >= len(self.segments)

    def execute(self, dt):
        if not self.segments[self.step].update(dt):
            self.step += 1
            print("Started next segment")

