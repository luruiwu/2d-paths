import rospy

rospy.init_node("pyrobot")
import Robot
from Path import Path

p = Path()

p.addSegment([1, 0], 0)
p.addSegment([2, 1], 0.5)

# Update at 60hz until the path ends
rate = rospy.Rate(60)
while not rospy.is_shutdown() and not p.finished():
    p.execute(1.0/60.0) # Lazy fixed-rate assumption
    rate.sleep()
Robot.sendDifferentialDriveCmd(0,0)

