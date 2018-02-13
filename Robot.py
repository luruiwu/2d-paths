import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion
from math import sin, cos

currentHeading = 0
robotLocation = np.array([0,0])

velocityCommander = rospy.Publisher("cmd_velocity", Twist, queue_size=10)

def updateOdom(odom):
    global robotLocation
    global currentHeading
    robotLocation = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
    orientation = euler_from_quaternion((odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w))
    currentHeading = orientation[2]

def getTime():
    return rospy.get_time()

def getAngle():
    return currentHeading

def getPosition():
    return robotLocation

def transformPointToRobot(point):
    p = point - getPosition()
    heading = getAngle()
    rot = np.array([[cos(heading), -sin(heading)], [sin(heading), cos(heading)]])
    return np.matmul(p, rot)

def sendDifferentialDriveCmd(v, omega):
    cmd = Twist(linear=Vector3(v, 0, 0), angular=Vector3(0, 0, omega))
    velocityCommander.publish(cmd)


odomSub = rospy.Subscriber("/odom", Odometry, updateOdom)
