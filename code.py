#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import sys
import time
from math import pi,sqrt,cos,sin

from geometry_msgs.msg import PointStamped,PoseStamped,Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion


class Robot_Laser:
    def __init__(s,topicName):
        s.msg = None
        rospy.Subscriber(topicName,LaserScan,s.update)

    def getIndex(s,angle):
        return int(round((angle * (s.msg.angle_max - s.msg.angle_min) / s.msg.angle_increment) / 360))

    def getValue(s,angle):
        return s.msg.ranges[getIndex(angle)]

    def update(s,msg):
        s.msg = msg
        sys.stdout.write("\nNEW APCKET LASER " + str(iteration))
        sys.stdout.flush()


    def values(s):
        return s.msg.ranges

    def getAngleInc(s):
        return s.msg.angle_increment

    def getMinAngle(s):
        return s.msg.angle_min

    def valuesReady(s):
        return s.msg != None

class Robot_Odometry:
    __slots__ = ['roll', 'pitch', 'yaw']

    def __init__(s,topicName):
        s.msg = None

        s.roll = 0.0
        s.pitch = 0.0
        s.yaw = 0.0

        rospy.Subscriber(topicName,Odometry,s.update)

    def update(s,msg):
        s.msg = msg
        s.calcQuat(msg);

    def log(s):
        sys.stdout.write("\nYPR: " + str('%0.3f'%s.yaw) + " " + str('%0.3f'%s.pitch) + " " + str('%0.3f'%s.pitch) + " " + str(iteration))
        sys.stdout.flush()

    def calcQuat(s,msg):

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        (s.roll, s.pitch, s.yaw) = euler_from_quaternion(quaternion)

class Robot:
    def __init__(s,Laser,Odometry,vel_topic):
        s.laser = Laser
        s.odometry = Odometry
        s.heading = 0.0

        s.move_publisher = rospy.Publisher(vel_topic, Twist, queue_size=1)
        s.arrow = rospy.Publisher('/markerarroyo',Marker, queue_size=1)
        s.isReady = True
        rospy.on_shutdown(s.stopRobot)

    def stopRobot(s):
        s.isReady = False

    def move(s,x,y,z,ax,ay,az):
        msg = Twist()

        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z

        msg.angular.x = ax
        msg.angular.y = ay
        msg.angular.z = az

        s.move_publisher.publish(msg)

    def generateMovementVector(s):
        resX = 0.0
        resY = 0.0

        if(s.laser.valuesReady() == True):

            incAngle = s.laser.getAngleInc()
            angle = s.laser.getMinAngle()

            for dist in s.laser.values():
                resX = resX + 1 / (dist * cos(angle))
                resY = resY + 1 / (dist * sin(angle))
                angle += incAngle

            s.createMarker(resX,resY,0)


    def createMarker(s,x,y,z):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.8
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.w = 1.0

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z


        marker.text = "Soc una flexaso"
        marker.color.r = 150;
        marker.color.g = 50;
        marker.color.b = 50;
        marker.color.a = 1.0

        s.arrow.publish(marker)

    def update(s):
        s.odometry.log()
        s.generateMovementVector();

iteration = 0
if __name__ == "__main__":

    rospy.init_node("Robot_test")
    rate = rospy.Rate(100)
    laser = Robot_Laser('/kobuki/laser/scan')
    odo = Robot_Odometry('/odom')

    r = Robot(laser,odo,'/cmd_vel')

    while r.isReady:
        r.update()
        iteration = iteration + 1
        rate.sleep()
