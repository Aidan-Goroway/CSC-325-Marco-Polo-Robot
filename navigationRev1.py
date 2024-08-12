#!/usr/bin/env python
import rospy, math, numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Robot:
    def __init__(self):
        self.init = False
        self.turtlesim_scan = LaserScan()
        self.direction = "no"
        self.degree = 0
        self.isForward = "no"
        self.distance = 0

    def laserCallback(self, data):
        self.turtlesim_scan.ranges = data.ranges

    def speakerCallback(self, data):
        commandL = data.data.split()
        commandL.pop(-1)
        print("command list: ", commandL)

        if len(commandL) == 2:
            self.isForward = commandL[0]
            self.distance = int(commandL[1])
        elif len(commandL) == 3:
            self.direction = commandL[0]
            self.degree = int(commandL[1])
        else:
            self.direction = commandL[0]
            self.degree = int(commandL[1])
            self.isForward = commandL[2]
            self.distance = int(commandL[3])
        

    def move(self, distance, isForward):
        """
        Move the robot in a given direction and distance.

        distance: how far we want robot to move
        isForward: does the robot go forward (as opposed to backwards)
        """
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(10)
        vel_msg = Twist()
            
        # t0: how long the turtle has moved
        t0 = rospy.get_rostime().secs
        while t0 == 0:
            t0 = rospy.get_rostime().secs

        current_distance = 0
        # If isForward is true, set the linear x value to 0.2
        # Otherwise set it to -0.2
        if isForward:
            vel_msg.linear.x = 0.2
        else:
            vel_msg.linear.x = -0.2

        while current_distance < distance:
            # Stops the robot from colliding with onstacles
            print("Current travel distance: " + str(current_distance) + "/" + str(distance))
            distance_obs = self.getMinDistance(isForward)
            print("Min distance to nearest obstacle: ", distance_obs)
            if 0.0 < distance_obs and distance_obs < 0.5:
                print("There is an obstacle: Robot is stopping.")
                break
            else:
                t1 = rospy.get_rostime().secs
                pub.publish(vel_msg)
                current_distance = 0.2 * (t1 - t0)       
                rate.sleep()
        
        vel_msg.linear.x = 0.0
        pub.publish(vel_msg)
        rate.sleep()

    # helper function
    def getMinDistance(self, isForward):
        """
        Gets the minimum distance from the robot's Lidar sensor to an obstacle
        within a 30° angle in front or back of it

        param: isForward (boolean): True if we are checking the robot's front for collision. False checks the back-cone for collision.
        
        return: The minimum distance from the nearest obstacle in the vision-cone.
        """
        laserArray = numpy.array(self.turtlesim_scan.ranges) # from 0 to 360
        dele = numpy.array([0])
        if isForward:
            leftArray = laserArray[0:15] # 0-15
            rightArray = laserArray[345:360] # 345-360
            laserArrayFront = numpy.concatenate((leftArray, rightArray)) # 30 degree angle in front
            # filter out 0 (in real world, laser data will turn INF into 0)
            laserArrayFront = numpy.setdiff1d(laserArrayFront, dele) 
            if laserArrayFront.size == 0:
                return 10
            return numpy.nanmin(laserArrayFront) 
        else:
            backArray = laserArray[165:195] # back
            laserArrayBack = numpy.setdiff1d(backArray, dele)
            if laserArrayBack.size == 0:
                return 10
            return numpy.nanmin(laserArrayBack) 

                
    def navigation(self, direction, degree, isForward, distance):
        """
        Moves the robot according to the parameters.

        directon: Which side the robot turn (left or right)
        degree: How many degrees does the robot turn?
        isForward: Does the robot move forward (or back)?
        distance: How far do we want the robot to move?
        """
        if direction == "left":
            rotate(degrees2radians(degree), False)
        elif direction == "right":
            rotate(degrees2radians(degree), True)

        if isForward == "forward":
            self.move(distance, True)
        elif isForward == "backward":
            self.move(distance, False)

    def runNavFunction(self):
        """
        Runs the navigation() function.
        """
        print("Command received.")
        self.navigation(self.direction, self.degree, self.isForward, self.distance)


def rotate(relative_angle, isClockwise):
        """
        Rotate the robot in a given angle and direction.
        """
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        outData = Twist()
        t0 = rospy.get_rostime().secs
        current_angle = 0
        rate = rospy.Rate(10)

        # The angular_speed is 15 degrees per second
        angular_speed = degrees2radians(5.0)

        if isClockwise:
            outData.angular.z = -angular_speed
        else:
            outData.angular.z = angular_speed

        while current_angle < relative_angle:
            pub.publish(outData)
            t1 = rospy.get_rostime().secs
            while t1 == 0:
                t0 = rospy.get_rostime().secs
            current_angle = angular_speed * (t1 - t0)
            rate.sleep()
        
        outData.angular.z = 0.0
        pub.publish(outData)
        rate.sleep()


# helper function
def degrees2radians(angle):
	return angle * (math.pi/180.0)


# launch a node (navigation), subscribes to the topic: scan
def moveRobot():
    """
    The main function that is called. Continuously waits for voice commands to navigate with.
    Also handles subscriptions to published topics.
    """ 
    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, myRobot.laserCallback)
    rospy.Subscriber("speakCommand", String, myRobot.speakerCallback)

    # print(myRobot.direction)
    # print(myRobot.degree)
    # print(myRobot.isForward)
    # print(myRobot.distance)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        print("Waiting for command.")
        while myRobot.direction == "no" and myRobot.isForward == "no":
            h = True
            # print("Test")
            
        myRobot.runNavFunction()
        # print(myRobot.direction)
        # print(myRobot.degree)
        # print(myRobot.isForward)
        # print(myRobot.distance)
        myRobot.direction = "no"
        myRobot.isForward = "no"
        rate.sleep()
    
    rospy.spin()


if __name__ == '__main__':
    myRobot = Robot()
    moveRobot()