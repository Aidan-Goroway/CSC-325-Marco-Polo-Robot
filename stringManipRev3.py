#!/usr/bin/env python
from audiorec import *
import rospy
from std_msgs.msg import String

def grabParams(textFromAudio):
    """Takes the text retrieved from textFromAudio and takes the important contextual
        words from it.

    Args:
        textFromAudio (string): The text (string) from the audio. Is "trained" on the
            releveant hotwords seen later in this method.

    Return:
        list: A list of relevant parameters to the robot's navigation methods.
            If an expected parameter is not aquired, a "dummy value" is placed in
            that spot instead.
    """
    textList = textFromAudio.split() # Converts text to an ordered list of words
    
    paramList = [] # distance (# meters), # rotation (# degrees), left/right, forwards/backwards
     
    if textList.count("right") > 0: # Left or Right
        paramList.append("right")
        paramList.append(rotationCheck(textList))
    elif textList.count("left") > 0:
        paramList.append("left")
        paramList.append(rotationCheck(textList))
    
    if textList.count("forward") > 0: # Forward or Backward
        paramList.append("forward")
        paramList.append(moveCheck(textList))
    elif textList.count("forwards") > 0:
        paramList.append("forward")
        paramList.append(moveCheck(textList))
    elif textList.count("backward") > 0:
        paramList.append("backward")
        paramList.append(moveCheck(textList))
    elif textList.count("backwards") > 0:
        paramList.append("backward")
        paramList.append(moveCheck(textList))
    else:
        paramList.append("noMove")

    print(paramList)
    return paramList

def rotationCheck(textList):
    """
    Checks if speaker specifies how many degrees the robot should turn.
    If so, returns a call to degreeHelper().
    If not, return 90 as a default.
    """
    if textList.count("degrees") > 0: # Degrees of rotation (30, 60, 90)
        return degreeHelper(textList[textList.index("degrees") - 1])
    elif textList.count("degree") > 0:
        return degreeHelper(textList[textList.index("degree") - 1])
    else:
        return 90


def moveCheck(textList):
    """
    Checks if the speaker specifies how far the robot should move.
    If so, return a call to distanceHelper().
    If not, return 1 as a default.
    """
    if textList.count("meters") > 0: # Distance to travel (1-5m)
       return distanceHelper(textList[textList.index("meters") - 1])
    elif textList.count("meter") > 0:
        return distanceHelper(textList[textList.index("meter") - 1])
    else:
        return 1
    
# helper functions
def degreeHelper(degree2num):
    """
    Converts the degrees (english) to a numerical format. Still a string.
    If the specified rotation is not valid, returns 90.
    """
    if degree2num == "thirty":
        return "30"
    elif degree2num == "sixty":
        return "60"
    elif degree2num == "ninety":
        return "90"
    else:
        return "90"

def distanceHelper(distance2num):
    """
    Converts the distance (english) to a numerical format. Still a string.
    If distacne not valid, returns 1 (meter).
    """
    if distance2num == "one":
        return "1"
    elif distance2num == "two":
        return "2"
    elif distance2num == "three":
        return "3"
    elif distance2num == "four":
        return "4"
    elif distance2num == "five":
        return "5"
    else:
        return "1"

def speaker():
    """
    This function handles all publishing information related to the return of grabParams.
    The bublished message is a string containing any of the 4 parameters for navigation, as long as there is at least 1.
    Does not publish if there are no valid values.
    """
    pub = rospy.Publisher("speakCommand", String, queue_size=10)
    rospy.init_node('speaker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        paramlist = grabParams(audio2text(False))
        paramStr = " ".join(map(str, paramlist))

        if paramStr == "noMove":
            rospy.loginfo("Nothing detected!")
        else:

            paramStr = paramStr + " %s" % rospy.get_time()
            rospy.loginfo(paramStr)

            pub.publish(paramStr)
            rate.sleep()

if __name__ == '__main__':
    try:
        speaker()
    except rospy.ROSInterruptException:
        pass