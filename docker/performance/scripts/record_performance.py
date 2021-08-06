#Author: Paul Smith
#Date: 8-6-21
#This program parses log files recoded during a given simulation and outputs the recorded perforamnce data to a single text file.

from scipy import interpolate
from math import sqrt
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import re
import sys

def NavTimes(rosoutLogFileLocation):
    """Return dictionary with total navigation time of turtlebot and count messages indicative of performance"""
    log = open(rosoutLogFileLocation, "r")
    lines = log.readlines()

    #lines that could also potentially be counted:
    #'Passing new path'
    #'Activating spin'
    #'Activating back_up'
    #'Activating wait'
    navBegin = "Received follow waypoint request"
    navComplete = "Completed all"
    controlLoopMissed = "Control loop missed its desired rate"
    waitMessage = "Activating wait"

    navEndTimestamp = 0
    completedAllWaypoints = "Dnf" #short for did not finish
    controlLoopMissedCounter = 0
    waitCounter = 0
    for line in lines:
        
        if navBegin in line:
            navBeginTimestamp = int(line[11:30])

        if navComplete in line:
            completedAllWaypoints = "Finished all waypoints!"
            navEndTimestamp = int(line[11:30])

        if controlLoopMissed in line:
            controlLoopMissedCounter += 1

        if waitMessage in line:
            waitCounter += 1

    #if navigation did not complete, simply use the last timestamp to estimate total navigation time 
    if navEndTimestamp == 0:
        line = lines[-2]
        navEndTimestamp = int(line[11:30]) 

    navTime = navEndTimestamp - navBeginTimestamp
    
    log.close()

    navDict = {}    
    navDict["navTime"] = navTime / 1000000000 #convert from nanoseconds to seconds
    navDict["completedAllWaypoints"] = completedAllWaypoints
    navDict["controlLoopMissedCounter"] = controlLoopMissedCounter
    navDict["waitCounter"] = waitCounter

    return navDict

def ParseOdomLog(odomLogLocation):
    """Return a dataframe of poses recorded from /odom by rosbag"""
    f = open(odomLogLocation, "r")
    odomLog = f.read()

    f.close()

    odomPoses = re.findall("position=geometry_msgs\.msg\.Point\(x=(-?\d+\.\w*-?\d*),\sy=(-?\d+\.\w*-?\d*),\sz=(-?\d+\.\w*-?\d*)\)", odomLog)
    odomTimestamps = re.findall("Timestamp:\s(\d+)\sMessage:", odomLog)

    odomPosesDf = pd.DataFrame(data=odomPoses, index=odomTimestamps, columns=['x','y','z'], dtype=np.float64)
    odomPosesDf.drop(columns=['z'], inplace=True)
    odomPosesDf.index = odomPosesDf.index.astype("int64")
    return odomPosesDf

#parsing out 'actual' robot coordinates from the gazebo log
#link to description of gazebo pose datatype: http://osrf-distributions.s3.amazonaws.com/gazebo/api/1.3.0/classgazebo_1_1math_1_1Pose.html#aa37a5a1bf0acfcff3e0cb11951b48a96

def ParseGazeboLog(gazeboLogLocation, correctNumberOfDigits):
    """Return a dataframe of logged gazebo poses timestamped by wall time"""
    f = open(gazeboLogLocation, "r")
    gazeboLog = f.read()
    f.close()

    turtlebotPoses = re.findall("<model\sname='turtlebot3_burger'><pose>(-?\d+.?\d*)\s(-?\d+.?\d*)\s(-?\d+.?\d*)\s(-?\d+.?\d*)\s(-?\d+.?\d*)\s(-?\d+.?\d*)\s", gazeboLog)
    turtlebotTimestamps = re.findall("<wall_time>(\d+)\s(\d+)</wall_time>", gazeboLog)
    
    #concatenate timestamp groups into one time because gazebo has a space in between for some reason
    fixedTimestamps = []
    for time in turtlebotTimestamps:
        fixedTimestamps.append(time[0]+time[1]) 
    
    turtlebotPosesDf = pd.DataFrame(data=turtlebotPoses, index=fixedTimestamps, columns=['x','y','z','roll','pitch','yaw'], dtype=np.float64)
    turtlebotPosesDf.drop(columns=['roll','pitch'], inplace=True)
    turtlebotPosesDf.index = turtlebotPosesDf.index.astype("int64")
    
    #drop rows without the right number of digits
    for i in turtlebotPosesDf.index:
        if len(str(i)) < correctNumberOfDigits:
            turtlebotPosesDf.drop(index=i, inplace=True)

    return turtlebotPosesDf

def AverageCpuUtilization(topLogLocation):
    """Calculates CPU utilization based on formula found on https://support.site24x7.com/portal/en/kb/articles/how-is-cpu-utilization-calculated-for-a-linux-server-monitor"""
    f = open(topLogLocation, "r")
    topLog = f.read()
    f.close()
    
    cpuId = re.findall("(\d+\.\d+)\sid,", topLog)
    
    total = 0
    for i in cpuId:
        total += (100 - float(i)) 
    
    totalCpuUtilization = total / len(cpuId)
    return totalCpuUtilization

def InterpolateGazebo(odomDf, gazeboXInterpolationFunction, gazeboYInterpolationFunction):
    """Use scipy interpolation functions to interpolate the gazebo poses using the recorded /odom timestamps"""
    interpolatableOdomIndices = []
    for odomIndex in odomDf.index:
        try:
            gazeboXInterpolationFunction(odomIndex)
            gazeboYInterpolationFunction(odomIndex)
            interpolatableOdomIndices.append(odomIndex)
        except:
            pass

    interpolatedGazeboXData = gazeboXInterpolationFunction(interpolatableOdomIndices)
    interpolatedGazeboYData = gazeboYInterpolationFunction(interpolatableOdomIndices)

    d = {'x': interpolatedGazeboXData, 'y': interpolatedGazeboYData}
    interpolatedGazeboDf = pd.DataFrame(index=interpolatableOdomIndices, data=d)
    return interpolatedGazeboDf

def CalculateDistanceFromGoal(tfLogLocation, turtlebotPosesDf, goalCoordinate):
    """T"""
    f = open(tfLogLocation, "r")
    tfLog = f.read()

    f.close()
    
    #The gazebo/odom logs are on a different map frame than /map, so we must find how to transform the two by parsing /tf
    #We're only interested in the final transform, so parsing every transform into a DataFrame only to look at the last index is probably an inefficient solution, but the best I could come up with. 
    mapToOdomTransformations = re.findall("child_frame_id='odom',\stransform=geometry_msgs\.msg\.Transform\(translation=geometry_msgs\.msg\.Vector3\(x=(-?\d+\.\w*-?\d*),\sy=(-?\d+\.\w*-?\d*),\sz=(-?\d+\.\w*-?\d*)\)", tfLog)
    mapTransformsDf = pd.DataFrame(data=mapToOdomTransformations, columns=['x','y','z'], dtype=np.float64)
    
    #transform gazebo/odom frame to map frame for only the last coordinate
    x = turtlebotPosesDf.iloc[-1].x + mapTransformsDf.iloc[-1].x
    y = turtlebotPosesDf.iloc[-1].y + mapTransformsDf.iloc[-1].y
    
    distance = sqrt((goal[0] - x)**2 + (goal[1] - y)**2)

    return distance

def CalculateTotalChangeInYaw(gazeboPoseDf):
    """Calculates the sum of the absolute change in rotation by the turtlebot.
    Using navTime calculated from rosout log instead of Gazebo indices due to 
    the gazebo log starting earlier and stopping later than the actual navigation"""
    #derivative = gazeboPoseDf.yaw.diff().abs() / gazeboPoseDf.index.to_series().diff()
    #derivative.dropna(inplace=True)
    #return derivative.sum()

    return gazeboPoseDf.yaw.diff().abs().sum()

def LogExperimentData(logDir, outputFile, header, goal):
    """Appends experiment data to a file"""
    
    #assigning log file locations to variables
    rosoutLog = logDir + "rosout_log.txt"
    gazeboLog = logDir + "gazebo_log.txt"
    odomLog = logDir + "odom_log.txt"
    topLog = logDir + "top-cpu-summary.txt"
    tfLog = logDir + "tf_log.txt"
    
    #parse /odom and gazebo logs
    odomPoseDf = ParseOdomLog(odomLog)
    gazeboPoseDf = ParseGazeboLog(gazeboLog, len(str(odomPoseDf.index[0])))
    
    #interpolate gazebo poses at /odom timestamps using scipy interpolation function so we can easily find the differences between the two plots 
    gazeboXInterpolationFunction = interpolate.interp1d(gazeboPoseDf.index, gazeboPoseDf['x'])
    gazeboYInterpolationFunction = interpolate.interp1d(gazeboPoseDf.index, gazeboPoseDf['y'])
    
    interpolatedGazeboDf = InterpolateGazebo(odomPoseDf, gazeboXInterpolationFunction, gazeboYInterpolationFunction)

    manhattanDistanceDf = abs(interpolatedGazeboDf - odomPoseDf)
    manhattanDistanceDf.dropna(inplace=True)
    
    #log recorded performance metrics
    log = open(outputFile, "a")
    log.write("\n{}\n".format(header))
    
    navDict = NavTimes(rosoutLog)
    log.write("Navigation time: {} seconds\n".format(navDict["navTime"]))
    log.write("Control loop missed desired rate {} times\n".format(navDict["controlLoopMissedCounter"]))
    log.write("Completed navigation? {}\n".format(navDict["completedAllWaypoints"]))
    log.write("Turtlebot activated wait {} times\n".format(navDict["waitCounter"]))

    cpuAvg = AverageCpuUtilization(topLog)
    log.write("Average CPU utilization = %.2f%%\n" % (cpuAvg))
    
    averageXError = manhattanDistanceDf['x'].mean()
    log.write("Average x localization error: {}\n".format(averageXError))

    averageYError = manhattanDistanceDf['y'].mean()
    log.write("Average y localization error: {}\n".format(averageYError))

    distanceFromGoal = CalculateDistanceFromGoal(tfLog, odomPoseDf, goal)
    log.write("Final distance from goal: {} meters\n".format(distanceFromGoal))

    totalChangeInYaw = CalculateTotalChangeInYaw(gazeboPoseDf)
    totalChangeInYawDegrees = totalChangeInYaw * 180/np.pi
    log.write("Total change in yaw:  %.3f degrees\n" % (totalChangeInYawDegrees))
    #log.write("In radians: %.3f radians\n" % (totalChangeInYaw))
    log.write("Change in yaw per second: %.3f deg/s\n" % (totalChangeInYawDegrees / navDict["navTime"]))
    #log.write("In radians: %.3f rad/s\n" % (totalChangeInYaw / navDict["navTime"]))

if __name__ == "__main__":
    logDir = str(sys.argv[1])
    outputFile = str(sys.argv[2])
    logHeader = str(sys.argv[3])
    goal = (2.5467472076416016, -0.03146596997976303)

    LogExperimentData(logDir, outputFile, logHeader, goal)
