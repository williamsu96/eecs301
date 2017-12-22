#!/usr/bin/env python
import roslib
import rospy
import time
import math
from datetime import datetime
from fw_wrapper.srv import *
from map import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        #rospy.loginfo("Get Motor Current Position %d: %f ",motor_id,resp1.val) 
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getDMSSensorValue():
    return getSensorValue(1)

def getLeftSensorValue():
    return getSensorValue(2)

def getRightSensorValue():
    return getSensorValue(5)

def isDebug() :
    return True

def isTest() :
    return True

def isEx() :
    return True

def isTimeout() :
    return False

def getTimeout() :
    return 15


def toDefaultStance():
    setMotorTargetPositionCommand(1,500)
    setMotorTargetPositionCommand(2,800)
    setMotorTargetPositionCommand(3,500)
    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(5,200)
    setMotorTargetPositionCommand(6,820)
    setMotorTargetPositionCommand(7,800)
    setMotorTargetPositionCommand(8,200)

#call before the first iteration of walkStep
def walkStance():
    setMotorTargetPositionCommand(1,500)
    setMotorTargetPositionCommand(2,500)
    setMotorTargetPositionCommand(3,800)
    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(5,200)
    setMotorTargetPositionCommand(6,700)
    setMotorTargetPositionCommand(7,700)
    setMotorTargetPositionCommand(8,200)

# each iteration takes a step forward with each leg
def walkStep():

    setMotorTargetPositionCommand(3,500)
    setMotorTargetPositionCommand(7,800)
    setMotorTargetPositionCommand(2,800)
    setMotorTargetPositionCommand(6,820)

    setMotorTargetPositionCommand(8,300)
    setMotorTargetPositionCommand(5,300)

    setMotorTargetPositionCommand(3,800)
    setMotorTargetPositionCommand(2,500)

    setMotorTargetPositionCommand(4,500)
    setMotorTargetPositionCommand(8,200)
    setMotorTargetPositionCommand(1,200)
    setMotorTargetPositionCommand(5,200)

    setMotorTargetPositionCommand(7,700)
    setMotorTargetPositionCommand(6,700)

    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(1,500)

#experimental
#in progress
def exWalkStep() :
    setMotorTargetPositionCommand(7,500)
    time.sleep(0.05)
    setMotorTargetPositionCommand(3,500)
    time.sleep(0.05)
    setMotorTargetPositionCommand(7,800)
    time.sleep(0.05)
    setMotorTargetPositionCommand(3,800)
    time.sleep(0.05)

    setMotorTargetPositionCommand(6,500)
    time.sleep(0.05)
    setMotorTargetPositionCommand(2,800)
    time.sleep(0.05)
    setMotorTargetPositionCommand(6,820)
    time.sleep(0.05)
    setMotorTargetPositionCommand(2,500)
    time.sleep(0.05)

    setMotorTargetPositionCommand(8,500)
    time.sleep(0.05)
    setMotorTargetPositionCommand(4,500)
    time.sleep(0.05)
    setMotorTargetPositionCommand(8,200)
    time.sleep(0.05)
    setMotorTargetPositionCommand(4,200)
    time.sleep(0.05)

    setMotorTargetPositionCommand(5,500)
    time.sleep(0.05)
    setMotorTargetPositionCommand(1,200)
    time.sleep(0.05)
    setMotorTargetPositionCommand(5,200)
    time.sleep(0.05)
    setMotorTargetPositionCommand(1,500)
    time.sleep(0.05)

def exWalkStance() :
    setMotorTargetPositionCommand(1,500)
    setMotorTargetPositionCommand(2,500)
    setMotorTargetPositionCommand(3,800)
    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(5,200)
    setMotorTargetPositionCommand(6,820)
    setMotorTargetPositionCommand(7,800)
    setMotorTargetPositionCommand(8,200)


# call before first iteration of walkStep
def backStance() :
    setMotorTargetPositionCommand(1,200)
    setMotorTargetPositionCommand(2,800)
    setMotorTargetPositionCommand(3,500)
    setMotorTargetPositionCommand(4,500)
    setMotorTargetPositionCommand(5,300)
    setMotorTargetPositionCommand(6,800)
    setMotorTargetPositionCommand(7,800)
    setMotorTargetPositionCommand(8,300)

# same as walkStep, but moves in opposite direction
def backStep() :

    setMotorTargetPositionCommand(1,500)
    setMotorTargetPositionCommand(5,200)
    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(8,200)

    setMotorTargetPositionCommand(6,700)
    setMotorTargetPositionCommand(7,700)

    setMotorTargetPositionCommand(1,200)
    setMotorTargetPositionCommand(4,500)

    setMotorTargetPositionCommand(2,500)
    setMotorTargetPositionCommand(6,800)
    setMotorTargetPositionCommand(3,800)
    setMotorTargetPositionCommand(7,800)

    setMotorTargetPositionCommand(5,300)
    setMotorTargetPositionCommand(8,300)

    setMotorTargetPositionCommand(2,800)
    setMotorTargetPositionCommand(3,500)

# call 6 times for ~90-degree left turn
def turnL() :
    
    setMotorTargetPositionCommand(6,700)
    setMotorTargetPositionCommand(2,800)
    setMotorTargetPositionCommand(6,820)

    setMotorTargetPositionCommand(8,300)
    setMotorTargetPositionCommand(4,500)
    setMotorTargetPositionCommand(8,200)

    setMotorTargetPositionCommand(5,300)
    setMotorTargetPositionCommand(1,500)
    setMotorTargetPositionCommand(5,200)

    setMotorTargetPositionCommand(7,700)
    setMotorTargetPositionCommand(3,800)
    setMotorTargetPositionCommand(7,800)

    setMotorTargetPositionCommand(2,500)
    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(1,200)
    setMotorTargetPositionCommand(3,500)

def turnL90() :
    for i in range(6) :
        current = datetime.now()
        elapsed = current - start
        if isDebug() :
            rospy.loginfo("Time elapsed: ")
            rospy.loginfo(elapsed.seconds)
        if isTimeout() and elapsed.seconds >= getTimeout() :
            return
        turnL()

def turnL45() :
    for i in range(3) :
        current = datetime.now()
        elapsed = current - start
        if isDebug() :
            rospy.loginfo("Time elapsed: ")
            rospy.loginfo(elapsed.seconds)
        if isTimeout() and elapsed.seconds >= getTimeout() :
            return
        turnL()

# call 6 times for ~90-degree right turn
def turnR() :
    
    setMotorTargetPositionCommand(6,700)
    setMotorTargetPositionCommand(2,500)
    setMotorTargetPositionCommand(6,820)

    setMotorTargetPositionCommand(8,300)
    setMotorTargetPositionCommand(4,200)
    setMotorTargetPositionCommand(8,200)

    setMotorTargetPositionCommand(5,300)
    setMotorTargetPositionCommand(1,200)
    setMotorTargetPositionCommand(5,200)

    setMotorTargetPositionCommand(7,700)
    setMotorTargetPositionCommand(3,500)
    setMotorTargetPositionCommand(7,800)

    setMotorTargetPositionCommand(2,800)
    setMotorTargetPositionCommand(4,500)
    setMotorTargetPositionCommand(1,500)
    setMotorTargetPositionCommand(3,800)

def turnR90() :
    for i in range(6) :
        current = datetime.now()
        elapsed = current - start
        if isDebug() :
            rospy.loginfo("Time elapsed: ")
            rospy.loginfo(elapsed.seconds)
        if isTimeout() and elapsed.seconds >= getTimeout() :
            return
        turnR()

def turnR45() :
    for i in range(3) :
        current = datetime.now()
        elapsed = current - start
        if isDebug() :
            rospy.loginfo("Time elapsed: ")
            rospy.loginfo(elapsed.seconds)
        if isTimeout() and elapsed.seconds >= getTimeout() :
            return
        turnR()

#call to turn ~180 degrees (left)
def turn180() :
    for i in range(11) :
        current = datetime.now()
        elapsed = current - start
        if isDebug() :
            rospy.loginfo("Time elapsed: ")
            rospy.loginfo(elapsed.seconds)
        if isTimeout() and elapsed.seconds >= getTimeout() :
            return
        turnL()

def isFrontBlocked() :
    DMSVal = getDMSSensorValue()
    if isDebug() and DMSVal > 1000 :
        rospy.loginfo("Front is blocked!")
    return DMSVal > 1000

def isLeftBlocked() :
    leftVal = getLeftSensorValue()
    if isDebug() and leftVal > 50 :
        rospy.loginfo("Left is blocked!")
    return leftVal > 50

def isRightBlocked():
    rightVal = getRightSensorValue()
    if isDebug() and rightVal > 30 :
        rospy.loginfo("Right is blocked!")
    return rightVal > 30

followLeft = False
#followRight = not followLeft

def walk():
    #insert sensor checks for deciding whether to turn or walk
    global followLeft

    #need to turn if front is blocked
    if isFrontBlocked() :
        if isRightBlocked() and isLeftBlocked() :
            turn180()
        elif isRightBlocked() :
            turnL90()
        elif isLeftBlocked() :
            turnR90()
        elif followLeft :
            turnR90()
        else :
            turnL90()

    # front is not blocked
    else :
        if followLeft and getLeftSensorValue() == 0 :
            turnL45()
            for i in range(3) :
                current = datetime.now()
                elapsed = current - start
                if isDebug() :
                    rospy.loginfo("Time elapsed: ")
                    rospy.loginfo(elapsed.seconds)
                if isTimeout() and elapsed.seconds >= getTimeout() :
                    return
                if not isEx() :
                    walkStep()
                else :
                    exWalkStep()
        elif not followLeft and getRightSensorValue() == 0 :
            turnR45()
            for i in range(3) :
                current = datetime.now()
                elapsed = current - start
                if isDebug() :
                    rospy.loginfo("Time elapsed: ")
                    rospy.loginfo(elapsed.seconds)
                if isTimeout() and elapsed.seconds >= getTimeout() :
                    return
                if not isEx() :
                    walkStep()
                else :
                    exWalkStep()
        else :
            for i in range(3) :
                current = datetime.now()
                elapsed = current - start
                if isDebug() :
                    rospy.loginfo("Time elapsed: ")
                    rospy.loginfo(elapsed.seconds)
                if isTimeout() and elapsed.seconds >= getTimeout() :
                    return
                if not isEx() :
                    walkStep()
                else :
                    exWalkStep()
    
def findDistance(testSet, trainSet):
    distance = 0
    for i in range(6):
        distance+= pow((testSet[i] - trainSet[i]), 2)
    return math.sqrt(distance)

traverseData = []
def loadTraverseData():
    global traverseData
    f = open('/home/rosuser/ros_workspace/src/eecs301_grp_l/traverse_data.txt','r')
    height = 0
    for lineI in f:
        height = height + 1
    traverseData = [[-1 for x in range(7)] for y in range(height)]
    rowNum = 0
    f.close()
    f = open('/home/rosuser/ros_workspace/src/eecs301_grp_l/traverse_data.txt','r')
    for line in f:
        row = eval(line)
        traverseData[rowNum] = row
        rowNum = rowNum+1

def getAction(testSet):
    distances = []
    global traverseData
    for line in traverseData:
        distance = findDistance(testSet, line)
        dp = [line, distance]
        distances.append(dp)
    distances.sort(key=lambda tup: tup[1])
    neighbors = []
    for x in range(3):
        neighbors.append(distances[x][0])
    fdr = 0
    fdf = 0
    ftl = 0
    ftr = 0
    dn = 0
    for x in range(3):
        if neighbors[x][6] == 'fdr':
            fdr = fdr+1
        elif neighbors[x][6] == 'fdf':
            fdf = fdf+1
        elif neighbors[x][6] == 'ftl':
            ftl = ftl+1
        elif neighbors[x][6] == 'ftr':
            ftr = ftr+1
        elif neighbors[x][6] == 'dn':
            dn = dn+1
    if fdr == max(fdr, fdf, ftl, ftr, dn):
        return 'fdr'
    elif fdf == max(fdr, fdf, ftl, ftr, dn):
        return 'fdf'
    elif ftl == max(fdr, fdf, ftl, ftr, dn):
        return 'ftl'
    elif ftr == max(fdr, fdf, ftl, ftr, dn):
        return 'ftr'
    elif dn == max(fdr, fdf, ftl, ftr, dn):
        return 'dn'
#Legs:
#1, 5 front left
#3, 7 back left
#2, 6 front right
#4, 8 back right

# Main function
if __name__ == "__main__":
    loadTraverseData()
    print(getAction([0,98,444,0,300,550]))
