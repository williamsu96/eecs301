#!/usr/bin/env python
import roslib
import rospy
import time
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

mp = EECSMap()

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
    return getSensorValue(5)

def getRightSensorValue():  
    return getSensorValue(2)

def logSensors() :
    rospy.loginfo("DMS Sensor: "+str(getDMSSensorValue()))
    rospy.loginfo("Left Sensor: "+str(getLeftSensorValue()))
    rospy.loginfo("RightSensor: "+str(getRightSensorValue()))
    

def isDebug() :
    return True

def isFrontBlocked() :
    DMSVal = getAverageDMSSensorValue()
    if isDebug() and DMSVal > 1100 :
        rospy.loginfo("Front is blocked!")
    return DMSVal > 1100

def isLeftBlocked() :
    leftVal = getAverageLeftSensorValue()
    if isDebug() and leftVal > 40 :
        rospy.loginfo("Left is blocked!")
    return leftVal > 40

def isRightBlocked():
    rightVal = getAverageRightSensorValue()
    if isDebug() and rightVal > 40 :
        rospy.loginfo("Right is blocked!")
    return rightVal > 40

def driveTurnCW() :
    setMotorWheelSpeed(10, 128)
    setMotorWheelSpeed(9, 256)
    setMotorWheelSpeed(10, 256)

def slowDriveTurnCW() :
    setMotorWheelSpeed(10, 64)
    setMotorWheelSpeed(9, 128)
    setMotorWheelSpeed(10, 128)

def slowDriveTurnCCW() :
    setMotorWheelSpeed(10, 1024+64)
    setMotorWheelSpeed(9, 1024+128)
    setMotorWheelSpeed(10, 1024+128)

def driveTurnCCW() :
    setMotorWheelSpeed(10, 1024+128)
    setMotorWheelSpeed(9, 1280)
    setMotorWheelSpeed(10, 1280)

def driveForward() :
    setMotorWheelSpeed(10,(1536 - 1024)/2 + 1024)
    setMotorWheelSpeed(9, 542)
    setMotorWheelSpeed(10, 1536)
    
def driveReverse() :
    setMotorWheelSpeed(10,512/2)
    setMotorWheelSpeed(9, 1566)
    setMotorWheelSpeed(10, 512)

def haltDrive() :
    setMotorWheelSpeed(9,(542)/2)
    setMotorWheelSpeed(10, 0)
    setMotorWheelSpeed(9, 0)

def haltDriveReverse() :
    setMotorWheelSpeed(9,(1566-1024)/2 + 1024)
    setMotorWheelSpeed(10, 0)
    setMotorWheelSpeed(9, 0)

def haltDriveOld() :
    setMotorWheelSpeed(10,(1536 - 1024)/2 + 1024)
    setMotorWheelSpeed(9, 0)
    setMotorWheelSpeed(10, 0)


numTrials = 5
def getAverageLeftSensorValue() : 
    tempValue = 0
    for i in range(numTrials) :
        tempValue += getLeftSensorValue()
    tempValue = tempValue / numTrials
    return tempValue

def getAverageRightSensorValue() : 
    tempValue = 0
    for i in range(numTrials) :
        tempValue += getRightSensorValue()
    tempValue = tempValue / numTrials
    return tempValue

def getAverageDMSSensorValue() : 
    tempValue = 0
    for i in range(numTrials) :
        tempValue += getDMSSensorValue()
    tempValue = tempValue / numTrials
    return tempValue

def dudfixErrorPreTurn() :
    RIGHT = 1
    LEFT = -1
    DONE = 0
    mode = LEFT
    baseRightIR = getAverageRightSensorValue()
    baseLeftIR = getAverageLeftSensorValue()
    while mode != DONE :
        if mode == LEFT :
            fineTurnCCW()
            currRightIR = getAverageRightSensorValue()
            currLeftIR = getAverageLeftSensorValue()
            if currRightIR >= baseRightIR and currLeftIR >= baseLeftIR :
                baseRightIR = currRightIR
                baseLeftIR = currLeftIR
            else :
                fineTurnCW()
                mode = RIGHT
        if mode == RIGHT :
            fineTurnCW()
            currRightIR = getAverageRightSensorValue()
            currLeftIR = getAverageLeftSensorValue()
            if currRightIR >= baseRightIR and currLeftIR >= baseLeftIR :
                baseRightIR = currRightIR
                baseLeftIR = currLeftIR
            else :
                fineTurnCCW()
                mode = DONE
#RIGHT = 1
#LEFT = -1
def fixErrorPreTurn(MODE) :
    RIGHT = 1
    LEFT = -1
    if(MODE==RIGHT) :
        prevIR = getAverageRightSensorValue()
        fineTurnCCW()
        currentIR = getAverageRightSensorValue()
        while currentIR > prevIR :
            prevIR = currentIR
            fineTurnCCW()
            currentIR = getAverageRightSensorValue()
        prevIR = getAverageRightSensorValue()
        fineTurnCW()
        currentIR = getAverageRightSensorValue()
        while getAverageRightSensorValue() > prevIR :
            prevIR = currentIR
            fineTurnCW()
            currentIR = getAverageRightSensorValue()
        fineTurnCCW()
    if(MODE==LEFT) :
        prevIR = getAverageLeftSensorValue()
        fineTurnCCW()
        currentIR = getAverageLeftSensorValue()
        while currentIR > prevIR :
            prevIR = currentIR
            fineTurnCCW()
            currentIR = getAverageLeftSensorValue()
        prevIR = getAverageLeftSensorValue()
        fineTurnCW()
        currentIR = getAverageLeftSensorValue()
        while getAverageLeftSensorValue() > prevIR :
            prevIR = currentIR
            fineTurnCW()
            currentIR = getAverageLeftSensorValue()
        fineTurnCCW()

turn90_t = 1.975
prevLeft = getAverageLeftSensorValue()
prevRight = getAverageRightSensorValue()
#change these
def driveTurnR90() :
    
    global prevLeft
    global prevRight
    currLeft = getAverageLeftSensorValue()
    currRight = getAverageRightSensorValue()

    #if currLeft > 40 and currLeft - prevLeft > 40 and prevLeft > 40:
    #    fineTurnCW()
    #elif prevLeft > 40 and currLeft > 40 and prevLeft - currLeft > 40 : 
    #    fineTurnCCW()
    #elif prevRight > 40 and currRight > 40 and prevRight - currRight > 40 :
    #    fineTurnCW() 
    #elif currRight > 40 and currRight - prevRight > 40 and prevRight > 40: 
    #    fineTurnCCW()

    #if currLeft > 10 : 
        #fixErrorPreTurn(-1)

    driveTurnCW()
    #1.73
    time.sleep(1.63)
    haltDrive()
    prevLeft = getAverageLeftSensorValue()
    prevRight = getAverageRightSensorValue()


    #if prevLeft > 10 : 
        #fixErrorPreTurn(-1)

def driveTurnL90() :
        
    global prevLeft
    global prevRight
    currLeft = getAverageLeftSensorValue()
    currRight = getAverageRightSensorValue()
    #if currLeft > 40 and currLeft - prevLeft > 40 and prevLeft > 40:
    #    fineTurnCW()
    #elif prevLeft > 40 and currLeft > 40 and prevLeft - currLeft > 40 : 
    #    fineTurnCCW()
    #elif prevRight > 40 and currRight > 40 and prevRight - currRight > 40 :
    #    fineTurnCW() 
    #elif currRight > 40 and currRight - prevRight > 40 and prevRight > 40: 
    #    fineTurnCCW()

    #if currRight > 10 : 
        #fixErrorPreTurn(1)

    driveTurnCCW()
    #1.84
    time.sleep(1.74)
    haltDrive()
    prevLeft = getAverageLeftSensorValue()
    prevRight = getAverageRightSensorValue()

    #if prevRight > 10 : 
        #fixErrorPreTurn(1)

def driveTurn180() :
    global prevLeft
    global prevRight
    currLeft = getAverageLeftSensorValue()
    currRight = getAverageRightSensorValue()
    #if currLeft > 40 and currLeft - prevLeft > 40 and prevLeft > 40:
    #    fineTurnCW()
    #elif prevLeft > 40 and currLeft > 40 and prevLeft - currLeft > 40 : 
    #    fineTurnCCW()
    #elif prevRight > 40 and currRight > 40 and prevRight - currRight > 40 :
    #    fineTurnCW() 
    #elif currRight > 40 and currRight - prevRight > 40 and prevRight > 40: 
    #    fineTurnCCW()

    #if currRight > 10 : 
        #fixErrorPreTurn(1)
    #if currLeft > 10 : 
        #fixErrorPreTurn(-1)

    driveTurnCW()
    time.sleep(3.62)
    haltDrive()
    prevLeft = getAverageLeftSensorValue()
    prevRight = getAverageRightSensorValue()

    #if prevRight > 10 : 
        #fixErrorPreTurn(1)
    #if prevLeft > 10 : 
        #fixErrorPreTurn(-1)


#set this to the starting direction of the robot
curDir = DIRECTION.South
#Tries to turn either 90 or 180 degrees to change to specified direction
def turnDir(trgDir) :
    global curDir

    if isDebug() :
        if curDir == DIRECTION.West :
            rospy.loginfo("Current Direction: West")
        elif curDir == DIRECTION.North :
            rospy.loginfo("Current Direction: North")
        elif curDir == DIRECTION.East :
            rospy.loginfo("Current Direction: East")
        elif curDir == DIRECTION.South :
            rospy.loginfo("Current Direction: South")
            
        if trgDir == DIRECTION.West :
            rospy.loginfo("Target Direction: West")
        elif trgDir == DIRECTION.North :
            rospy.loginfo("Target Direction: North")
        elif trgDir == DIRECTION.East :
            rospy.loginfo("Target Direction: East")
        elif trgDir == DIRECTION.South :
            rospy.loginfo("Target Direction: South")

    if curDir == DIRECTION.West :
        if trgDir == DIRECTION.North :
            driveTurnR90()
        elif trgDir == DIRECTION.South :
            driveTurnL90()
        elif trgDir == DIRECTION.East :
            driveTurn180()
    elif curDir == DIRECTION.North :
        if trgDir == DIRECTION.West :
            driveTurnL90()
        elif trgDir == DIRECTION.South :
            driveTurn180()
        elif trgDir == DIRECTION.East :
            driveTurnR90()
    elif curDir == DIRECTION.East :
        if trgDir == DIRECTION.North :
            driveTurnL90()
        elif trgDir == DIRECTION.South :
            driveTurnR90()
        elif trgDir == DIRECTION.West :
            driveTurn180()
    elif curDir == DIRECTION.South :
        if trgDir == DIRECTION.North :
            driveTurn180()
        elif trgDir == DIRECTION.West :
            driveTurnR90()
        elif trgDir == DIRECTION.East :
            driveTurnL90()
    curDir = trgDir

def forwardErrorCheck() :
    global prevLeft
    global prevRight
    
    currDMSReading = getAverageDMSSensorValue()
    if currDMSReading > 1000 and currDMSReading < 1900 : 
        currLeft = getAverageLeftSensorValue()
        currRight = getAverageRightSensorValue()
        if currLeft > 40 and currLeft - prevLeft > 40 and prevLeft > 40:
            fineTurnCW()
        elif prevLeft > 40 and currLeft > 40 and prevLeft - currLeft > 40 : 
            fineTurnCCW()
        elif prevRight > 40 and currRight > 40 and prevRight - currRight > 40 :
            fineTurnCW() 
        elif currRight > 40 and currRight - prevRight > 40 and prevRight > 40: 
            fineTurnCCW()
        while not currDMSReading > 1900 : 
            fineDriveForward()
            currDMSReading = getAverageDMSSensorValue()
            rospy.loginfo(currDMSReading)

    elif currDMSReading > 2200 :  
        currLeft = getAverageLeftSensorValue()
        currRight = getAverageRightSensorValue()
        if currLeft > 40 and currLeft - prevLeft > 40 and prevLeft > 40:
            fineTurnCW()
        elif prevLeft > 40 and currLeft > 40 and prevLeft - currLeft > 40 : 
            fineTurnCCW()
        elif prevRight > 40 and currRight > 40 and prevRight - currRight > 40 :
            fineTurnCW() 
        elif currRight > 40 and currRight - prevRight > 40 and prevRight > 40: 
            fineTurnCCW()
        while not currDMSReading < 2200 : 
            fineDriveReverse()
            currDMSReading = getAverageDMSSensorValue()

#should move forward one tile

def tileStep() :
    #global prevLeft
    #global prevRight
    #currLeft = getAverageLeftSensorValue()
    #currRight = getAverageRightSensorValue()
    #if currLeft > 40 and currLeft - prevLeft > 40 and prevLeft > 40:
    #    fineTurnCW()
    #    veryFineDriveForward()
    #elif prevLeft > 40 and currLeft > 40 and prevLeft - currLeft > 40 : 
    #    fineTurnCCW()
    #    veryFineDriveForward()
    #elif prevRight > 40 and currRight > 40 and prevRight - currRight > 40 :
    #    fineTurnCW() 
    #    veryFineDriveForward()
    #elif currRight > 40 and currRight - prevRight > 40 and prevRight > 40: 
    #    fineTurnCCW()
    #    veryFineDriveForward()
    
    driveForward()
    #3
    time.sleep(3)
    haltDrive()
    #forwardErrorCheck()
    #prevLeft = currLeft
    #prevRight = currRight




explored = []
for i in range(0, 8) :
    nextRow = [0] * 8
    explored.append(nextRow)

#finds a path with the shortest distance between src and trg;
#returns the first direction to move along that path
def BFSDir(srci, srcj, trgi, trgj) :
    visited = {}
    visited[str(srci)+":"+str(srcj)] = 1
    search = []
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.North) == 0 :
        search.insert(0, [srci-1, srcj, DIRECTION.North])
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.West) == 0 :
        search.insert(0, [srci, srcj-1, DIRECTION.West])
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.South) == 0 :
        search.insert(0, [srci+1, srcj, DIRECTION.South])
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.East) == 0 :
        search.insert(0, [srci, srcj+1, DIRECTION.East])
    while len(search) > 0 :
        neighbor = search.pop()
        if str(neighbor[0])+":"+str(neighbor[1]) in visited :
            continue
        if neighbor[0] < 0 or neighbor[1] < 0 or neighbor[0] > 7 or neighbor[1] > 7 :
            continue
        visited[str(neighbor[0])+":"+str(neighbor[1])] = 1
        if neighbor[0] == trgi and neighbor[1] == trgj :
            return neighbor[2]
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.North) == 0 :
            search.insert(0, [neighbor[0]-1, neighbor[1], neighbor[2]])
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.West) == 0 :
            search.insert(0, [neighbor[0], neighbor[1]-1, neighbor[2]])
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.South) == 0 :
            search.insert(0, [neighbor[0]+1, neighbor[1], neighbor[2]])
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.East) == 0 :
            search.insert(0, [neighbor[0], neighbor[1]+1, neighbor[2]])
    return DIRECTION.West

#finds and returns the shortest distance between src and trg
def BFSDist(srci, srcj, trgi, trgj) :
    visited = {}
    visited[str(srci)+":"+str(srcj)] = 1
    search = []
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.North) == 0 :
        search.insert(0, [srci-1, srcj, 1])
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.West) == 0 :
        search.insert(0, [srci, srcj-1, 1])
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.South) == 0 :
        search.insert(0, [srci+1, srcj, 1])
    if mp.getNeighborObstacle(srci, srcj, DIRECTION.East) == 0 :
        search.insert(0, [srci, srcj+1, 1])
    while len(search) > 0 :
        neighbor = search.pop()
        if str(neighbor[0])+":"+str(neighbor[1]) in visited :
            continue
        if neighbor[0] < 0 or neighbor[1] < 0 or neighbor[0] > 7 or neighbor[1] > 7 :
            continue
        visited[str(neighbor[0])+":"+str(neighbor[1])] = 1
        if neighbor[0] == trgi and neighbor[1] == trgj :
            return neighbor[2]
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.North) == 0 :
            search.insert(0, [neighbor[0]-1, neighbor[1], neighbor[2]+1])
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.West) == 0 :
            search.insert(0, [neighbor[0], neighbor[1]-1, neighbor[2]+1])
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.South) == 0 :
            search.insert(0, [neighbor[0]+1, neighbor[1], neighbor[2]+1])
        if mp.getNeighborObstacle(neighbor[0], neighbor[1], DIRECTION.East) == 0 :
            search.insert(0, [neighbor[0], neighbor[1]+1, neighbor[2]+1])
    #shouldn't execute
    return -1

def getLeftDir() :
    if curDir == DIRECTION.West :
        return DIRECTION.South
    elif curDir == DIRECTION.North :
        return DIRECTION.West
    elif curDir == DIRECTION.East :
        return DIRECTION.North
    elif curDir == DIRECTION.South :
        return DIRECTION.East

def getRightDir() :
    if curDir == DIRECTION.West :
        return DIRECTION.North
    elif curDir == DIRECTION.North :
        return DIRECTION.East
    elif curDir == DIRECTION.East :
        return DIRECTION.South
    elif curDir == DIRECTION.South :
        return DIRECTION.West

 

def fineTurnCW() :
    slowDriveTurnCW()
    time.sleep(.08)
    #rospy.loginfo("fineturnCW!!")
    haltDrive()

def fineTurnCCW() :
    slowDriveTurnCCW()
    time.sleep(.2)
    #rospy.loginfo("fineturnCounterCW!!")
    haltDrive()

def fineDriveForward() :
    driveForward()
    time.sleep(0.05)
    haltDrive()

def veryFineDriveForward() :
    driveForward()
    time.sleep(0.02)
    haltDrive()

def fineDriveReverse() :
    driveReverse()
    time.sleep(0.05)
    haltDriveReverse()

def saveMap(fileName) :
    mapRepr = ""
    for i in range(0, 8) :
        for j in range(0, 8) :
            mapRepr += str(mp.getNeighborObstacle(i, j, DIRECTION.North)) + " "
            mapRepr += str(mp.getNeighborObstacle(i, j, DIRECTION.West)) + " "
            mapRepr += str(mp.getNeighborObstacle(i, j, DIRECTION.South)) + " "
            mapRepr += str(mp.getNeighborObstacle(i, j, DIRECTION.East)) + " "
    f = open(fileName, 'w')
    f.write(mapRepr.rstrip() + "\n")
    f.close()

def loadMap(fileName) :
    f = open(fileName, 'r')
    mapRepr = map(int, f.readline().rstrip().split(" "))
    f.close()
    idx = 0
    for i in range(0,8) :
        for j in range(0,8) :
            mp.setObstacle(i, j, mapRepr[idx], DIRECTION.North)
            idx += 1
            mp.setObstacle(i, j, mapRepr[idx], DIRECTION.West)
            idx += 1
            mp.setObstacle(i, j, mapRepr[idx], DIRECTION.South)
            idx += 1
            mp.setObstacle(i, j, mapRepr[idx], DIRECTION.East)
            idx += 1

def query(srci, srcj) :
    loadMap("./map.dat")
    mp.printObstacleMap()
    while True :
        print("Please indicate target destination:")
        print("i j")
        ti, tj = map(int, raw_input().split(" "))
        print("Please indicate target direction: North, West, East, or South (case-sensitive)")
        inDir = raw_input()
        while srci != ti or srcj != tj :
            nDir = BFSDir(srci, srcj, ti, tj)
            turnDir(nDir)
            tileStep()
            if nDir == DIRECTION.West :
                srcj -= 1
            elif nDir == DIRECTION.North :
                srci -= 1
            elif nDir == DIRECTION.East :
                srcj += 1
            elif nDir == DIRECTION.South :
                srci += 1
        if inDir == "North" :
            turnDir(DIRECTION.North)
        elif inDir == "South" :
            turnDir(DIRECTION.South)
        elif inDir == "West" :
            turnDir(DIRECTION.West)
        else :
            turnDir(DIRECTION.East)

def explore(srci, srcj) :
    global explored
    f = open('traverse_data.txt','a')
    mp.clearObstacleMap()
    for i in range(0, 8):
        for j in range(0,8):
            mp.setObstacle(i, j, 1, DIRECTION.North)
            mp.setObstacle(i, j, 1, DIRECTION.West)
            mp.setObstacle(i, j, 1, DIRECTION.South)
            mp.setObstacle(i, j, 1, DIRECTION.East)

    mp.printObstacleMap()
 
    explored[srci][srcj] = 1
    #check if there is a wall to the left or right; update map as necessary
    if not isLeftBlocked() :
        mp.setObstacle(srci, srcj, 0, getLeftDir())
    if not isRightBlocked() :
        mp.setObstacle(srci, srcj, 0, getRightDir())
    #turn 90 degrees and check the other two directions
    turnDir(getRightDir())
    if not isLeftBlocked() :
        mp.setObstacle(srci, srcj, 0, getLeftDir())
    if not isRightBlocked() :
        mp.setObstacle(srci, srcj, 0, getRightDir())

    toExplore = []
    if not mp.getNeighborObstacle(srci, srcj, DIRECTION.North) and srci > 0:
        toExplore.append([srci-1, srcj])
    if not mp.getNeighborObstacle(srci, srcj, DIRECTION.West) and srcj > 0:
        toExplore.append([srci, srcj-1])
    if not mp.getNeighborObstacle(srci, srcj, DIRECTION.East) and srcj < 7:
        toExplore.append([srci, srcj+1])
    if not mp.getNeighborObstacle(srci, srcj, DIRECTION.South) and srci < 7:
        toExplore.append([srci+1, srcj])

    while len(toExplore) > 0 :
        minDist = -1
        minCell = []
        minIndex = 0
        index = 0
        for cell in toExplore :
            cellDist = BFSDist(srci, srcj, cell[0], cell[1])
            print(str(srci)+"," + str(srcj) + ":" + str(cell)+ ":"+str(cellDist))
            if minDist == -1 or (cellDist < minDist and cellDist != -1)  :
                minDist = cellDist
                minCell = cell
                minIndex = index
            index += 1
        toExplore.pop(minIndex)    
        
        while srci != minCell[0] or srcj != minCell[1] :
            nDir = BFSDir(srci, srcj, minCell[0], minCell[1])
            turnDir(nDir)

            #format: prevL, prevR, prevDMS, currL, currR, currDMS, label
            f.write('['+str(getAverageLeftSensorValue())+',')
            f.write(str(getAverageRightSensorValue())+',')
            f.write(str(getAverageDMSSensorValue())+',')
            tileStep()
            x = raw_input('what action to take? ')
            f.write(str(getAverageLeftSensorValue())+',')
            f.write(str(getAverageRightSensorValue())+',')
            f.write(str(getAverageDMSSensorValue())+',')
            #fineTurnL, fineTurnR, fineDriveF, fineDriveR, doNothing
            f.write("'"+x+"'"+']'+'\n')
            print(x is "ftl" or x is "ftr")
            if x == 'fdf':
                fineDriveForward()
            elif x == 'fdr':
                fineDriveReverse()
            elif x == 'ftl':
                fineTurnCCW()
                veryFineDriveForward()
            elif x == 'ftr':
                fineTurnCW()
                veryFineDriveForward()

            if nDir == DIRECTION.West :
                srcj -= 1
            elif nDir == DIRECTION.North :
                srci -= 1
            elif nDir == DIRECTION.East :
                srcj += 1
            elif nDir == DIRECTION.South :
                srci += 1

        explored[srci][srcj] = 1
        logSensors()
        #check if there is a wall to the left or right; update -1map as necessary
        if not isLeftBlocked() :
            mp.setObstacle(srci, srcj, 0, getLeftDir())
        if not isRightBlocked() :
            mp.setObstacle(srci, srcj, 0, getRightDir())
        if not isFrontBlocked() :
            mp.setObstacle(srci, srcj, 0, curDir)

        saveMap("./map.dat")

        if (srci >= 1) and (mp.getNeighborObstacle(srci, srcj, DIRECTION.North) == 0) and not explored[srci-1][srcj] and [srci-1, srcj] not in toExplore :
            toExplore.append([srci-1, srcj])
        if (srcj >= 1) and (mp.getNeighborObstacle(srci, srcj, DIRECTION.West) == 0) and not explored[srci][srcj-1] and [srci, srcj-1] not in toExplore :
            toExplore.append([srci, srcj-1])
        if (srcj <= 6) and (mp.getNeighborObstacle(srci, srcj, DIRECTION.East) == 0) and not explored[srci][srcj+1] and [srci, srcj+1] not in toExplore :
            toExplore.append([srci, srcj+1])
        if (srci <= 6) and (mp.getNeighborObstacle(srci, srcj, DIRECTION.South) == 0) and not explored[srci+1][srcj] and [srci+1, srcj] not in toExplore:
            toExplore.append([srci+1, srcj])
        
        mp.printObstacleMap()
        if isDebug() :
            rospy.loginfo("Estimated position:"+str(srci) + ", " + str(srcj))
            logSensors()
    
    query(srci, srcj)

def printPath(srci, srcj, trgi, trgj) :
    while srci != trgi or srcj != trgj :
        nDir = BFSDir(srci, srcj, trgi, trgj)
        if nDir == DIRECTION.West :
            srcj -= 1
        elif nDir == DIRECTION.North :
            srci -= 1
        elif nDir == DIRECTION.East :
            srcj += 1
        elif nDir == DIRECTION.South :
            srci += 1

        rospy.loginfo(str(srci) + ", " + str(srcj))

def driveTurnCW90() :
    driveTurnCW()
    time.sleep(1.95)
    haltDrive()
def driveTurnCCW90() :
    driveTurnCCW()
    time.sleep(1.81)
    haltDrive()



#Legs:
#1, 5 front left
#3, 7 back left
#2, 6 front right
#4, 8 back right

#perfect position sensor readings:
#DMS: 1971 +-
#Left: 169 +-

# Main function
if __name__ == "__main__":
    global curDir 

    print("Which direction is the robot facing: North, West, East, or South (case-sensitive)?")
    inDir = raw_input()
    if inDir == "North" :
        curDir = DIRECTION.North
    elif inDir == "South" :
        curDir = DIRECTION.South
    elif inDir == "West" :
        curDir = DIRECTION.West
    else :
        curDir = DIRECTION.East
    print("What are the starting coordinates?\ni j")
    si, sj = map(int, raw_input().split(" "))

    print("Load map from map.dat, or explore ? [L/E]")
    load = raw_input()

    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group l Control Node...")
    startTime = time.time();

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz

    setMotorMode(9, 1)
    setMotorMode(10,1)
    
    #while True :
    #    logSensors()
    #    time.sleep(1)

    if load == "E" :
        explore(si, sj)
    else :
        query(si, sj)

    #tileStep()
    #rospy.loginfo(getAverageLeftSensorValue())
    #rospy.loginfo(getAverageRightSensorValue())
    #rospy.loginfo(getAverageDMSSensorValue())
    
    #rospy.loginfo(time.time() - startTime)
