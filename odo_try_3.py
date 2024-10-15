# VEX V5 Python Project with Competition Template
import math as m
import sys
import vex
from vex import *
import motor_group
import drivetrain
import smartdrive

#region config
brain          = vex.Brain()
inertial_21    = vex.Inertial(vex.Ports.PORT21)
encoder_left   = vex.Encoder(brain.three_wire_port.a)
encoder_right  = vex.Encoder(brain.three_wire_port.c)
encoder_middle = vex.Encoder(brain.three_wire_port.e)
#endregion config


# Creates a competition object that allows access to Competition methods.
competition = vex.Competition()

def pre_auton():
    global Sl, Sr, Ss, wheelDiameter, trackingDiameter
    global x, y, lastLeftPos, lastRightPos, lastSidePos, deltaTheta, thetaNew, thetaM, curLeft
    global curRight, curSide, leftAtReset, rightAtReset, thetaReset
    global deltaLeft, deltaRight, deltaSide, deltaLr, deltaRr, deltaX, deltaY
    global theta, radius
    # All activities that occur before competition start
    # Example: setting initial positions
    
        


def autonomous():
    global Sl, Sr, Ss, wheelDiameter, trackingDiameter
    global x, y, lastLeftPos, lastRightPos, lastSidePos, deltaTheta, thetaNew, thetaM, curLeft
    global curRight, curSide, leftAtReset, rightAtReset, thetaReset
    global deltaLeft, deltaRight, deltaSide, deltaLr, deltaRr, deltaX, deltaY
    global theta, radius
    # Place autonomous code here
    pass

def drivercontrol():
    global Sl, Sr, Ss, wheelDiameter, trackingDiameter
    global lcal, x, y, lastLeftPos, lastRightPos, lastSidePos, deltaTheta, thetaNew, thetaM, curLeft
    global curRight, curSide, leftAtReset, rightAtReset, thetaReset
    global deltaLeft, deltaRight, deltaSide, deltaLr, deltaRr, deltaX, deltaY
    global theta, radius, angle
    
    Sl = 1.125 #remember to change this  #distance from tracking center to middle of left wheel
    Sr = 1.125 #remember to change this  #distance from tracking center to middle of right wheel
    Ss = 0 #remember to change this  #distance from tracking center to middle of the tracking wheel
    wheelDiameter = 4.125 #diameter of the side wheels being used for tracking
    trackingDiameter = 4.125 #diameter of the sideways tracking wheel
    
    
    lcal = (m.pi/180)*(wheelDiameter/2)
    
    x = 0
    y = 0
    
    lastLeftPos = 0
    lastRightPos = 0
    lastSidePos = 0
    
    
    deltaTheta = 0
    thetaNew = 0
    thetaM = 0
    
    
    
    curLeft = 0
    curRight = 0
    curSide = 0
    
    leftAtReset = 0
    rightAtReset = 0
    thetaReset = 0
    
    deltaLeft = 0
    deltaRight = 0
    deltaSide = 0
    
    deltaLr = 0
    deltaRr = 0
    
    deltaX = 0
    deltaY = 0
    
    
    theta = 0
    radius = 0
    

    inertial_21.calibrate()
   
    
    
    # Place drive control code here, inside the loop
    while True:
        # This is the main loop for the driver control.
        # Each time through the loop you should update motor
        # movements based on input from the controller.
        updatePosition()
        

def updatePosition() :
    global Sl, Sr, Ss, wheelDiameter, trackingDiameter
    global lcal, x, y, lastLeftPos, lastRightPos, lastSidePos, deltaTheta, thetaNew, thetaM, curLeft
    global curRight, curSide, leftAtReset, rightAtReset, thetaReset, tracking
    global deltaLeft, deltaRight, deltaSide, deltaLr, deltaRr, deltaX, deltaY
    global theta, radius, angle
    
      # Store encoder values in local variables
    curLeft = encoder_left.rotation(vex.DEGREES)
    curRight = encoder_right.rotation(vex.DEGREES) #step 1
    curSide = encoder_middle.rotation(vex.DEGREES)
    
    # Calculate the change in each encoder since the last cycle
    # Take in wheel diameter *  the last position * radian -> calculating arclength since last cycle of encoding
    deltaLeft = (curLeft - lastLeftPos)*lcal
    deltaRight = (curRight - lastRightPos)*lcal #step 2
    deltaSide = (curSide - lastSidePos)*(m.pi/180)*(trackingDiameter/2)
    
    # Updating values to the previous values of the encoders
    lastLeftPos = curLeft
    lastRightPos = curRight #step 3
    lastSidePos = curSide
    
    # Calculating the total change in left and right encoders
    # Convert to the distance of wheel travel
    deltaLr = (curLeft - leftAtReset)*lcal #step 4
    deltaRr = (curRight - rightAtReset)*lcal
    
      #Calculate the new absolute orientation, theta(1) = theta(r) + (deltaLr - deltaRr)/(Sl + Sr)
    thetaNew = (thetaReset + (deltaLr - deltaRr)/(Sl + Sr)) #step 5
    
      # Calculate the change in angle delataTheta
    deltaTheta = thetaNew - angle #step 6
    deltaSide = deltaSide - Ss*deltaTheta
    
      # deltaTheta to calculate the offset
      # Check if deltaTheta 0 is equal to local offset
    if (deltaTheta == 0) :
        deltaX = deltaSide; #step 7
        deltaY = deltaRight
      
      # Calculate the local offset with 2sin(theta/2) 
      # Referencing equation 6
    else :
        deltaX = (2*m.sin(deltaTheta/2))*(deltaSide/deltaTheta + Ss); #step 8
        deltaY = (2*m.sin(deltaTheta/2))*(deltaRight/deltaTheta +Sr)
      
      # Calculate the average orientation for thetaM = theta(0) (angle) + change in theta/2
    thetaM = angle + deltaTheta/2; #step 9
    
      # Calculate the global offset
    theta = m.atan2(deltaY, deltaX)
    radius = m.sqrt(deltaX*deltaX + deltaY*deltaY)
    theta = theta-thetaM;                          #step 10
    deltaX = radius*m.cos(theta)
    deltaY = radius*m.sin(theta)
    
     
    thetaNew+=m.pi
    while (thetaNew <= 0) :
        thetaNew+=2*m.pi
      
        thetaNew = thetaNew % (2*m.pi)
        thetaNew -= m.pi
    
     # Calculate the new absolute position 
    angle = thetaNew
    x = x - deltaX #step 11
    y = y + deltaY

    
    brain.screen.print_line(1, x)
    brain.screen.print_line(2, y)
    

    
# Do not adjust the lines below

# Set up (but don't start) callbacks for autonomous and driver control periods.
competition.autonomous(autonomous)
competition.drivercontrol(drivercontrol)

# Run the pre-autonomous function.
pre_auton()

# Robot Mesh Studio runtime continues to run until all threads and
# competition callbacks are finished.
