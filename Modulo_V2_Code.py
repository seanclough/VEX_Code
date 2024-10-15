# VEX V5 Python Project with Competition Template
import sys
import vex
from vex import *
import motor_group
import drivetrain
import smartdrive
import math
import timer

#region config
brain         = vex.Brain()
arms2         = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO18_1, True)
left2         = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO18_1, False)
right1        = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO18_1, True)
conveyor      = vex.Motor(vex.Ports.PORT5, vex.GearSetting.RATIO18_1, False)
arms          = vex.Motor(vex.Ports.PORT7, vex.GearSetting.RATIO18_1, False)
back          = vex.Motor(vex.Ports.PORT8, vex.GearSetting.RATIO18_1, False)
topLeft       = vex.Motor(vex.Ports.PORT9, vex.GearSetting.RATIO18_1, False)
triport       = vex.Triport(vex.Ports.PORT10)
backPot       = vex.Pot(triport.a)
bottomRight   = vex.Motor(vex.Ports.PORT11, vex.GearSetting.RATIO18_1, True)
rightBottom   = vex.Motor(vex.Ports.PORT15, vex.GearSetting.RATIO18_1, True)
inertial_21   = vex.Inertial(vex.Ports.PORT21)
armReset      = vex.Limit(brain.three_wire_port.a)
otherArmReset = vex.Bumper(brain.three_wire_port.b)
needle        = vex.Pneumatics(brain.three_wire_port.d)
hood1         = vex.Pneumatics(brain.three_wire_port.e)
hood2         = vex.Pneumatics(brain.three_wire_port.f)
claw          = vex.Pneumatics(brain.three_wire_port.g)
pot           = vex.Pot(brain.three_wire_port.h)
con           = vex.Controller(vex.ControllerType.PRIMARY)
#endregion config

# Creates a competition object that allows access to Competition methods.
competition = vex.Competition()

def findPower(current, wanted, intensity, acceleration, Kp, Ki, variance):
    # global means these variables can be used in other functions
    global done, totalError, power, lastPower
    error =  wanted - current
    if Ki*totalError > 10:
       totalError = 10./Ki
    if Ki*totalError < -10:
       totalError = -10./Ki
    power = Kp*error + Ki*totalError
    power = (1-acceleration/100.)*lastPower + acceleration/100.*power
    if power > intensity:
       power = intensity
    if power*-1 > intensity:
       power = -1*intensity
    if abs(error) <= variance:
       done = True
    totalError += error
    lastPower = power
    
# MoveTo Functions:
def armsMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        findPower(arms.rotation(vex.DEGREES), position, speed, acceleration, .5, .001, 1)
        arms.spin(vex.FORWARD, power, vex.PERCENT)
        arms2.spin(vex.FORWARD, power, vex.PERCENT)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
def forwardMoveTo(position, speed, acceleration, Kp, Ki, variance):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        findPower(topLeft.rotation(vex.DEGREES), position, speed, acceleration, Kp, Ki, variance)
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
def spinMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        inertialAngle = inertial_21.heading(vex.DEGREES)
        if inertial_21.heading(vex.DEGREES) > 180:
            inertialAngle -= 360
        findPower(inertialAngle, position, speed, acceleration, .5, .001, 1)
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.REVERSE, power, vex.PERCENT)
        rightBottom.spin(vex.REVERSE, power, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
def backMogoMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        findPower(backPot.value(vex.DEGREES), position, speed, acceleration, 7, .001, 1)
        back.spin(vex.FORWARD, power, vex.PERCENT)
    back.stop(vex.BrakeType.HOLD)
def clawGrab():
    claw.open()
def clawLetGo():
    claw.close()
    
def newForwardMoveTo(position, speed, accel, Kp, Ki, Kh, accelBreakPoint):
    # position: where you want it to go
    # speed: max speed that it will spin at
    # accel: limits how fast the robot accelerates (max value is 100)
    # Kp: proportional constant; increasing will increase speed and overall arrival time to position but higher values might also cause oscillation
    # Ki: integral constant; increasing will cause oscillation, small values make the robot get to position value by keeping the power as the proportional value slows down
    # Kh: heading constant; (experimental) too high should = oscillations and too low should = slow reaction time
    # accelBreakPoint: accel is shut off after this point, HAS TO BE BETWEEN THE STARTING POSITION AND THE WANTED POSITION
    global done, totalError, power, lastPower
    done = False
    totalError = 0 
    power = 0
    lastPower = 0
    updatedAccel = accel
    # saves starting heading and then tries to continue on that heading
    heading = inertial_20.heading(vex.DEGREES)
    totalHeadingError = 0
    while not done:
        # assumes accelBreakPoint is between starting position and ending position
        if updatedAccel != accel:
            # this first if just removes redundancy 
            if accelBreakPoint < position:
                # second if checks if we are going forward or backward
                if left.rotation(vex.DEGREES) > accelBreakPoint:
                    # third if turns off the accel
                    updatedAccel = 100
            else:
                if left.rotation(vex.DEGREES) < accelBreakPoint:
                    updatedAccel = 100
        
        # findPower function:
        error =  position - left.rotation(vex.DEGREES)
        
        # find headingError
        rawHeadingError = inertial_20.heading(vex.DEGREES) - heading
        # this is jank because error is actually wanted - actual but this is actual - wanted
        # don't have enough time to make it right, it should work the way it is though
        if rawHeadingError <= -180:
            headingError = rawHeadingError + 360
        elif rawHeadingError > 180:
            headingError = rawHeadingError - 360
        else:
            headingError = rawHeadingError
        
        # limits totalError
        if Ki*totalError > 10:
            totalError = 10./Ki
        if Ki*totalError < -10:
            totalError = -10./Ki
        
        # the actual PI part
        # error (input) -> function -> power (output)
        power = Kp*error + Ki*totalError 
        power = (1-accel/100.)*lastPower + accel/100.*power
        
        # limits speed
        if power > speed:
            power = speed
        if power*-1 > speed:
            power = -1*speed
        
        # this is how we get out of the loop; when we get to the position, we stop moving
        if abs(error) <= 5:
            done = True
        
        totalError += error
        lastPower = power
        totalHeadingError += headingError
        
        # this makes the motors moves
        # this is also where we adjust the robot's heading
        #left.spin(vex.FORWARD, power - Kh * totalHeadingError, vex.PERCENT)
        #left2.spin(vex.FORWARD, power - Kh * totalHeadingError, vex.PERCENT)
        #right.spin(vex.FORWARD, power + Kh * totalHeadingError, vex.PERCENT)
        #right2.spin(vex.FORWARD, power + Kh * totalHeadingError, vex.PERCENT)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
    
    # after we get to where we want to go, we have to stop the motors
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)

# MoveTowards Functions:
def moveBackTowards(position, intensity, acceleration):
    global backError, backTotalError, backPower, lastBackPower
    backError =  position - backPot.value(vex.DEGREES)
    if .001*backTotalError > 30:
       backTotalError = 10000
    if .001*backTotalError < -30:
       backTotalError = -10000
    backPower = 7*backError + .001*backTotalError
    backPower = (1-acceleration/100.)*lastBackPower + acceleration/100.*backPower
    if backPower > intensity:
       backPower = intensity
    if backPower*-1 > intensity:
       backPower = -1*intensity
    backTotalError += backError
    lastBackPower = backPower
def moveArmTowards(position, intensity, acceleration):
    global armError, armTotalError, armPower, lastArmPower
    armError =  position - arms.rotation(vex.DEGREES)
    if .001*armTotalError > 10:
       armTotalError = 10000
    if .001*armTotalError < -10:
       armTotalError = -10000
    armPower = .5*armError + .001*armTotalError
    armPower = (1-acceleration/100.)*lastArmPower + acceleration/100.*armPower
    if armPower > intensity:
       armPower = intensity
    if armPower*-1 > intensity:
       armPower = -1*intensity
    armTotalError += armError
    lastArmPower = armPower

# Auton Function:
def pickAuton():
    global value_choice
    value_choice = pot.value(vex.DEGREES)
    brain.screen.print_line(4, value_choice)
    while value_choice == 0.0:
        value_choice = pot.value(vex.DEGREES)
    brain.screen.print_line(3, pot.value(vex.DEGREES)) 
    brain.screen.print_line(1, "Selected Auton: ")
    if(value_choice >= 0 and value_choice < 25):
        doubleYellowShort()
        brain.screen.print_line(2, "double yellow short first")
    elif(value_choice >= 25 and value_choice < 80):
        
        brain.screen.print_line(2, "AWP Line, scores rings and moves mogo")
    elif(value_choice >= 80 and value_choice < 130):
        rampRings()
        brain.screen.print_line(2, "Ramp Rings (for AWP)")
    elif(value_choice >= 130 and value_choice < 200):
        leftYellow()
        brain.screen.print_line(2, "Left side single yellow mogo")
    elif(value_choice >= 200 and value_choice < 300):
        brain.screen.print_line(2, "Empty auton slot")
    else:
        doubleYellowShort()
        brain.screen.print_line(2, "No Auton Here! Did default code")
        
def doubleYellowShort():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08

    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    hood1.open()
    hood2.open()
    claw.open()
   
    updatedAccel = forwardAccel
    updatedAccel = 100
    accelBreakPoint = 1300
    intensity = 100
    while not done: 
        if accelBreakPoint < topLeft.rotation(vex.DEGREES):
            # second if checks if we are going forward or backward
            if topLeft.rotation(vex.DEGREES) > accelBreakPoint:
                # third if turns off the accel
                updatedAccel = 100
                #intensity = 50
                intensity = 100
        con.screen.print_(intensity)
        #was 1590 distance
        findPower(topLeft.rotation(vex.DEGREES), 1650, intensity, updatedAccel, .3, .001, 5)
        
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(90, 100, 100)
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
        if topLeft.rotation(vex.DEGREES) > 1590:
            claw.close()
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    
    #claw.close()
    #vex.wait(.3, vex.SECONDS)
    #armsMoveTo(250, 100, 100)
    
    #was 1000
    forwardMoveTo(600, 100, forwardAccel, .3, .001, 5)
    #change this number^ (1000 -> lower number)
    
    hood1.close()
    hood2.close()
    armsMoveTo(1200,100,100)
    
    #was 122.5
    spinMoveTo(131.5, 100, forwardAccel)
    #change this number (122.5 -> higher number)
    
    #was 200
    constant = -600
    #change this number if needed (200 -> lower number)
    
    #was 114.2
    backMogoMoveTo(115.5, 100, 100)
    forwardMoveTo(constant, 100, forwardAccel, .3, .001, 5)
    back.spin(vex.FORWARD, -10,vex.PERCENT)
    forwardMoveTo(constant - 400, 10, 100, .3, .001, 5)
    back.stop(vex.BrakeType.HOLD)
    forwardMoveTo(constant + 2000, 100, forwardAccel, .3, .001, 5)
def doubleYellowTall():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08
    
    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    hood1.open()
    hood2.open()
    claw.open()
   
    while not done: 
        findPower(topLeft.rotation(vex.DEGREES), xxx, 100, forwardAccel, .3, .001, 5)
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(100, 100, 100)
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    
    claw.close()
    vex.wait(.3, vex.SECONDS)
    #armsMoveTo(250, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    hood1.close()
    hood2.close()
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    back.spin(vex.FORWARD, -10,vex.PERCENT)
    forwardMoveTo(xxx, 10, 100, .3, .001, 5)
    back.stop(vex.BrakeType.HOLD)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
def rightYellow():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08

    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    hood1.open()
    hood2.open()
    claw.open()
    
    while not done: 
        findPower(topLeft.rotation(vex.DEGREES), 1690, 100, forwardAccel, .3, .001, 5)
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(100, 100, 100)
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    
    claw.close()
    vex.wait(.3, vex.SECONDS)
    #armsMoveTo(250, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    hood1.close()
    hood2.close()
    armsMoveTo(xxx, 100, 100)
    spinMoveTo(xxx, 100, forwardAccel)
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    back.spin(vex.FORWARD, -10,vex.PERCENT)
    forwardMoveTo(xxx, 10, 100, .3, .001, 5)
    back.stop(vex.BrakeType.HOLD)
    
    backMogoMoveTo(xxx, 100, 100) 
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    conveyor.spin(FORWARD, 100, vex.PERCENT)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    conveyor.stop(vex.BrakeType.HOLD)
def middleYellow():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08
    
    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    hood1.open()
    hood2.open()
    claw.open()
   
    while not done: 
        findPower(topLeft.rotation(vex.DEGREES), xxx, 100, forwardAccel, .3, .001, 5)
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(100, 100, 100)
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    
    claw.close()
    vex.wait(.3, vex.SECONDS)
    #armsMoveTo(250, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    hood1.close()
    hood2.close()
    armsMoveTo(xxx, 100, 100)
    spinMoveTo(xxx, 100, forwardAccel)
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(xxx, 100, 100)
    conveyor.spin(FORWARD, 100, vex.DEGREES)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    conveyor.stop(vex.BrakeType.HOLD)
def leftYellow():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08
    
    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    hood1.open()
    hood2.open()
    claw.open()
    
    """
    updatedAccel = forwardAccel
    accelBreakPoint = 1200
    intensity = 100
    while not done: 
        if accelBreakPoint < topLeft.rotation(vex.DEGREES):
            # second if checks if we are going forward or backward
            if topLeft.rotation(vex.DEGREES) > accelBreakPoint:
                # third if turns off the accel
                updatedAccel = 100
                intensity = 50
        con.screen.print_(intensity)
        findPower(topLeft.rotation(vex.DEGREES), 1520, intensity, updatedAccel, .3, .001, 5)
        
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(100, 100, 100)
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    """
    
    updatedAccel = forwardAccel
    updatedAccel = 100
    accelBreakPoint = 1300
    intensity = 100
    armResetPushed = True
    otherArmResetPushed = True
    while not done: 
        if accelBreakPoint < topLeft.rotation(vex.DEGREES):
            # second if checks if we are going forward or backward
            if topLeft.rotation(vex.DEGREES) > accelBreakPoint:
                # third if turns off the accel
                updatedAccel = 100
                #intensity = 50
                intensity = 100
        con.screen.print_(intensity)
        #was 1590 distance
        findPower(topLeft.rotation(vex.DEGREES), 1750, intensity, updatedAccel, .3, .001, 5)
        
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        
        """
        if not armReset.pressing() and armResetPushed:
            arms.set_rotation(0, vex.DEGREES)
            armResetPushed = False
        """
        if not otherArmReset.pressing() and otherArmResetPushed:
            arms.set_rotation(0, vex.DEGREES)
            otherArmResetPushed = False
        if not otherArmReset:
            moveArmTowards(1000, 100, 100)
        else: 
            moveArmTowards(140, 100, 100)
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
        if topLeft.rotation(vex.DEGREES) > 1570:
            claw.close()
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    
    """
    claw.close()
    vex.wait(.3, vex.SECONDS)
    #armsMoveTo(250, 100, 100)
    """
    
    done = False
    totalError = 0
    lastPower = 0
    updatedAccel = forwardAccel
    accelBreakPoint = 100
    intensity = 100
    while not done: 
        if accelBreakPoint > topLeft.rotation(vex.DEGREES):
            # second if checks if we are going forward or backward
            if topLeft.rotation(vex.DEGREES) > accelBreakPoint:
                # third if turns off the accel
                updatedAccel = 100
                intensity = 50
        con.screen.print_(intensity)
        findPower(topLeft.rotation(vex.DEGREES), 0, intensity, updatedAccel, .3, .001, 5)
        
        topLeft.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(1000, 100, 100)
        hood1.close()
        hood2.close()
        arms.spin(vex.FORWARD, armPower, vex.PERCENT)
        arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
    topLeft.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    
    spinMoveTo(-92, 100, forwardAccel)
    
    forwardMoveTo(500, 100, 100, .3, .001, 5)
    backMogoMoveTo(115, 100, 100)
    
    constant = 0
    """
    #at this point back mogo lift is down and is facing alliance mogo, need to move back, then move back 
    forwardMoveTo(constant, 100, 100)
    backMogoMoveTo(75.5, 100, 100)
    
    #at this point, robot should have back mogo in right position to score rings
    conveyor.spin(FORWARD, 100, vex.PERCENT)
    vex.wait(1, vex.SECONDS)
    # might have to change wait time
    
    forwardMoveTo(constant + 200, 20, 100)
    vex.wait(.5,vex.SECONDS)
    # might have to change 200
    forwardMoveTo(constant, 70, 100)
    vex.wait(1,vex.SECONDS)
    
    forwardMoveTo(constant + 200, 20, 100)
    vex.wait(.5,vex.SECONDS)
    forwardMoveTo(constant, 70, 100)
    vex.wait(1,vex.SECONDS)
    
    backMogoMoveTo(114.2, 100, 100)
    conveyor.stop(vex.BrakeType.HOLD)
    
    
    
    
    
    
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(xxx, 100, 100)
    conveyor.spin(FORWARD, 100, vex.DEGREES)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    conveyor.stop(vex.BrakeType.HOLD)
    """
def AWPplusShort():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08
    
    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    claw.open()
    
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(xxx, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    armsMoveTo(xxx, 100, 100)
    conveyor.spin(FORWARD, 100, vex.DEGREES)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    conveyor.stop(vex.BrakeType.HOLD)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(xxx, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    conveyor.spin(FORWARD, 100, vex.DEGREES)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    conveyor.stop(vex.BrakeType.HOLD)
    armsMoveTo(100, 100, 100)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    claw.close()
    vex.wait(.3, vex.SECONDS)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
def AWPplusTall():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    forwardAccel = .08
    
    done = False
    totalError = 0
    lastPower = 0
   
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    claw.open()
    
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(xxx, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    armsMoveTo(xxx, 100, 100)
    conveyor.spin(FORWARD, 100, vex.DEGREES)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    conveyor.stop(vex.BrakeType.HOLD)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(114.2, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    backMogoMoveTo(xxx, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    conveyor.spin(FORWARD, 100, vex.DEGREES)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    conveyor.stop(vex.BrakeType.HOLD)
    armsMoveTo(100, 100, 100)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    claw.close()
    vex.wait(.3, vex.SECONDS)
    spinMoveTo(xxx, 100, forwardAccel)
    forwardMoveTo(xxx, 100, forwardAccel, .3, .001, 5)
    
def AWP_line():
    forwardAccel = .08
    forwardMoveTo(-200, 30, forwardAccel, .3, .001)
    backMogoMoveTo(400, 60, 100)
    forwardMoveTo(100, 50, forwardAccel, .3, .001)
    backMogoMoveTo(1240, 100, 100)
    spinMoveTo(12, 100, 10)
    # forward move to was set to -350, changed it
    forwardMoveTo(-390, 30, forwardAccel, .3, .001)
    #backMogoMoveTo(400, 100, 100)
    forwardMoveTo(100, 10, forwardAccel, .3, .001)
def rampRings():
    armsMoveTo(200, 100, 100)
    conveyor.spin(FORWARD, 100, vex.PERCENT)
def oldleftYellow():
    forwardAccel = .08
    clawLetGo()
    armsMoveTo(70, 100, 100)
    forwardMoveTo(1670, 100, forwardAccel, .3, .001)
    clawGrab()
    armsMoveTo(120, 100, 100)
    forwardMoveTo(100, 100, forwardAccel, .3, .001)
    
def pre_auton():
    inertial_21.calibrate()

def autonomous():
    pickAuton()
    

def drivercontrol():
    global done, totalError, power, lastPower
    global backError, backTotalError, backPower, lastBackPower
    global armError, armTotalError, armPower, lastArmPower
    global value_choice
    
    topLeft.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    conveyor.stop(vex.BrakeType.HOLD)
    rightBottom.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    back.stop(vex.BrakeType.HOLD)
    
    # Slowmode variables
    """
    ready = True
    slowmode = False
    timer_2 = timer.Timer()
    timer_2.start()
    """
    
    ready2 = True
    slowmode2 = False
    timer_3 = timer.Timer()
    timer_3.start()
    
    # starting positions
    backPosition0 = False
    backPosition1 = False
    backPosition2 = False
    backPosition3 = False
    ringForward = False
    ringReverse = False
    ringStop = True
    ringReady = True
    armOptimal = False
    armOptimalPositon = 110
    
    
    backError = 0
    backTotalError = 0
    backPower = 0
    lastBackPower = 0
    
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    XpressTimer = timer.Timer()
    Xpressed = False
    
    hoodWanted = False
    
    AReady = True
    Apressed = False
    ApressTimer = timer.Timer()
    slowmode = False
    
    #doubleYellowShort()
    #leftYellow()
    #claw.open()
    
    while True:
        
        brain.screen.print_line(5, otherArmReset.pressing())
        """
        # slowmode code:
        if con.buttonA.pressing() and ready:
            slowmode = not slowmode
            ready = False
            timer_2.reset()
            timer_2.start()
        if timer_2.elapsed_time() > .6:
            ready = True
        """
        
        if con.buttonA.pressing() and AReady:
            Apressed = True
            ApressTimer.reset()
            ApressTimer.start()
            AReady = False
        if Apressed and con.buttonA.pressing():
            if ApressTimer.elapsed_time() > 1.5:
                needle.open()
        if not con.buttonA.pressing():
            if Apressed and ApressTimer.elapsed_time() < 1.5:
                slowmode = not slowmode
            ApressTimer.stop()
            ApressTimer.reset()
            AReady = True
            Apressed = False
            needle.close()
        
        # Drivetrain code (to make it move)
        forwardPower = con.axis2.position(vex.PERCENT) #multipliers can be added
        spinPower = con.axis4.position(vex.PERCENT) 
        rightPower = forwardPower-spinPower
        leftPower = forwardPower+spinPower
        if slowmode:
            rightPower *= .2
            leftPower *= .2
        right1.spin(vex.FORWARD, rightPower, vex.PERCENT)
        rightBottom.spin(vex.FORWARD, rightPower, vex.PERCENT)
        topLeft.spin(vex.FORWARD, leftPower, vex.PERCENT)
        left2.spin(vex.FORWARD, leftPower, vex.PERCENT)
        
        if con.buttonL1.pressing():
            armOptimal = False
            hoodWanted = False
            armPower = 100
            if backPosition3:
                backPosition3 = False
                backPosition2 = True
        
        elif con.buttonL2.pressing() and arms.rotation(vex.DEGREES) >= armOptimalPositon:
            armOptimal = False
            armPower = -100
        
        elif con.buttonL2.pressing():
            armOptimal = False
            armPower = -100
        elif con.buttonY.pressing():
            armOptimal = True
            hoodWanted = True
            backPosition3 = True
            backPosition0 = False
            backPostition1 = False
            backPosition2 = False
            ringStop = True
            ringForward = False
            ringReverse = False
        elif con.buttonB.pressing():
            armOptimal = True
            hoodWanted = False
        """
        elif arms.rotation(vex.DEGREES) < armOptimalPositon:
            armOptimal = True
        """
        if ((not con.buttonL2.pressing()) and (not con.buttonL1.pressing())) and (not armOptimal):
            armPower = 0
        
        if con.buttonLeft.pressing() and not backPosition1  and not backPosition2:
            backPosition1 = True
            backPosition0 = False
            backPosition3 = False
            done = False
            ready2 = False
            timer_3.reset()
            timer_3.start()
        if timer_3.elapsed_time() > .25:
            ready2 = True
        if con.buttonLeft.pressing() and ready2 and backPosition1:
            backPosition2 = True
            backPosition1 = False
            backPosition3 = False
            done = False
        if con.buttonRight.pressing():
            backPosition0 = True
            done = False
        if backPosition0:
            backPosition1 = False
            backPosition2 = False
            backPosition3 = False
        
        if con.buttonUp.pressing(): 
            backPosition0 = False
            backPosition1 = False
            backPosition2 = False
            backPosition3 = False
            backPower = -100
        if con.buttonDown.pressing(): 
            backPosition0 = False
            backPosition1 = False
            backPosition2 = False
            backPosition3 = False
            backPower = 100
        if not con.buttonUp.pressing() and not con.buttonDown.pressing() and not backPosition0 and not backPosition1 and not backPosition2 and not backPosition3:
            backPower = 0
        
        if con.buttonR1.pressing(): 
            clawGrab()
        if con.buttonR2.pressing():
            clawLetGo()
        
        """
        #cycles through ringForward, ringReverse, and ringStop
        if con.buttonX.pressing() and ringReady:
            if ringForward and not(ringReverse or ringStop):
                ringForward = False
                ringReverse = True
            elif ringStop and not(ringForward or ringReverse):
                ringStop = False
                ringForward = True
            else:
                #ringReverse if statement redundant
                #this captures funky if funky presents itself
                ringForward = False
                ringReverse = False
                ringStop = True
            ringReady = False
        if not con.buttonX.pressing():
            ringReady = True
        if ringForward and not (ringReverse or ringStop):
            conveyor.spin(vex.FORWARD, 100, vex.PERCENT)
        elif ringReverse and not (ringForward or ringStop):
            conveyor.spin(vex.REVERSE, 100, vex.PERCENT)
        else:
            conveyor.stop(vex.BrakeType.HOLD)
        """
        # short press & off = forward, long press & off = reverse, press & on = off
        if con.buttonX.pressing() and ringStop and ringReady:
            Xpressed = True
            XpressTimer.reset()
            XpressTimer.start()
            ringReady = False
        if con.buttonX.pressing() and not ringStop and ringReady:
            ringStop = True
            ringForward = False
            ringReverse = False
            ringReady = False
        if Xpressed and not con.buttonX.pressing():
            XpressTimer.stop()
            if XpressTimer.elapsed_time() > .7:
                ringReverse = True
            else: 
                ringForward = True
            ringStop = False
            Xpressed = False
        if not con.buttonX.pressing():
            ringReady = True
            
        if backPosition3 and arms.rotation(vex.DEGREES) < armOptimalPositon + 10 and hoodWanted:
            hood1.open()
            hood2.open()
        else:
            hood1.close()
            hood2.close()
        
        if backPosition0:
            moveBackTowards(117, 100, 100)
        if backPosition1:
            moveBackTowards(95.5, 100, 100)
        if backPosition2:
            moveBackTowards(77.1, 100, 100)
        if backPosition3:
            moveBackTowards(66.5, 100, 100)
        brain.screen.print_line(1, ringStop)
        brain.screen.print_line(2, ringForward)
        brain.screen.print_line(3, ringReverse)
        
        if ringReverse:
            conveyor.spin(FORWARD, -100, vex.DEGREES)
        if ringForward:
            conveyor.spin(FORWARD, 100, vex.DEGREES)
        if ringStop:
            conveyor.stop(vex.BrakeType.HOLD)
        
        if armOptimal:
            moveArmTowards(armOptimalPositon, 100, 100)
        
        if abs(backPower) < 5:
            back.stop(vex.BrakeType.HOLD)
            done = True
        else:
            back.spin(vex.FORWARD, backPower, vex.PERCENT)
        
        if abs(armPower) < 5:
            arms.stop(vex.BrakeType.HOLD)
            arms2.stop(vex.BrakeType.HOLD)
        else:
            arms.spin(vex.FORWARD, armPower, vex.PERCENT)
            arms2.spin(vex.FORWARD, armPower, vex.PERCENT)
        
        brain.screen.print_line(4, backPot.value(vex.DEGREES))

# Do not adjust the lines below

# Set up (but don't start) callbacks for autonomous and driver control periods.
competition.autonomous(autonomous)
competition.drivercontrol(drivercontrol)

# Run the pre-autonomous function.
pre_auton()

# Robot Mesh Studio runtime continues to run until all threads and
# competition callbacks are finished.
