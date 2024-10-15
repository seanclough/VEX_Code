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
left1         = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO18_1, False)
arms2         = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO18_1, True)
left2         = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO18_1, False)
right1        = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO18_1, True)
conveyor      = vex.Motor(vex.Ports.PORT5, vex.GearSetting.RATIO18_1, False)
right2        = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO18_1, True)
arms          = vex.Motor(vex.Ports.PORT7, vex.GearSetting.RATIO18_1, False)
back          = vex.Motor(vex.Ports.PORT8, vex.GearSetting.RATIO18_1, False)
frontLeft     = vex.Motor(vex.Ports.PORT9, vex.GearSetting.RATIO18_1, False)
inertial_21   = vex.Inertial(vex.Ports.PORT21)
leftEncoder   = vex.Encoder(brain.three_wire_port.a)
rightEncoder  = vex.Encoder(brain.three_wire_port.c)
middleEncoder = vex.Encoder(brain.three_wire_port.e)
claw          = vex.Pneumatics(brain.three_wire_port.g)
pot           = vex.Pot(brain.three_wire_port.h)
con           = vex.Controller(vex.ControllerType.PRIMARY)
#endregion config


# Creates a competition object that allows access to Competition methods.
competition = vex.Competition()

def findPower(current, wanted, intensity, acceleration, Kp, Ki):
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
    if abs(error) <= 5:
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
        findPower(arms.rotation(vex.DEGREES), position, speed, acceleration, .5, .001)
        arms.spin(vex.FORWARD, power, vex.PERCENT)
        arms2.spin(vex.FORWARD, power, vex.PERCENT)
    arms.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
def forwardMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        findPower(left1.rotation(vex.DEGREES), position, speed, acceleration, .3, .001)
        left1.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
    left1.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
def spinMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        inertialAngle = inertial_21.heading(vex.DEGREES)
        if inertial_21.heading(vex.DEGREES) > 180:
            inertialAngle -= 360
        findPower(inertialAngle, position, speed, acceleration, .5, .001)
        left1.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right1.spin(vex.REVERSE, power, vex.PERCENT)
        right2.spin(vex.REVERSE, power, vex.PERCENT)
    left1.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
def backMogoMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        findPower(back.rotation(vex.DEGREES), position, speed, acceleration, .5, .001)
        back.spin(vex.FORWARD, power, vex.PERCENT)
    back.stop(vex.BrakeType.HOLD)
def clawGrab():
    claw.open()
def clawLetGo():
    claw.close()

# MoveTowards Functions:
def moveBackTowards(position, intensity, acceleration):
    global backError, backTotalError, backPower, lastBackPower
    backError =  position - back.rotation(vex.DEGREES)
    if .001*backTotalError > 10:
       backTotalError = 10000
    if .001*backTotalError < -10:
       backTotalError = -10000
    backPower = .5*backError + .001*backTotalError
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
        doubleYellow()
        brain.screen.print_line(2, "Right side double yellow mogo")
    elif(value_choice >= 25 and value_choice < 80):
        AWP_line()
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
        doubleYellow()
        brain.screen.print_line(2, "No Auton Here! Did default code")
def doubleYellow():
    forwardAccel = .08
    clawLetGo()
    armsMoveTo(65, 100, 100)
    forwardMoveTo(1690, 100, forwardAccel)
    clawGrab()
    vex.wait(.3, vex.SECONDS)
    #armsMoveTo(250, 100, 100)
    forwardMoveTo(1000, 100, forwardAccel)
    spinMoveTo(130, 100, forwardAccel)
    backMogoMoveTo(1240, 100, 100)
    forwardMoveTo(200, 100, forwardAccel)
    back.spin(vex.FORWARD, -10,vex.PERCENT)
    forwardMoveTo(-200, 10, 100)
    back.stop(vex.BrakeType.HOLD)
    forwardMoveTo(1800, 100, forwardAccel)
def AWP_line():
    forwardAccel = .08
    forwardMoveTo(-200, 30, forwardAccel)
    backMogoMoveTo(400, 60, 100)
    forwardMoveTo(100, 50, forwardAccel)
    backMogoMoveTo(1240, 100, 100)
    spinMoveTo(12, 100, 10)
    forwardMoveTo(-350, 30, forwardAccel)
    #backMogoMoveTo(400, 100, 100)
    forwardMoveTo(100, 10, forwardAccel)
def rampRings():
    forwardAccel = 1
    """
    backMogoMoveTo(xxx score ring, 100, 100)
    forwardMoveTo(xxx (ring falls off), 100, forwardAccel)
    """ 
def leftYellow():
    forwardAccel = .08
    clawLetGo()
    armsMoveTo(70, 100, 100)
    forwardMoveTo(1670, 100, forwardAccel)
    clawGrab()
    armsMoveTo(120, 100, 100)
    forwardMoveTo(100, 100, forwardAccel)
    
def pre_auton():
    inertial_21.calibrate()

def autonomous():
    pickAuton()

def drivercontrol():
    global done, totalError, power, lastPower
    global backError, backTotalError, backPower, lastBackPower
    global armError, armTotalError, armPower, lastArmPower
    global value_choice
    
    left1.stop(vex.BrakeType.HOLD)
    arms2.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right1.stop(vex.BrakeType.HOLD)
    conveyor.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    arms.stop(vex.BrakeType.HOLD)
    back.stop(vex.BrakeType.HOLD)
    
    # Slowmode variables
    ready = True
    slowmode = False
    timer_2 = timer.Timer()
    timer_2.start()
    
    ready2 = True
    slowmode2 = False
    timer_3 = timer.Timer()
    timer_3.start()
    
    # starting positions
    backPosition0 = False
    backPosition1 = False
    backPosition2 = False
    armOptimal = False
    
    
    backError = 0
    backTotalError = 0
    backPower = 0
    lastBackPower = 0
    
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    
    while True:
        # slowmode code:
        if con.buttonA.pressing() and ready:
            slowmode = not slowmode
            ready = False
            timer_2.reset()
            timer_2.start()
        if timer_2.elapsed_time() > .6:
            ready = True
        
        # Drivetrain code (to make it move)
        forwardPower = con.axis2.position(vex.PERCENT) #multipliers can be added
        spinPower = con.axis4.position(vex.PERCENT)*.7 #multiplier IS added (to make turning slower)
        rightPower = forwardPower-spinPower
        leftPower = forwardPower+spinPower
        if slowmode:
            rightPower *= .2
            leftPower *= .2
        right1.spin(vex.FORWARD, rightPower, vex.PERCENT)
        right2.spin(vex.FORWARD, rightPower, vex.PERCENT)
        left1.spin(vex.FORWARD, leftPower, vex.PERCENT)
        left2.spin(vex.FORWARD, leftPower, vex.PERCENT)
        
        if con.buttonL1.pressing():
            armOptimal = False
            armPower = 100
        elif con.buttonL2.pressing():
            armOptimal = False
            armPower = -100
        elif con.buttonB.pressing(): #or arms.rotation(vex.DEGREES) <= 85"""
            armOptimal = True
        if ((not con.buttonL2.pressing()) and (not con.buttonL1.pressing())) and (not armOptimal):
            armPower = 0
        
        if con.buttonLeft.pressing() and backPosition0 and not backPosition2:
            backPosition1 = True
            backPosition0 = False
            done = False
            ready2 = False
            timer_3.reset()
            timer_3.start()
        if timer_3.elapsed_time() > .25:
            ready2 = True
        if con.buttonLeft.pressing() and ready2 and backPosition1:
            backPosition2 = True
            backPosition1 = False
            done = False
        if con.buttonRight.pressing():
            backPosition0 = True
            done = False
        if backPosition0:
            backPosition1 = False
            backPosition2 = False
        brain.screen.print_line(5, timer_3.elapsed_time())
        brain.screen.print_line(6, ready2)
        
        if con.buttonUp.pressing(): 
            backPosition0 = False
            backPosition1 = False
            backPosition2 = False
            backPower = -100
        if con.buttonDown.pressing(): 
            backPosition0 = False
            backPosition1 = False
            backPosition2 = False
            backPower = 100
        if not con.buttonUp.pressing() and not con.buttonDown.pressing() and not backPosition0 and not backPosition1 and not backPosition2:
            backPower = 0
        
        if con.buttonR1.pressing(): 
            clawGrab()
        if con.buttonR2.pressing():
            clawLetGo()
        
        if backPosition0:
            moveBackTowards(1240, 100, 100)
        if backPosition1:
            moveBackTowards(900, 100, 100)
        if backPosition2:
            moveBackTowards(400, 100, 100)
        if armOptimal:
            moveArmTowards(20, 100, 100)
        
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

# Do not adjust the lines below

# Set up (but don't start) callbacks for autonomous and driver control periods.
competition.autonomous(autonomous)
competition.drivercontrol(drivercontrol)

# Run the pre-autonomous function.
pre_auton()

# Robot Mesh Studio runtime continues to run until all threads and
# competition callbacks are finished.
