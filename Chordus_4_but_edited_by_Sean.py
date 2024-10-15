# VEX V5 Python Project with Competition Template
import sys
import vex
from vex import *
import motor_group
import drivetrain
import smartdrive
# those are automatic ^
# these I added manually v
import math
import timer
# math is used for...math and timer is used for...the timer

# you set up this part over there -> 
# notice how some are "False" and some are "True"
# that just reverses the motor (forward -> backward, backward -> forward)
#region config
brain       = vex.Brain()
arm_        = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO18_1, False)
arm2_       = vex.Motor(vex.Ports.PORT6, vex.GearSetting.RATIO18_1, True)
clawV3      = vex.Motor(vex.Ports.PORT14, vex.GearSetting.RATIO18_1, True)
back        = vex.Motor(vex.Ports.PORT15, vex.GearSetting.RATIO18_1, False)
right2      = vex.Motor(vex.Ports.PORT16, vex.GearSetting.RATIO18_1, True)
left2       = vex.Motor(vex.Ports.PORT17, vex.GearSetting.RATIO18_1, False)
left        = vex.Motor(vex.Ports.PORT18, vex.GearSetting.RATIO18_1, True)
right       = vex.Motor(vex.Ports.PORT19, vex.GearSetting.RATIO18_1, False)
inertial_20 = vex.Inertial(vex.Ports.PORT20)
pot         = vex.Pot(brain.three_wire_port.a)
dt          = drivetrain.Drivetrain((left, left2), (right, right2), 319.1858, 292.1, vex.DistanceUnits.MM, 1)
con         = vex.Controller(vex.ControllerType.PRIMARY)
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
    if abs(error) <= 1:
       done = True
    totalError += error
    lastPower = power
    
def findPower2(current, wanted, intensity, Kp, Ki):
    # global means these variables can be used in other functions
    global done, totalError, power
    error =  wanted - current
    if Ki*totalError > 10:
       totalError = 10./Ki
    if Ki*totalError < -10:
       totalError = -10./Ki
    power = Kp*error + Ki*totalError
    if power > intensity:
       power = intensity
    if power*-1 > intensity:
       power = -1*intensity
    if abs(error) <= 5:
       done = True
    totalError += error

"""
Movement Functions written by Sean
"""
    
def armsMoveTo(position, speed, acceleration):
    global done, totalError, power
    done = False
    totalError = 0
    while not done: 
        findPower(arm_.rotation(vex.DEGREES), position, speed, acceleration, .5, .0001)
        arm_.spin(vex.FORWARD, power, vex.PERCENT)
        arm2_.spin(vex.FORWARD, power, vex.PERCENT)
    arm_.stop(vex.BrakeType.HOLD)
    arm2_.stop(vex.BrakeType.HOLD)


def forwardMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    while not done: 
        findPower(left.rotation(vex.DEGREES), position, speed, acceleration, .3, .0001)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    
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

def hyperForward(position, speed, acceleration, Kp, Ki):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        findPower(left1.rotation(vex.DEGREES), position, speed, acceleration, Kp, Ki)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    
def spinMoveTo(position, speed, acceleration):
    global done, totalError, power, lastPower
    done = False
    totalError = 0
    lastPower = 0
    while not done: 
        inertialAngle = inertial_20.heading(vex.DEGREES)
        if inertial_20.heading(vex.DEGREES) > 180:
            inertialAngle -= 360
            
        brain.screen.print_line(1, inertialAngle)
        findPower(inertialAngle, position, speed, acceleration, .5, .001)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.REVERSE, power, vex.PERCENT)
        right2.spin(vex.REVERSE, power, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    
    
    
    
def backMogoMoveTo(position, speed, acceleration):
    global done, totalError, power
    done = False
    totalError = 0
    while not done: 
        findPower(back.rotation(vex.DEGREES), position, speed, acceleration, .5, .001)
        back.spin(vex.FORWARD, power, vex.PERCENT)
    back.stop(vex.BrakeType.HOLD)

def clawGrab(speed):
    global done, totalError, power, timer_
    done = False
    totalError = 0
    timer_.reset()
    timer_.start()
    while not done:
        findPower(clawV3.rotation(vex.DEGREES), -500, speed, 100, .5, .001)
        clawV3.spin(vex.FORWARD, power, vex.PERCENT)
        if timer_.elapsed_time()>2:
            done = True
    timer_.stop()
    clawV3.stop(vex.BrakeType.HOLD)

def clawLetGo(speed):
    global done, totalError, power
    done = False
    totalError = 0
    while not done:
        findPower(clawV3.rotation(vex.DEGREES), -10, speed, 100, .5, .001)
        clawV3.spin(vex.FORWARD, power, vex.PERCENT)
    clawV3.stop(vex.BrakeType.HOLD)

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

def moveClawTowards(position, intensity, acceleration):
    global clawError, clawTotalError, clawPower, lastClawPower, timer_34
    clawError =  position - clawV3.rotation(vex.DEGREES)
    if .001*clawTotalError > 10:
       clawTotalError = 10000
    if .001*clawTotalError < -10:
       clawTotalError = -10000
    clawPower = .5*clawError + .001*clawTotalError
    clawPower = (1-acceleration/100.)*lastClawPower + acceleration/100.*clawPower
    if clawPower > intensity:
       clawPower = intensity
    if clawPower*-1 > intensity:
       clawPower = -1*intensity
    brain.screen.print_line(5, timer_34.elapsed_time())
    if timer_34.elapsed_time()>1.5:
            power = 0
    clawTotalError += clawError
    lastClawPower = clawPower

def moveArmTowards(position, intensity, acceleration):
    global armError, armTotalError, armPower, lastArmPower
    armError =  position - arm_.rotation(vex.DEGREES)
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
    
def pickAuton():
    global value_choice
    value_choice = pot.value(vex.DEGREES)
    brain.screen.print_line(4, value_choice)
    while value_choice == 0.0:
        value_choice = pot.value(vex.DEGREES)
    brain.screen.print_line(3, pot.value(vex.DEGREES)) 
    brain.screen.print_line(1, "Selected Auton: ")
    if(value_choice >= 0 and value_choice < 25):
        single_yellow()
        brain.screen.print_line(2, "Right side double yellow mogo")
    elif(value_choice >= 25 and value_choice < 80):
        bingle_yellow(30)
        brain.screen.print_line(2, "AWP Line, scores rings and moves mogo")
    elif(value_choice >= 80 and value_choice < 130):
        newForwardMoveTo(1000, 100, .0001, .5, .001, 10, 200)
        brain.screen.print_line(2, "Ramp Rings (for AWP)")
    elif(value_choice >= 130 and value_choice < 200):
        bingle_yellow(75)
        brain.screen.print_line(2, "Left side single yellow mogo")
    elif(value_choice >= 200 and value_choice < 300):
        brain.screen.print_line(2, "Empty auton slot")
    else:
        double_yellow_yellow()
        brain.screen.print_line(2, "No Auton Here! Did default code")
    
def single_yellow():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    # Place autonomous code here
    #inertial_21.calibrate()
    #variable initialization
    timer_ = timer.Timer()
    #stuff that doesn't move right away needs to hold
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    lastPower = 0
    
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    

    while not done: 
        
        findPower(left.rotation(vex.DEGREES), 1104, 100, 6, .3, .0005)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(-460, 100, 100)
        arm_.spin(vex.FORWARD, armPower, vex.PERCENT)
        arm2_.spin(vex.FORWARD, armPower, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    arm_.stop(vex.BrakeType.HOLD)
    arm2_.stop(vex.BrakeType.HOLD)
    
    done = False
    totalError = 0
    
    timer_.reset()
    timer_.start()
    
    vex.wait(.2, vex.SECONDS)
    
    # reset ckaw fucntinon to the facet of .001
    # open claw at the start
    while not done:
        findPower2(clawV3.rotation(vex.DEGREES), -500, 100, .5, .001)
        clawV3.spin(vex.FORWARD, power, vex.PERCENT)
        if timer_.elapsed_time()>1:
            done = True
    timer_.stop()
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    
    vex.wait(.2, vex.SECONDS)
    
    while not done: 
        findPower(left.rotation(vex.DEGREES), 50, 100, 7, .8, .0005)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(-615, 100, 100)
        arm_.spin(vex.FORWARD, armPower, vex.PERCENT)
        arm2_.spin(vex.FORWARD, armPower, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    arm_.stop(vex.BrakeType.HOLD)
    arm2_.stop(vex.BrakeType.HOLD)

def bingle_yellow(speed):
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    # Place autonomous code here
    #inertial_21.calibrate()
    #variable initialization
    timer_ = timer.Timer()
    #stuff that doesn't move right away needs to hold
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    lastPower = 0
    
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    

    while not done: 
        findPower(left.rotation(vex.DEGREES), 1104, speed, 1, .3, .0005)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(-430, 100, 100)
        arm_.spin(vex.FORWARD, armPower, vex.PERCENT)
        arm2_.spin(vex.FORWARD, armPower, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    arm_.stop(vex.BrakeType.HOLD)
    arm2_.stop(vex.BrakeType.HOLD)
    
    done = False
    totalError = 0
    
    timer_.reset()
    timer_.start()
    
    vex.wait(.2, vex.SECONDS)
    
    # reset ckaw fucntinon to the facet of .001
    # open claw at the start
    while not done:
        findPower2(clawV3.rotation(vex.DEGREES), -500, 100, .5, .0001)
        clawV3.spin(vex.FORWARD, power, vex.PERCENT)
        if timer_.elapsed_time()>1:
            done = True
    timer_.stop()
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    
    vex.wait(.2, vex.SECONDS)
    

    
    
    while not done: 
        
        findPower(left.rotation(vex.DEGREES), 50, speed, 1, .8, .0005)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(-615, 100, 100)
        arm_.spin(vex.FORWARD, armPower, vex.PERCENT)
        arm2_.spin(vex.FORWARD, armPower, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    arm_.stop(vex.BrakeType.HOLD)
    arm2_.stop(vex.BrakeType.HOLD)
    
    
def double_yellow():
    global done, totalError, power, lastPower
    global armError, armTotalError, armPower, lastArmPower
    # Place autonomous code here
    #inertial_21.calibrate()
    #variable initialization
    timer_ = timer.Timer()
    #stuff that doesn't move right away needs to hold
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    lastPower = 0
    
    armError = 0
    armTotalError = 0
    armPower = 0
    lastArmPower = 0
    

    while not done: 
        
        findPower(left.rotation(vex.DEGREES), 1104, 100, 6, .3, .0005)
        left.spin(vex.FORWARD, power, vex.PERCENT)
        left2.spin(vex.FORWARD, power, vex.PERCENT)
        right.spin(vex.FORWARD, power, vex.PERCENT)
        right2.spin(vex.FORWARD, power, vex.PERCENT)
        moveArmTowards(-460, 100, 100)
        arm_.spin(vex.FORWARD, armPower, vex.PERCENT)
        arm2_.spin(vex.FORWARD, armPower, vex.PERCENT)
    left.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    arm_.stop(vex.BrakeType.HOLD)
    arm2_.stop(vex.BrakeType.HOLD)
    
    done = False
    totalError = 0
    
    timer_.reset()
    timer_.start()
    
    vex.wait(.2, vex.SECONDS)
    
    # reset ckaw fucntinon to the facet of .001
    # open claw at the start
    while not done:
        findPower2(clawV3.rotation(vex.DEGREES), -500, 100, .5, .001)
        clawV3.spin(vex.FORWARD, power, vex.PERCENT)
        if timer_.elapsed_time()>1:
            done = True
    timer_.stop()
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    
    moveArmTowards(-615, 100, 100)
    spinMoveTo(90,100,.08)
    moveBackTowards(1600/2, 100, 100)
    forwardMoveTo(900,100,.08)
    moveBackTowards(9000/2, 100, 100)
    # forwardMoveTo(1400,100,.08)

    
    
    

def pre_auton():
    # All activities that occur before competition start
    # Example: setting initial positions
    inertial_20.calibrate()

    
def autonomous():
    global done, totalError, power, lastPower, value_choice
    global armError, armTotalError, armPower, lastArmPower
    # Place autonomous code here
    #inertial_21.calibrate()
    #variable initialization
    timer_ = timer.Timer()
    #stuff that doesn't move right away needs to hold
    right.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    clawV3.stop(vex.BrakeType.HOLD)
    done = False
    totalError = 0
    lastPower = 0
    armError = 0
    armTotalError = 0
    lastArmPower = 0
    
    pickAuton()

    

    
    

    
    
def drivercontrol():
    # this stuff runs when you are actually driving the bot
    global done, totalError, power, lastPower, backError, backTotalError, backPower, lastBackPower, clawError, clawTotalError, clawPower, lastClawPower, armError, armTotalError, armPower, lastArmPower, timer_34
    global value_choice
    # Place drive control code here, inside the loop
    # initializes variables and stops motors
    left.stop(vex.BrakeType.HOLD)
    right.stop(vex.BrakeType.HOLD)
    left2.stop(vex.BrakeType.HOLD)
    right2.stop(vex.BrakeType.HOLD)
    rightPower = 0
    leftPower = 0 
    done = False
    ready = True
    slowmode = False
    timer_2 = timer.Timer()
    timer_2.start()
    backPosition = False
    clawPosition = True
    armOptimal = False
    backError = 0
    backTotalError = 0
    clawError = 0
    clawTotalError = 0
    armError = -315 - arm_.rotation(vex.DEGREES)
    armTotalError = 0
    backPower = 0
    clawPower = 0
    armPower = 0
    lastBackPower = 0
    lastClawPower = 0
    lastArmPower = 0
    timer_34 = timer.Timer()
    
    newForwardMoveTo(1000, 100, .0001, 30, .001, 10, 100)
    while True:

        if con.buttonX.pressing() and ready:
            slowmode = not slowmode
            ready = False
            timer_2.reset()
            timer_2.start()
        if timer_2.elapsed_time() > .6:
            ready = True
        
        # ignore turn power, Long didn't want it
        # spin + forward = turn so that's fine
        # spin 100% = right goes -100, left goes 100 = [-100,100]
        # forward 100% = both go 100 = [100, 100]
        # 50% spin, 50% forward = [-50, 50] + [50, 50] = [0, 100] = right goes 0%, left goes 100%
        # motors turn >100 to 100 automatically
        
        forwardPower = con.axis2.position(vex.PERCENT) #multipliers can be added
        spinPower = con.axis4.position(vex.PERCENT)*.7 #multiplier IS added (to make turning slower)
        # turnPower is commented out so it is inactive
        #turnPower = con.axis1.position(vex.PERCENT)
        turnPower = 0
        
        if turnPower > 0:
            rightPower = forwardPower-spinPower
            leftPower = forwardPower+spinPower+turnPower
        else: 
            rightPower = forwardPower - spinPower - turnPower
            leftPower = forwardPower + spinPower
        
        if slowmode:
            rightPower *= .2
            leftPower *= .2
        
        right.spin(vex.FORWARD, rightPower, vex.PERCENT)
        right2.spin(vex.FORWARD, rightPower, vex.PERCENT)
        left.spin(vex.FORWARD, leftPower, vex.PERCENT)
        left2.spin(vex.FORWARD, leftPower, vex.PERCENT)
        
        # Switched left buttons arm powers from positive to negative
        if con.buttonL1.pressing():
            armOptimal = False
            armPower = -100
        elif con.buttonL2.pressing():
            armOptimal = False
            armPower = 100
        elif con.buttonB.pressing() or con.buttonA.pressing():
            armOptimal = True
        
        if ((not con.buttonL2.pressing()) and (not con.buttonL1.pressing())) and (not armOptimal):
            armPower = 0
        
        if con.buttonR1.pressing():
            clawPosition = False
        elif con.buttonR2.pressing():
            clawPosition = True
            timer_34.reset()
            timer_34.start() 
        
        if con.buttonUp.pressing():
            backPosition = False
        elif con.buttonDown.pressing():
            backPosition = True
            
        if con.buttonLeft.pressing():
            moveBackTowards(2500/2, 100, 100)
        
        
        
        if backPosition:
            moveBackTowards(9900/2, 100, 100)
        else:
            moveBackTowards(1600/2, 100, 100)
            
        if clawPosition:
            moveClawTowards(-500, 100, 100)
        else:
            moveClawTowards(-10, 100, 100)
        if armOptimal:
            moveArmTowards(-340, 100, 100)
        
        
        
        if abs(backPower) < 5:
            back.stop(vex.BrakeType.HOLD)
        else:
            back.spin(vex.FORWARD, backPower, vex.PERCENT)
        
        if abs(clawPower) < 5:
            clawV3.stop(vex.BrakeType.HOLD)
        else:
            clawV3.spin(vex.FORWARD, clawPower, vex.PERCENT)
        
        if abs(armPower) < 5:
            arm_.stop(vex.BrakeType.HOLD)
            arm2_.stop(vex.BrakeType.HOLD)
        else:
            arm_.spin(vex.FORWARD, armPower, vex.PERCENT)
            arm2_.spin(vex.FORWARD, armPower, vex.PERCENT)
        
        brain.screen.print_line(1, inertial_20.heading(vex.DEGREES))
        brain.screen.print_line(2, pot.value(vex.DEGREES))
            

# Do not adjust the lines below

# Set up (but don't start) callbacks for autonomous and driver control periods.
competition.autonomous(autonomous)
competition.drivercontrol(drivercontrol)

# Run the pre-autonomous function.
pre_auton()

# Robot Mesh Studio runtime continues to run until all threads and
# competition callbacks are finished.