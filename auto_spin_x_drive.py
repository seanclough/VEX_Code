import vex
import drivetrain
import smartdrive
import math
import sys
import random

#region config
brain       = vex.Brain()
motor_1     = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO18_1, False)
motor_2     = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO18_1, False)
motor_3     = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO18_1, False)
motor_4     = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO18_1, False)
inertial_21 = vex.Inertial(vex.Ports.PORT21)
con         = vex.Controller(vex.ControllerType.PRIMARY)
#endregion config

# main thread
a = 0
b = 0
c = 0
f = 0
m_one = 0
m_two = 0
m_three = 0
m_four = 0
ra = 0
ri = 0
raw_gyro = 0
rN = 0
raN = 0
l = 0
s = 0
raw_m = []
new_m = []
actual_m = []
inertial_21.calibrate()

def find_ra(raw_gyro):
  global ra
  if raw_gyro>180:
    ra= (raw_gyro-360)/180*math.pi
  else:
    ra= raw_gyro/180*math.pi

def find_ri(a, b):
  global ri
  if b<0: 
      ri=-math.atan(a/b)-math.pi/2
  elif b>0:
      ri=-math.atan(a/b)+math.pi/2
  else: 
      if a<0:
          ri=math.pi
      else:
          ri=0
    
def scale_ab(): 
    global a, b
    #the jank reduction tool
    s = math.sqrt(a**2+b**2)/100
    a=a/s
    b=b/s
    
def scale_motor_sine_funtions(raw_m):
    global c, f, new_m, actual_m
    l = 0
    new_m = []
    actual_m = []
    #finding largest value and dividing raw_m by it
    for count in range(len(raw_m)):
        if math.fabs(raw_m[count]) > math.fabs(l):
            l = math.fabs(raw_m[count])
    for motor_value in raw_m:
        new_m.append(float(motor_value) / l)
    #scale c and f if needed
    if c+f > 100:
        c = c/(f+c)*100.
        f= 100-c
    #update motor array
    for motor_value in new_m:
        actual_m.append(motor_value*f + c)
        
def find_c():
    global c, raN
    ca=con.axis3.position(vex.PERCENT)
    cb=con.axis4.position(vex.PERCENT)
    if math.sqrt(ca**2+cb**2)>100:
        cs=math.sqrt(ca**2+cb**2)/100
        ca=ca/cs
        cb=cb/cs
    cf=math.sqrt(ca**2+cb**2)
    if cb<0: 
        rc=-math.atan(ca/cb)-math.pi/2
    elif cb>0:
        rc=-math.atan(ca/cb)+math.pi/2
    else: 
        if ca<0:
            rc=math.pi
        else:
            rc=0
    rac=raN-rc
    if rac>0 and rac<math.pi:
        c=-1
    elif rac<-math.pi:
        c=-1
    else:
        c=1
        
    #bufferamount=50
    #power=1
    
    if -.5<rac<.5 or rac<-2*(math.pi)+.5 or rac>2*(math.pi)-.5:
        c=c*4*rac**2
    
    c=c*cf
    
def auto():
  global raw_gyro, a, b, f, m, l, ra, ri, c, m_one, m_two, m_three, m_four, rN, raN, new_m, actual_m
  # Place autonomous code here
  pass

def driver():
  global raw_gyro, a, b, f, m, l, ra, ri, c, m_one, m_two, m_three, m_four, rN, raN, new_m, actual_m
  motor_1.stop(vex.BrakeType.HOLD)
  motor_2.stop(vex.BrakeType.HOLD)
  motor_3.stop(vex.BrakeType.HOLD)
  motor_4.stop(vex.BrakeType.HOLD)
  rN = 0
  while True:
    a = con.axis2.position(vex.PERCENT)
    b = con.axis1.position(vex.PERCENT)
    if math.sqrt(a ** 2 + b ** 2)>100:
        scale_ab()
    f = math.sqrt(a ** 2 + b ** 2)
    find_ri(a, b)
    find_ra(inertial_21.heading(vex.DEGREES))
    raN = ra - rN
    find_c()
    m_one = -math.sin(raN-math.pi/4-ri)
    m_two = -math.sin(raN+math.pi/4-ri)
    m_three = math.sin(raN+math.pi/4-ri)
    m_four = math.sin(raN-math.pi/4-ri)
    raw_m = [m_one, m_two, m_three, m_four]
    scale_motor_sine_funtions(raw_m)
    motor_1.spin(vex.FORWARD, (actual_m[0]), vex.PERCENT)
    motor_2.spin(vex.FORWARD, (actual_m[1]), vex.PERCENT)
    motor_3.spin(vex.FORWARD, (actual_m[2]), vex.PERCENT)
    motor_4.spin(vex.FORWARD, (actual_m[3]), vex.PERCENT)
    if con.buttonX.pressing():
      rN = ra
      """
      so you can reset the gyro while the program is running
      just spin so forward on the bot is forward for you and push x
      then next time you run it, remember to actually let the gyro calibrate
      """
    vex.wait(.015,vex.SECONDS)
    
competition = vex.Competition()
competition.autonomous(auto)
competition.drivercontrol(driver)


