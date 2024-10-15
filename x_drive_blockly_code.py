#originally programmed in blockly, converted to python

import vex
import drivetrain
import smartdrive
import math
import sys

#region config
brain       = vex.Brain()
motor_1     = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO18_1, False)
motor_2     = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO18_1, False)
motor_3     = vex.Motor(vex.Ports.PORT3, vex.GearSetting.RATIO18_1, False)
motor_4     = vex.Motor(vex.Ports.PORT4, vex.GearSetting.RATIO18_1, False)
inertial_21 = vex.Inertial(vex.Ports.PORT21)
con         = vex.Controller(vex.ControllerType.PRIMARY)
#endregion config

raw_gyro = None
a = None
b = None
f = None
ra = None
ri = None
c = None
m_one = None
m_two = None
m_three = None
m_four = None
rN = None
raN = None

competition = vex.Competition()

def driver():
  global raw_gyro, a, b, f, ra, ri, c, m_one, m_two, m_three, m_four, rN, raN
  # Place drive control code here, inside the loop
  motor_1.stop(vex.BrakeType.HOLD)
  motor_2.stop(vex.BrakeType.HOLD)
  motor_3.stop(vex.BrakeType.HOLD)
  motor_4.stop(vex.BrakeType.HOLD)
  while True:
    a = con.axis2.position(vex.PERCENT)
    b = con.axis1.position(vex.PERCENT)
    c = con.axis4.position(vex.PERCENT)
    f = math.sqrt(a ** 2 + b ** 2)
    if f != 0:
      find_ri(a, b, f)
    find_ra(inertial_21.heading(vex.DEGREES))
    raN = ra - rN
    m_one = ((-1 * f) * math.sqrt(2)) * math.sin((raN + (-45 - ri)) / 180.0 * math.pi) + c
    m_two = ((-1 * f) * math.sqrt(2)) * math.sin((raN + (45 - ri)) / 180.0 * math.pi) + c
    m_three = ((1 * f) * math.sqrt(2)) * math.sin((raN + (45 - ri)) / 180.0 * math.pi) + c
    m_four = ((1 * f) * math.sqrt(2)) * math.sin((raN + (-45 - ri)) / 180.0 * math.pi) + c
    motor_1.spin(vex.FORWARD, m_one, vex.PERCENT)
    motor_2.spin(vex.FORWARD, m_two, vex.PERCENT)
    motor_3.spin(vex.FORWARD, m_three, vex.PERCENT)
    motor_4.spin(vex.FORWARD, m_four, vex.PERCENT)
    if con.buttonX.pressing():
      rN = ra
competition.drivercontrol(driver)

def find_ra(raw_gyro):
  global a, b, f, ra, ri, c, m_one, m_two, m_three, m_four, rN, raN
  if raw_gyro > 180:
    ra = raw_gyro - 360
  else:
    ra = raw_gyro

def auto():
  global raw_gyro, a, b, f, ra, ri, c, m_one, m_two, m_three, m_four, rN, raN
  # Place autonomous code here
  pass
competition.autonomous(auto)

def find_ri(a, b, f):
  global raw_gyro, ra, ri, c, m_one, m_two, m_three, m_four, rN, raN
  if math.acos(float(a) / f) / math.pi * 180 == 180:
    ri = 180
  if math.acos(float(a) / f) / math.pi * 180 == math.asin(float(b) / f) / math.pi * 180:
    ri = math.acos(float(a) / f) / math.pi * 180
  if -1 * (math.acos(float(a) / f) / math.pi * 180) == math.asin(float(b) / f) / math.pi * 180:
    ri = -1 * (math.acos(float(a) / f) / math.pi * 180)
  if 1 * (math.acos(float(a) / f) / math.pi * 180) == -1 * (math.asin(float(b) / f) / math.pi * 180) + 180:
    ri = 1 * (math.acos(float(a) / f) / math.pi * 180)
  if -1 * (math.acos(float(a) / f) / math.pi * 180) == -1 * (math.asin(float(b) / f) / math.pi * 180) + 180:
    ri = -1 * (math.acos(float(a) / f) / math.pi * 180)
  if 1 * (math.acos(float(a) / f) / math.pi * 180) == -1 * (math.asin(float(b) / f) / math.pi * 180) - 180:
    ri = 1 * (math.acos(float(a) / f) / math.pi * 180)
  if -1 * (math.acos(float(a) / f) / math.pi * 180) == -1 * (math.asin(float(b) / f) / math.pi * 180) - 180:
    ri = -1 * (math.acos(float(a) / f) / math.pi * 180)


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
# All activities that occur before competition start
# Example: setting initial positions
pass

