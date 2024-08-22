import gc
import time
import math
from machine import Pin
from servo import ServoCluster, Calibration, Servo, servo2040, ANGULAR, LINEAR, CONTINUOUS


CoxaLength = 41
FemurLength = 80
TibiaLength = 135

PosX = 0
PosY = 0
PosZ = 0
RotX = 0
RotY = 0
RotZ = 0



BodyCenterOffsetX_1 = 63
BodyCenterOffsetX_2 = 81
BodyCenterOffsetX_3 = 63
BodyCenterOffsetX_4 = -63
BodyCenterOffsetX_5 = -81
BodyCenterOffsetX_6 = -63

BodyCenterOffsetY_1 = 83
BodyCenterOffsetY_2 = 0
BodyCenterOffsetY_3 = -83
BodyCenterOffsetY_4 = -83
BodyCenterOffsetY_5 = 0
BodyCenterOffsetY_6 = 83

#initial feet position

FeetPosX_1 = math.cos(60/180*math.pi)*(CoxaLength+FemurLength)
FeetPosZ_1 = TibiaLength
FeetPosY_1 = math.sin(60/180*math.pi)*(CoxaLength+FemurLength)


FeetPosX_2 = CoxaLength + FemurLength
FeetPosZ_2 = TibiaLength
FeetPosY_2 = 0


FeetPosX_3 = math.cos(60/180*math.pi)*(CoxaLength+FemurLength)
FeetPosZ_3 = TibiaLength
FeetPosY_3 = math.sin(-60/180*math.pi)*(CoxaLength+FemurLength)

FeetPosX_4 = -math.cos(60/180*math.pi)*(CoxaLength+FemurLength)
FeetPosZ_4 = TibiaLength
FeetPosY_4 = math.sin(-60/180*math.pi)*(CoxaLength+FemurLength)

FeetPosX_5 =-(CoxaLength+FemurLength)
FeetPosZ_5 = TibiaLength
FeetPosY_5 = 0

FeetPosX_6 = -math.cos(60/180*math.pi)*(CoxaLength+FemurLength)
FeetPosZ_6 = TibiaLength
FeetPosY_6 = math.sin(60/180*math.pi)*(CoxaLength+FemurLength)

#Body inverse kinematics

#leg 1
TotalY_1 = FeetPosY_1 + BodyCenterOffsetY_1 + PosY
TotalX_1 = FeetPosX_1 + BodyCenterOffsetX_1 + PosX
DistBodyCenterFeet_1 = math.sqrt((TotalY_1 ** 2) + (TotalX_1 ** 2))
AngleBodyCenterX_1 =  math.pi/2 - math.atan2(TotalX_1, TotalY_1)
RollZ_1 = math.tan(RotZ * math.pi/180)*TotalX_1
PitchZ_1 = math.tan(RotX * math.pi/180)*TotalY_1
BodyIKX_1 = math.cos(AngleBodyCenterX_1 + (RotZ*math.pi/180))* DistBodyCenterFeet_1 - TotalX_1
BodyIKY_1 = math.sin(AngleBodyCenterX_1 + (RotZ*math.pi/180))* DistBodyCenterFeet_1 - TotalY_1
BodyIKZ_1 = RollZ_1 + PitchZ_1

#leg 2
TotalY_2 = FeetPosY_2 + BodyCenterOffsetY_2 + PosY
TotalX_2 = FeetPosX_2 + BodyCenterOffsetX_2 + PosX
DistBodyCenterFeet_2 = math.sqrt((TotalY_2 ** 2) + (TotalX_2 ** 2))
AngleBodyCenterX_2 =  math.pi/2 - math.atan2(TotalX_2, TotalY_2)
RollZ_2 = math.tan(RotY * math.pi/180)*TotalX_2
PitchZ_2 = math.tan(RotX * math.pi/180)*TotalY_2
BodyIKX_2 = math.cos(AngleBodyCenterX_2 + (RotZ*math.pi/180))*DistBodyCenterFeet_2-TotalX_2
BodyIKY_2 = math.sin(AngleBodyCenterX_2 + (RotZ*math.pi/180))*DistBodyCenterFeet_2-TotalY_2
BodyIKZ_2 = RollZ_2 + PitchZ_2

#leg 3
TotalY_3 = FeetPosY_3 + BodyCenterOffsetY_3 + PosY
TotalX_3 = FeetPosX_3 + BodyCenterOffsetX_3 + PosX
DistBodyCenterFeet_3 = math.sqrt((TotalY_3 ** 2) + (TotalX_3 ** 2))
AngleBodyCenterX_3 =  math.pi/2 - math.atan2(TotalX_3, TotalY_3)
RollZ_3 = math.tan(RotY * math.pi/180)*TotalX_3
PitchZ_3 = math.tan(RotX * math.pi/180)*TotalY_3
BodyIKX_3 = math.cos(AngleBodyCenterX_3 + (RotZ*math.pi/180))*DistBodyCenterFeet_3-TotalX_3
BodyIKY_3 = math.sin(AngleBodyCenterX_3 + (RotZ*math.pi/180))*DistBodyCenterFeet_3-TotalY_3
BodyIKZ_3 = RollZ_3 + PitchZ_3

#leg 4
TotalY_4 = FeetPosY_4 + BodyCenterOffsetY_4 + PosY
TotalX_4 = FeetPosX_4 + BodyCenterOffsetX_4 + PosX
DistBodyCenterFeet_4 = math.sqrt((TotalY_4 ** 2) + (TotalX_4 ** 2))
AngleBodyCenterX_4 =  math.pi/2 - math.atan2(TotalX_4, TotalY_4)
RollZ_4 = math.tan(RotY * math.pi/180)*TotalX_4
PitchZ_4 = math.tan(RotX * math.pi/180)*TotalY_4
BodyIKX_4 = math.cos(AngleBodyCenterX_4 + (RotZ*math.pi/180))*DistBodyCenterFeet_4-TotalX_4
BodyIKY_4 = math.sin(AngleBodyCenterX_4 + (RotZ*math.pi/180))*DistBodyCenterFeet_4-TotalY_4
BodyIKZ_4 = RollZ_4 + PitchZ_4

#leg 5
TotalY_5 = FeetPosY_5 + BodyCenterOffsetY_5 + PosY
TotalX_5 = FeetPosX_5 + BodyCenterOffsetX_5 + PosX
DistBodyCenterFeet_5 = math.sqrt((TotalY_5 ** 2) + (TotalX_5 ** 2))
AngleBodyCenterX_5 =  math.pi/2 - math.atan2(TotalX_5, TotalY_5)
RollZ_5 = math.tan(RotY * math.pi/180)*TotalX_5
PitchZ_5 = math.tan(RotX * math.pi/180)*TotalY_5
BodyIKX_5 = math.cos(AngleBodyCenterX_5 + (RotZ*math.pi/180))*DistBodyCenterFeet_5-TotalX_5
BodyIKY_5 = math.sin(AngleBodyCenterX_5 + (RotZ*math.pi/180))*DistBodyCenterFeet_5-TotalY_5
BodyIKZ_5 = RollZ_5 + PitchZ_5

#leg 6
TotalY_6 = FeetPosY_6 + BodyCenterOffsetY_6 + PosY
TotalX_6 = FeetPosX_6 + BodyCenterOffsetX_6 + PosX
DistBodyCenterFeet_6 = math.sqrt((TotalY_6 ** 2) + (TotalX_6 ** 2))
AngleBodyCenterX_6 =  math.pi/2 - math.atan2(TotalX_6, TotalY_6)
RollZ_6 = math.tan(RotY * math.pi/180)*TotalX_6
PitchZ_6 = math.tan(RotX * math.pi/180)*TotalY_6
BodyIKX_6 = math.cos(AngleBodyCenterX_6 + (RotZ*math.pi/180))*DistBodyCenterFeet_6-TotalX_6
BodyIKY_6 = math.sin(AngleBodyCenterX_6 + (RotZ*math.pi/180))*DistBodyCenterFeet_6-TotalY_6
BodyIKZ_6 = RollZ_6 + PitchZ_6

#Leg Inverse kinematics

#Leg 1
NewPosX_1 = FeetPosX_1 + PosX + BodyIKX_1
NewPosZ_1 = FeetPosZ_1 + PosZ + BodyIKZ_1
NewPosY_1 = FeetPosY_1 + PosY + BodyIKY_1
CoxaFeetDist_1 = math.sqrt((NewPosX_1 ** 2) + (NewPosY_1 ** 2))
IKSW_1 = math.sqrt((CoxaFeetDist_1 - CoxaLength ) ** 2 + NewPosZ_1 ** 2)
IKA1_1 = math.atan((CoxaFeetDist_1 - CoxaLength)/NewPosZ_1)
IKA2_1 = math.acos((TibiaLength ** 2 - FemurLength ** 2 - IKSW_1 ** 2)/(-2 * IKSW_1 *  FemurLength))
TAngle_1 = math.acos((IKSW_1 ** 2 - TibiaLength ** 2 - FemurLength ** 2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_1 = 90 - TAngle_1 * 180/math.pi
IKFemurAngle_1 = 90 - (IKA1_1 + IKA2_1) * 180/math.pi
IKCoxaAngle_1 = 90 - math.atan2(NewPosX_1, NewPosY_1) * 180/math.pi

#Leg 2
NewPosX_2 = FeetPosX_2 + PosX + BodyIKX_2
NewPosZ_2 = FeetPosZ_2 + PosZ + BodyIKZ_2
NewPosY_2 = FeetPosY_2 + PosY + BodyIKY_2
CoxaFeetDist_2 = math.sqrt((NewPosX_2 ** 2)   + (NewPosY_2 ** 2))
IKSW_2 = math.sqrt((CoxaFeetDist_2 - CoxaLength ) ** 2 + NewPosZ_2 ** 2)
IKA1_2 = math.atan((CoxaFeetDist_2 - CoxaLength)/NewPosZ_2)
IKA2_2 = math.acos((TibiaLength ** 2 - FemurLength ** 2 - IKSW_2 ** 2)/(-2 * IKSW_2 *  FemurLength))
TAngle_2 = math.acos((IKSW_2 ** 2 - TibiaLength ** 2 - FemurLength ** 2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_2 = 90 - TAngle_2 * 180/math.pi
IKFemurAngle_2 = 90 - (IKA1_2 + IKA2_2) * 180/math.pi
IKCoxaAngle_2 = 90 - math.atan2(NewPosX_2, NewPosY_2) * 180/math.pi

#Leg 3
NewPosX_3 = FeetPosX_3 + PosX + BodyIKX_3
NewPosZ_3 = FeetPosZ_3 + PosZ + BodyIKZ_3
NewPosY_3 = FeetPosY_3 + PosY + BodyIKY_3
CoxaFeetDist_3 = math.sqrt((NewPosX_3 ** 2)   + (NewPosY_3 ** 2))
IKSW_3 = math.sqrt((CoxaFeetDist_3 - CoxaLength ) ** 2 + NewPosZ_3 ** 2)
IKA1_3 = math.atan((CoxaFeetDist_3 - CoxaLength)/NewPosZ_3)
IKA2_3 = math.acos((TibiaLength ** 2 - FemurLength ** 2 - IKSW_3 ** 2)/(-2 * IKSW_3 *  FemurLength))
TAngle_3 = math.acos((IKSW_3 ** 2 - TibiaLength ** 2 - FemurLength ** 2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_3 = 90 - TAngle_3 * 180/math.pi
IKFemurAngle_3 = 90 - (IKA1_3 + IKA2_3) * 180/math.pi
IKCoxaAngle_3 = 90 - math.atan2(NewPosX_3, NewPosY_3) * 180/math.pi

#Leg 4
NewPosX_4 = FeetPosX_4 + PosX + BodyIKX_4
NewPosZ_4 = FeetPosZ_4 + PosZ + BodyIKZ_4
NewPosY_4 = FeetPosY_4 + PosY + BodyIKY_4
CoxaFeetDist_4 = math.sqrt((NewPosX_4 ** 2)   + (NewPosY_4 ** 2))
IKSW_4 = math.sqrt((CoxaFeetDist_4 - CoxaLength ) ** 2 + NewPosZ_4 ** 2)
IKA1_4 = math.atan((CoxaFeetDist_4 - CoxaLength)/NewPosZ_4)
IKA2_4 = math.acos((TibiaLength ** 2 - FemurLength ** 2 - IKSW_4 ** 2)/(-2 * IKSW_4 *  FemurLength))
TAngle_4 = math.acos((IKSW_4 ** 2 - TibiaLength ** 2 - FemurLength ** 2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_4 = 90 - TAngle_4 * 180/math.pi
IKFemurAngle_4 = 90 - (IKA1_4 + IKA2_4) * 180/math.pi
IKCoxaAngle_4 = 90 - math.atan2(NewPosX_4, NewPosY_4) * 180/math.pi

#Leg 5
NewPosX_5 = FeetPosX_5 + PosX + BodyIKX_5
NewPosZ_5 = FeetPosZ_5 + PosZ + BodyIKZ_5
NewPosY_5 = FeetPosY_5 + PosY + BodyIKY_5
CoxaFeetDist_5 = math.sqrt((NewPosX_5 ** 2)   + (NewPosY_5 ** 2))
IKSW_5 = math.sqrt((CoxaFeetDist_5 - CoxaLength ) ** 2 + NewPosZ_5 ** 2)
IKA1_5 = math.atan((CoxaFeetDist_5 - CoxaLength)/NewPosZ_5)
IKA2_5 = math.acos((TibiaLength ** 2 - FemurLength ** 2 - IKSW_5 ** 2)/(-2 * IKSW_5 *  FemurLength))
TAngle_5 = math.acos((IKSW_5 ** 2 - TibiaLength ** 2 - FemurLength ** 2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_5 = 90 - TAngle_5 * 180/math.pi
IKFemurAngle_5 = 90 - (IKA1_5 + IKA2_5) * 180/math.pi
IKCoxaAngle_5 = 90 - math.atan2(NewPosX_5, NewPosY_5) * 180/math.pi

#Leg 6
NewPosX_6 = FeetPosX_6 + PosX + BodyIKX_6
NewPosZ_6 = FeetPosZ_6 + PosZ + BodyIKZ_6
NewPosY_6 = FeetPosY_6 + PosY + BodyIKY_6
CoxaFeetDist_6 = math.sqrt((NewPosX_6 ** 2)   + (NewPosY_6 ** 2))
IKSW_6 = math.sqrt((CoxaFeetDist_6 - CoxaLength ) ** 2 + NewPosZ_6 ** 2)
IKA1_6 = math.atan((CoxaFeetDist_6 - CoxaLength)/NewPosZ_6)
IKA2_6 = math.acos((TibiaLength ** 2 - FemurLength ** 2 - IKSW_6 ** 2)/(-2 * IKSW_6 *  FemurLength))
TAngle_6 = math.acos((IKSW_6 ** 2 - TibiaLength ** 2 - FemurLength ** 2)/(-2 * FemurLength * TibiaLength))
IKTibiaAngle_6 = 90 - TAngle_6 * 180/math.pi
IKFemurAngle_6 = 90 - (IKA1_6 + IKA2_6) * 180/math.pi
IKCoxaAngle_6 = 90 - math.atan2(NewPosX_6, NewPosY_6) * 180/math.pi


#Servo angles for each leg
CoxaAngle_1 = IKCoxaAngle_1 - 60
FemurAngle_1 = IKFemurAngle_1
TibiaAngle_1 = IKTibiaAngle_1

CoxaAngle_2 = IKCoxaAngle_2
FemurAngle_2 = IKFemurAngle_2
TibiaAngle_2 = IKTibiaAngle_2

CoxaAngle_3 = IKCoxaAngle_3 + 60
FemurAngle_3 = IKFemurAngle_3
TibiaAngle_3 = IKTibiaAngle_3

CoxaAngle_4 = IKCoxaAngle_4 - 240
FemurAngle_4 = IKFemurAngle_4
TibiaAngle_4 = IKTibiaAngle_4

CoxaAngle_5 = IKCoxaAngle_5 - 180
FemurAngle_5 = IKFemurAngle_5
TibiaAngle_5 = IKTibiaAngle_5

CoxaAngle_6 = IKCoxaAngle_6 - 120
FemurAngle_6 = IKFemurAngle_6
TibiaAngle_6 = IKTibiaAngle_6








#Variables containing Servo calibration values
r11 = Calibration()
r12 = Calibration()
r13 = Calibration()

l11 = Calibration()
l12 = Calibration()
l13 = Calibration()

r21 = Calibration()
r22 = Calibration()
r23 = Calibration()

l21 = Calibration()
l22 = Calibration()
l23 = Calibration()

r31 = Calibration()
r32 = Calibration()
r33 = Calibration()

l31 = Calibration()
l32 = Calibration()
l33 = Calibration()

r11.apply_two_pairs(1860, 1240, -45, 45)
r12.apply_two_pairs(1900, 1260, -45, 45)
r13.apply_two_pairs(1860, 1240, -45, 45)

l11.apply_two_pairs(1840, 1220, -45, 45)
l12.apply_two_pairs(1800, 1160, -45, 45)
l13.apply_two_pairs(1850, 1190, -45, 45)

r21.apply_two_pairs(1820, 1180, -45, 45)
r22.apply_two_pairs(1810, 1170, -45, 45)
r23.apply_two_pairs(1820, 1160, -45, 45)

l21.apply_two_pairs(1860, 1220, -45, 45)
l22.apply_two_pairs(1860, 1220, -45, 45)
l23.apply_two_pairs(1780, 1140, -45, 45)

r31.apply_two_pairs(1860, 1220, -45, 45)
r32.apply_two_pairs(1760, 1120, -45, 45)
r33.apply_two_pairs(1880, 1220, -45, 45)

l31.apply_two_pairs(1840, 1160, -45, 45)
l32.apply_two_pairs(1900, 1260, -45, 45)
l33.apply_two_pairs(1820, 1160, -45, 45)


# Turn on the relay to give power for the servos
power = Pin(servo2040.ADC0, Pin.OUT)
power.value(1)


"""
NOTE: ServoCluster uses the RP2040's PIO system, and as
such may have problems when running code multiple times.
If you encounter issues, try resetting your board.
"""

# Free up hardware resources ahead of creating a new ServoCluster
gc.collect()

# Create a servo cluster for pins 0 to 17, using PIO 0 and State Machine 0
START_PIN = servo2040.SERVO_1
END_PIN = servo2040.SERVO_18
servos = ServoCluster(pio=0, sm=0, pins=list(range(START_PIN, END_PIN + 1)))


#Calibrate the servos
servos.calibration(0, r31)
servos.calibration(1, r32)
servos.calibration(2, r33)
servos.calibration(3, l31)
servos.calibration(4, l32)
servos.calibration(5, l33)
servos.calibration(6, r21)
servos.calibration(7, r22)
servos.calibration(8, r23)
servos.calibration(9, l21)
servos.calibration(10, l22)
servos.calibration(11, l23)
servos.calibration(12, r11)
servos.calibration(13, r12)
servos.calibration(14, r13)
servos.calibration(15, l11)
servos.calibration(16, l12)
servos.calibration(17, l13)


#Servo angle offsets
CoxaOffset = 0
FemurOffset = 0
TibiaOffset = 0

CoxaAngle_1 = CoxaAngle_1 + CoxaOffset
CoxaAngle_2 = CoxaAngle_2 + CoxaOffset
CoxaAngle_3 = CoxaAngle_3 + CoxaOffset
CoxaAngle_4 = CoxaAngle_4 + CoxaOffset
CoxaAngle_5 = CoxaAngle_5 + CoxaOffset
CoxaAngle_6 = CoxaAngle_6 + CoxaOffset
FemurAngle_1 = FemurAngle_1 + FemurOffset
FemurAngle_2 = FemurAngle_2 + FemurOffset
FemurAngle_3 = FemurAngle_3 + FemurOffset
FemurAngle_4 = FemurAngle_4 - FemurOffset
FemurAngle_5 = FemurAngle_5 - FemurOffset
FemurAngle_6 = FemurAngle_6 - FemurOffset
TibiaAngle_1 = TibiaAngle_1 + TibiaOffset
TibiaAngle_2 = TibiaAngle_2 + TibiaOffset
TibiaAngle_3 = TibiaAngle_3 + TibiaOffset
TibiaAngle_4 = TibiaAngle_4 - TibiaOffset
TibiaAngle_5 = TibiaAngle_5 - TibiaOffset
TibiaAngle_6 = TibiaAngle_6 - TibiaOffset


#Leg 1 højre forben
servos.value(12, CoxaAngle_1)
servos.value(13, FemurAngle_1)
servos.value(14, TibiaAngle_1)

#Leg 2 højre midterben
servos.value(6, CoxaAngle_2)
servos.value(7, FemurAngle_2)
servos.value(8, TibiaAngle_2)

#Leg 3 højre bagben
servos.value(0, CoxaAngle_3)
servos.value(1, FemurAngle_3)
servos.value(2, TibiaAngle_3)

#Leg 4 venstre forben
servos.value(3, CoxaAngle_4)
servos.value(4, FemurAngle_4)
servos.value(5, TibiaAngle_4)

#Leg 5 venstre midterben
servos.value(9, CoxaAngle_5)
servos.value(10, FemurAngle_5)
servos.value(11, TibiaAngle_5)

#Leg 6 venstre bagben
servos.value(15, CoxaAngle_6)
servos.value(16, FemurAngle_6)
servos.value(17, TibiaAngle_6)



time.sleep(2)

#turn off the power for the servos
power.value(0)

# Disable the servos
servos.disable_all()

print("Leg 1")
print(CoxaAngle_1)
print(FemurAngle_1)
print(TibiaAngle_1)

print("Leg 2")
print(CoxaAngle_2)
print(FemurAngle_2)
print(TibiaAngle_2)

print("Leg 3")
print(CoxaAngle_3)
print(FemurAngle_3)
print(TibiaAngle_3)

print("Leg 4")
print(CoxaAngle_4)
print(FemurAngle_4)
print(TibiaAngle_4)

print("Leg 5")
print(CoxaAngle_5)
print(FemurAngle_5)
print(TibiaAngle_5)

print("Leg 6")
print(CoxaAngle_6)
print(FemurAngle_6)
print(TibiaAngle_6)
