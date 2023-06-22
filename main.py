import time
import math
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from gpiozero import RotaryEncoder

# GPIO setmode
GPIO.setmode(GPIO.BCM)

# Define GPIO for Motors RIGHT SIDE
IN3 = 6
IN4 = 13
ENB = 26

# Define GPIO for Motors LEFT SIDE
IN1 = 4
IN2 = 27
ENA = 22

# Define RIGHT Encoder
ENCRA = 25
ENCRB = 5

# Define LEFT Encoder
ENCLA = 23
ENCLB = 24

# Set pins for Motors RIGHT SIDE
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Set pins for Motors LEFT SIDE
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set pins for RIGHT Encoder
encoderRight = RotaryEncoder(ENCRA, ENCRB, max_steps=0)

# Set pins for LEFT Encoder
encoderLeft = RotaryEncoder(ENCLA, ENCLB, max_steps=0)

# Reset Driver pins
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
GPIO.output(IN3, GPIO.LOW)
GPIO.output(IN4, GPIO.LOW)
time.sleep(0.1)

# PWM
pA = GPIO.PWM(ENB, 100)
pB = GPIO.PWM(ENA, 100)
time.sleep(0.1)

# Set up directions for Robot motion
def forward():
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 1)

def stop():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)

#Define mobile robot variables and parameters
ppr = 34
tsample = 0.05
directionR = 1
directionL = 1
radius = 0.03365
length = 0.1565
wr = 0.0
wl = 0.0
vr = 0.0
vl = 0.0
VrefRight = 0.0
VrefLeft = 0.0
xc = 0.0
yc = 0.0
xNext = 0
yNext = 0
thetaNext = 0
distance = 0.0
theta = 0.0
V = 0.0
W = 0.0

#Define state feedback variables
errorX = 0.0
errorY = 0.0
errorTheta = 0.0
errorDistance = 0.0

Kdistance = 0.495 #Needs to be tuned properly
Ktheta = 0.55 #Needs to be tuned properly


#Define state feedback and innter loop time variable
tprev = 0.0
previousComputeTime = 0.0


#Define Encoder variables
stepsRightCurr = 0
stepsLeftCurr = 0
prevStepsRight = 0
prevStepsLeft = 0

# PID Right Motor
KP_R = 15.0
KI_R = 0.01
KD_R = 0.1
errorSpeedRightPrevious = 0.0
errorSpeedRightSum = 0.0
errorSpeedRightDerrivative = 0.0
PWMR = 0.0
PWMRprev = 0.0

# PID Left Motor
KP_L = 15.0
KI_L = 0.01
KD_L = 0.1
errorSpeedLeftPrevious = 0.0
errorSpeedLeftSum = 0.0
errorSpeedLeftDerrivative = 0.0
PWML = 0.0
PWMLprev = 0.0

#Physical Constraints
maxLinearVelocity = 0.45
maxAngularVelocity = np.pi / 3.5

#Goals
XGoal = [1,2]
YGoal = [1,2]

#Plot (for storing position history)
xPoints = [0]
yPoints = [0]

try:
    #Arm mobile robot motors
    forward()
    pA.start(100)
    pB.start(100)
    pA.ChangeDutyCycle(0)
    pB.ChangeDutyCycle(0)

    #Set start time
    timeNow = time.perf_counter()
    tprev = timeNow
    previousComputeTime = timeNow

    #Main Loop
    i = 0
    for i in range(len(XGoal)):
        while True:
            time.sleep(0.001) #or 0.01 (Give time for encoders to update)
            timeNow = time.perf_counter()
            stepsRightCurr = encoderRight.steps
            stepsLeftCurr = encoderLeft.steps
            if(timeNow - tprev > tsample):
                tprev = timeNow
                dt = timeNow - previousComputeTime
                previousComputeTime = timeNow
                
                #Get Reference signal Vref and Wref
                errorX = XGoal[i] - xc
                errorY = YGoal[i] - yc
                errorDistance = np.sqrt(errorX * errorX + errorY * errorY)
                errorTheta = np.arctan2(errorY, errorX) - theta
                Vref = errorDistance * Kdistance
                Wref = errorTheta * Ktheta

                #Clip reference value to phisical constraints                
                if np.absolute(Vref) > maxLinearVelocity:
                    Vref = maxLinearVelocity * np.sign(Vref)
                if np.absolute(Wref) > maxAngularVelocity:
                    Wref = maxAngularVelocity * np.sign(Wref)                
                
                #Calculate Speed for motors based on Vref and Wref
                VrefRight = ((2.0 * Vref + Wref * length) / 2.0)
                VrefLeft = ((2.0 * Vref - Wref * length) / 2.0)
                
                #Get current speed of wheels in RPM based on encoder ticks change  
                changeRightTick = stepsRightCurr - prevStepsRight
                changeLeftTick = stepsLeftCurr - prevStepsLeft
                prevStepsRight = stepsRightCurr
                prevStepsLeft = stepsLeftCurr
                RPMRight = (changeRightTick / ppr) / (dt / 60)
                RPMLeft = (changeLeftTick / ppr) / (dt / 60)
                
                RPMLeft = -RPMLeft

                #Convert RPM to angular velocity of wheels                
                wr = (np.pi / 30) * RPMRight
                wl = (np.pi / 30) * RPMLeft
                
                #Estimate wheels Liniear (Tangetal) velocity based on angular velocity
                vr = wr * radius
                vl = wl * radius
                
                # Speed Error (For PID)
                errorSpeedRight = VrefRight - vr
                errorSpeedLeft = VrefLeft - vl
                
                # Integral part (For PID)
                errorSpeedRightSum += errorSpeedRight * dt
                errorSpeedLeftSum += errorSpeedLeft * dt
                
                # Derivative part (For PID)
                errorSpeedRightDerrivative = (errorSpeedRight - errorSpeedRightPrevious) / dt
                errorSpeedLeftDerrivative = (errorSpeedLeft - errorSpeedLeftPrevious) / dt
                
                uR = 0
                uL = 0
                # Compute Control Signal (PID)
                if(VrefRight != 0.0):
                    uR = KP_R * errorSpeedRight + KI_R * errorSpeedRightSum + KD_R * errorSpeedRightDerrivative
                if(VrefLeft != 0.0):
                    uL = KP_L * errorSpeedLeft + KI_L * errorSpeedLeftSum + KD_L * errorSpeedLeftDerrivative
                
                # Motor power
                PWMR = PWMRprev + uR
                PWML = PWMLprev + uL

                #Clip PWM value
                if PWMR > 100.0:
                    PWMR = 100.0

                if PWML > 100.0:
                    PWML = 100.0
                    
                if PWMR < 0.0:
                    PWMR = 0.0

                if PWML < 0.0:
                    PWML = 0.0
                    
                PWMRprev = PWMR
                PWMLprev = PWML

                #Apply PWM signals
                pA.ChangeDutyCycle(PWMR)
                pB.ChangeDutyCycle(PWML)
                
                #
                errorSpeedRightPrevious = errorSpeedRight
                errorSpeedLeftPrevious = errorSpeedLeft
                
                V = (vr + vl) / 2.0
                W = (vr - vl) / length
                
                print("X Current=%.3f" %xc, "\tY Current=%.3f" %yc, "\tV=%.3f" %V, "\tW=%.3f" %W)

                # Localisation (Use 4th order Range-Kutta numerical integrator (Final Form) to estimate position)
                xNext = (xc + dt * V * (2.0 + np.cos(dt * W / 2.0)) * np.cos(theta + dt * W / 2.0) / 3.0)
                yNext = (yc + dt * V * (2.0 + np.cos(dt * W / 2.0)) * np.sin(theta + dt * W / 2.0) / 3.0)
                thetaNext = theta + (dt * W)

                #Update Position
                xc = xNext
                yc = yNext
                theta = thetaNext
                
                #Store current position in position history
                xPoints.append(xc)
                yPoints.append(yc)
                
                #Set Goal Terminate value (Acceptable distance error)
                if(errorDistance < 0.08):
                    pA.ChangeDutyCycle(0)
                    pB.ChangeDutyCycle(0)
                    print("Goal Reached!")
                    break
            
except KeyboardInterrupt:
    stop()
    time.sleep(0.1)
    print('Command Interrpted.')
    encoderRight.close()
    time.sleep(0.1)
    encoderLeft.close()
    time.sleep(0.1)
    GPIO.cleanup()
    time.sleep(0.1)
    
finally:
    xPoints = np.array(xPoints)
    yPoints = np.array(yPoints)
    plt.scatter(xPoints,yPoints)
    plt.scatter(XGoal,YGoal, marker='x', color='red')
    plt.ylim([-0.5,3])
    plt.xlim([-0.5,3])
    plt.show()
    stop()
    time.sleep(0.1)
    print('Done.')
    encoderRight.close()
    time.sleep(0.1)
    encoderLeft.close()
    time.sleep(0.1)
    GPIO.cleanup()
    time.sleep(0.1)