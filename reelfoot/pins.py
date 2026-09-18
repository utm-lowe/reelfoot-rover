"""
Reelfoot Rover pin definitions for interfacing with the GPIO pins.
All pins are given as their GPIO (Broadcom) number. The pins defined 
are:

    Encoder Pins
    ============
    FL_Enc - Front Left Encoder Input
    RL_Enc - Rear Left Encoder Input
    FR_Enc - Front Right Encoder Input
    RR_Enc - Rear Right Encoder Input

    Sonar Pins
    ==========
    SO1_Trig - Sonar 1 Trigger
    SO2_Trig - Sonar 2 Trigger
    SO3_Trig - Sonar 3 Trigger

    Motor Direction Pins
    ====================
    FL_ML1 - Front Left Motor Logic 1
    FL_ML2 - Front Left Motor Logic 2
    RL_ML1 - Rear Left Motor Logic 1
    RL_ML2 - Rear Left Motor Logic 2
    FR_ML1 - Front Right Motor Logic 1
    FR_ML2 - Front Right Motor Logic 2
    RR_ML1 - Rear Right Motor Logic 1
    RR_ML2 - Rear Right Motor Logic 2

    Motor Speed Pins (PWM)
    ======================
    FL_Speed - Front Left Speed
    RL_Speed - Rear Left Speed
    FR_Speed - Front Right Speed
    RR_Speed - Rear Right Speed
"""

# Encoder Inputs
FL_Enc=4
RL_Enc=17
FR_Enc=27
RR_Enc=22

# Sonar Pins
SO1_Trig = 14
SO2_Trig = 15
SO3_Trig = 18
SO_Echo = 7

# Motor Direction Pins
FL_ML1 = 10
FL_ML2 = 9
RL_ML1 = 11
RL_ML2 = 5
FR_ML1 = 6
FR_ML2 = 13
RR_ML1 = 19
RR_ML2 = 26

# Motor Speed (PWM) Pins
FL_Speed = 16
FR_Speed = 21
RL_Speed = 12
RR_Speed = 20
