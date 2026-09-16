# Reelfoot Rover

The Reelfoot Rover is a robot which is used in the CSCI 458 - Autonomous Mobile 
Robotics at the University of Tennessee at Matin. This robot is a 4wd robot 
with each wheel being independantly controlled via PWM. The robot is controlled
via a Raspberry Pi computer, and power for the motors and all peripherals is drawn
from the power rails of the Raspberr Pi.

This repository contains the circuit board which controls the following:
- 4 Independentally Controlled Motors
- 4 Optical Wheel Encoders
- 3 HC-SR04 Ultrasonic Range Finder (Sonars)

The board is designed to be used along with the Raspberry Pi Sense Hat. You don't
need the sense hat, but I rather like it. So I made sure to not use any of the pins 
used by the sense hat. You can stack this board under the sense hat, or connect it
with a ribbon cable.

This board is licensed under the MIT License and provided in the hope that you'll
find it helpful and use it in your own robotics tinkering.