"""
The Reelfoot Rover controls four motors:
    FL - Front Left
    FR - Front Right
    RL - Rear Left
    RR - Rear Right
These motors have an accompanying speed and direction setting.
"""
import lgpio
import pins as p

# Constants
FORWARD = (1, 0)
REVERSE = (0, 1)
BRAKE = (1, 1)
PWM_FREQUENCY = 100

class Motor:
    """
    Basic motor control via PWM on the dual H-Bridge.
    """
    def __init__(self, handle, ml1, ml2, sl):
        """
        Initialize the motor.

        Parameters:
          - handle: The lgpio handle used to control the motor.
          - ml1: Motor logic line 1 of the H-Bridge.
          - ml2: Motor logic line 2 of the H-bridge.
          - sl:  The Speed / Enable line of the H-Bridge.
        """

        # initialize class fields
        self.__handle = handle
        self.__ml1 = ml1
        self.__ml2 = ml2
        self.__sl = sl

        # claim output pins for the motor
        lgpio.gpio_claim_output(handle, ml1)
        lgpio.gpio_claim_output(handle, ml2)
        lgpio.gpio_claim_output(handle, sl)

        # initially, we are stopped
        self.set_direction(BRAKE)
        self.set_speed(0)

    def set_direction(self, direction):
        """
        Set the direction of the motor to the given direction tuple for the ml1
        and ml2 pins.
        """
        self.__direction = direction
        lgpio.gpio_write(self.__handle, self.__ml1, self.__direction[0])
        lgpio.gpio_write(self.__handle, self.__ml2, self.__direction[1])

    def set_speed(self, speed):
        """
        Set the speed of the motor to the given speed (0-100).
        """
        self.__speed = speed
        lgpio.tx_pwm(self.__handle, self.__sl, PWM_FREQUENCY, speed)

    def get_direction(self):
        return self.__direction
    
    def get_speed(self):
        return self.__speed

# define the motors
handle = lgpio.gpiochip_open(0)
FL=Motor(handle, p.FL_ML1, p.FL_ML2, p.FL_Speed)
FR=Motor(handle, p.FR_ML1, p.FR_ML2, p.FR_Speed)
RL=Motor(handle, p.RL_ML1, p.RL_ML2, p.RL_Speed)
RR=Motor(handle, p.RR_ML1, p.RR_ML2, p.RR_Speed)

