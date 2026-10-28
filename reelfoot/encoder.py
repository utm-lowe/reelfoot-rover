"""
The encoder module counts the number of ticks on each of the four encoders.

In order to determine speed, these numbers should be checked periodically and
divided by elapsed time.
"""
from . import pins as p
import lgpio
import time
import threading

def encoder_callback(chip, gpio, level, tick):
    """
    Callback for the GPIO pin alert. This is invoked by the lgpio library on
    rising edges.
    """
    global FL, FR, RL, RR, unknown

    if gpio == p.FL_Enc:
        FL += 1
    elif gpio == p.FR_Enc:
        FR += 1
    elif gpio == p.RL_Enc:
        RL += 1
    elif gpio == p.RR_Enc:
        RR += 1
    else:
        unknown+=1

# Encoder counts
FL = FR = RL = RR = 0
unknown=0

# encoder pin setup
callbacks = []
handle = lgpio.gpiochip_open(0)
for e in (p.FL_Enc, p.FR_Enc, p.RL_Enc, p.RR_Enc):
    time.sleep(0.2)
    lgpio.gpio_claim_alert(handle, e, lgpio.RISING_EDGE, lgpio.SET_PULL_NONE)
    time.sleep(0.1)
    callbacks.append(lgpio.callback(handle, e, lgpio.RISING_EDGE, encoder_callback))
FL_cb, FR_cb, RL_cb, RR_cb = callbacks 
