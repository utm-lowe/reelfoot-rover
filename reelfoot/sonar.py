import pins as p
import lgpio
import time

# constants
SPEED_OF_SOUND = 343  
TRIGGER = (p.SO1_Trig, p.SO2_Trig, p.SO3_Trig)

# time measuring variables
complete = False
start = 0
end = 0

def echo_callback(chip, gpio, level, tick):
    """
    Call back for the GPIO pin alert. This is invoked by the lgpio library on
    rising and falling edges.
    """
    global start, end, complete

    if level == 1:
        start = tick
    else:
        end = tick
        complete = True

def ping(idx):
    """
    Ping the sonar indicated by idx (0, 1, or 2). 
    Returns the distance to the nearest object in meters or -1 on timeoutl.
    """
    global start, end, complete

    # reset the counter
    complete = False
    start = 0
    end = 0

    # Pulse the trigger pin
    lgpio.gpio_write(handle, TRIGGER[idx], 0)
    time.sleep(5e-6)
    lgpio.gpio_write(handle, TRIGGER[idx], 1)
    time.sleep(10e-6)
    lgpio.gpio_write(handle, TRIGGER[idx], 0)

    # wait for a response
    counter = 0
    while not complete:
        time.sleep(0.01)
        counter = counter + 1
        if counter >= 3:
            return -1

    # calculate distance
    elapsed = (end - start) / 1e9
    return elapsed * SPEED_OF_SOUND / 2

def scan_ring():
    """
    Scan the entire ring and return a list of results.
    """
    result = []
    for i in range(len(TRIGGER)):
        result.append(ping(i))

    return result

# initialize the sonar ring
handle = lgpio.gpiochip_open(0)
for t in TRIGGER:
    lgpio.gpio_claim_output(handle, t, 0)
    lgpio.gpio_write(handle, t, 0)
time.sleep(0.2)
lgpio.gpio_claim_alert(handle, p.SO_Echo, lgpio.BOTH_EDGES, lgpio.SET_PULL_NONE)
time.sleep(0.1)
cb = lgpio.callback(handle, p.SO_Echo, lgpio.BOTH_EDGES, echo_callback)
