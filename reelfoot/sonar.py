import pins as p
import lgpio
import time
import threading

# constants
SPEED_OF_SOUND = 343  
TRIGGER = (p.SO1_Trig, p.SO2_Trig, p.SO3_Trig)

# time measuring variables
done = threading.Event()
start = None
end = None
timeout = 0.30

def echo_callback(chip, gpio, level, tick):
    """
    Call back for the GPIO pin alert. This is invoked by the lgpio library on
    rising and falling edges.
    """
    global start, end
    if level == 1:
        start = tick
    elif start is not None:      # ignore orphan falling edges
        end = tick
        done.set()

def ping(idx):
    """
    Ping the sonar indicated by idx (0, 1, or 2). 
    Returns the distance to the nearest object in meters or -1 on timeout.
    """    
    global start, end

    # don't fire while a previous echo is still holding the OR'd line high
    deadline = time.monotonic() + 0.25
    while lgpio.gpio_read(handle, p.SO_Echo) and time.monotonic() < deadline:
        time.sleep(0.001)
    start = end = None
    done.clear()

    # fire and wait
    lgpio.tx_pulse(handle, TRIGGER[idx], 10, 10, 0, 1)
    if not done.wait(timeout):
        return -1
    return (end - start) / 1e9 * SPEED_OF_SOUND / 2


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
