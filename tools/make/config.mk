PLATFORM=CF2
DEBUG=1

## Automatically reboot this address to bootloader before flashing [radio://0/80/2M/E7E7E7E7XX]
CLOAD_CMDS = -w radio://0/80/2M/E7E7E7E701 

# W/ make cload - Connect to CF w/ address, put in bootloader mode and flash the binary 


# ESTIMATOR=kalman # sets the default estimator at runtime (complementary,kalman)
# CONTROLLER=GTC # Sets the default controller (Mellinger,PID,GTC) at runtime and can't be overwritten 


## Weight of the Crazyflie, including decks. The default setting is a Crazyflie 2.X without decks.
# CFLAGS += -DCF_MASS=0.027f // in kg

## Use morse when flashing the LED to indicate that the Crazyflie is calibrated
# CFLAGS += -DCALIBRATED_LED_MORSE