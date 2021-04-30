#include "controller_gtc.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>

void controllerGTCInit(void)
{
    controllerGTCTest();
}

bool controllerGTCTest(void)
{
    return true;
}

void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

}