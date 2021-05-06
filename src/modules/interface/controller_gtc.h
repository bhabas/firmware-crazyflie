/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 *
 */

#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#include "stabilizer_types.h"

void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void GTC_Command(setpoint_t *setpoint);

static inline int32_t thrust2PWM(float f) 
{
    // Conversion values derived from J. Forster
    float a = 2.108e-11;
    float b = 1.06e-6;

    float s = 1; // sign of value
    int32_t f_pwm = 0;

    s = f/fabsf(f);
    f = fabsf(f);
    
    f_pwm = s*(sqrtf(4*a*f+b*b)/(2*a) - b/(2*a));

    return f_pwm;

}        

#endif //__CONTROLLER_GTC_H__

