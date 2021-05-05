/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include "controller.h"

static bool motorSetEnable = false;
static bool safeModeEnable = true;

static float MS1,MS2,MS3,MS4; // Motorspeeds [rad/s]

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL) // Limit PWM value to UINT16_MAX = 65,535

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  

  if (getControllerType() == ControllerTypeGTC)
  {
    // Note: control struct has int16_t datatype with PWM limit of 32,767 for roll, pitch, and yaw. 
    // So values are compressed in half in GTC and re-expanded here to preserve values greater than 32,767
    int32_t r = (control->roll)*2; 
    int32_t p = (control->pitch)*2;
    int32_t y = (control->yaw)*2;

    motorPower.m1 = limitThrust(control->thrust - r - p - y); // Add respective thrust components and limit to (0 <= PWM <= 65,535)
    motorPower.m2 = limitThrust(control->thrust - r + p + y);
    motorPower.m3 = limitThrust(control->thrust + r + p - y);
    motorPower.m4 = limitThrust(control->thrust + r - p + y);
    
  }
  else
  {
    // Default 'X' Configuration
    int16_t r = control->roll / 2.0f; // Divide roll thrust between each motor
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw); // Add respective thrust components
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  }



  

  if (motorSetEnable)
  { // This maps thrust to voltage, compensates for battery voltage, then converts to PWM  
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  { // Cap motor force low end to idleThrust
    if (motorPower.m1 < idleThrust) {
      motorPower.m1 = idleThrust;
    }
    if (motorPower.m2 < idleThrust) {
      motorPower.m2 = idleThrust;
    }
    if (motorPower.m3 < idleThrust) {
      motorPower.m3 = idleThrust;
    }
    if (motorPower.m4 < idleThrust) {
      motorPower.m4 = idleThrust;
    }

    if(safeModeEnable)
    {
      motorsSetRatio(MOTOR_M1, 0);
      motorsSetRatio(MOTOR_M2, 0);
      motorsSetRatio(MOTOR_M3, 0);
      motorsSetRatio(MOTOR_M4, 0);
    }
    else{
      motorsSetRatio(MOTOR_M1, motorPower.m1);
      motorsSetRatio(MOTOR_M2, motorPower.m2);
      motorsSetRatio(MOTOR_M3, motorPower.m3);
      motorsSetRatio(MOTOR_M4, motorPower.m4);
    }

    // Convert PWM to motor speeds (Forster: Eq. 3.4b)
    MS1 = 0.04077*motorPower.m1 + 380.836;
    MS2 = 0.04077*motorPower.m2 + 380.836;
    MS3 = 0.04077*motorPower.m3 + 380.836;
    MS4 = 0.04077*motorPower.m4 + 380.836;

    
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_ADD(PARAM_UINT8, SafeMode, &safeModeEnable)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_ADD(LOG_FLOAT, MS1, &MS1)
LOG_ADD(LOG_FLOAT, MS2, &MS2)
LOG_ADD(LOG_FLOAT, MS3, &MS3)
LOG_ADD(LOG_FLOAT, MS4, &MS4)
LOG_GROUP_STOP(motor)
