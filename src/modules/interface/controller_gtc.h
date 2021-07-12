/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 *
 */

#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#include "stabilizer_types.h"
#include "physicalConstants.h"

// STANDARD LIBRARIES
#include <math.h>
#include <stdio.h>
#include <stdint.h>

// CF LIBARARIES
#include "math3d.h"
#include "log.h"
#include "param.h"
#include "debug.h"

// CF HEADERS
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "physicalConstants.h"


// FUNCTION PRIMITIVES
void controllerGTCInit(void);
bool controllerGTCTest(void);
void controllerGTCReset(void);
void controllerGTCTraj(void);
void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void GTC_Command(setpoint_t *setpoint);


// SYSTEM PARAMETERS
static float m = 0.037; // [g]
static float g = GRAVITY_MAGNITUDE;
struct mat33 J; // Rotational Inertia Matrix [kg*m^2]
static float h_ceiling = 2.50f; // [m]

static float d = 0.040f;    // COM to Prop [m]
static float dp = 0.028284; // COM to Prop along x-axis [m]
                            // [dp = d*sin(45 deg)]

static float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
static float const c_tf = 0.00618f;  // Moment Coeff [Nm/N]


// LOGGING VARIABLES
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

// STATE ERRORS
static struct vec e_x;  // Pos-error [m]
static struct vec e_v;  // Vel-error [m/s]
static struct vec e_PI;  // Pos. Integral-error [m*s]

static struct vec e_R;  // Rotation-error [rad]
static struct vec e_w;  // Omega-error [rad/s]
static struct vec e_RI; // Rot. Integral-error [rad*s]

// STATE VALUES
static struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
static struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
static struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
static struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

static struct mat33 R; // Orientation as rotation matrix
static struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]

// DESIRED STATES
static struct vec x_d = {0.0f,0.0f,0.2f}; // Pos-desired [m]
static struct vec v_d = {0.0f,0.0f,0.0f}; // Vel-desired [m/s]
static struct vec a_d = {0.0f,0.0f,0.0f}; // Acc-desired [m/s^2]

static struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
static struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
static struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
static struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

static struct vec b1_d = {1.0f,0.0f,0.0f};    // Desired body x-axis in global coord. [x,y,z]
static struct vec b2_d;    // Desired body y-axis in global coord.
static struct vec b3_d;    // Desired body z-axis in global coord.
static struct vec b3;      // Current body z-axis in global coord.

static struct mat33 R_d;   // Desired rotational matrix from b_d vectors





static struct vec e_3 = {0.0f, 0.0f, 1.0f}; // Global z-axis

static struct vec F_thrust_ideal;           // Ideal thrust vector
static float F_thrust = 0.0f;               // Desired body thrust [N]
static float F_thrust_max = 0.64f;          // Max possible body thrust [N}]
static struct vec M;                        // Desired body moments [Nm]
static struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*mm]
static float Moment_flag = false;


// TEMPORARY CALC VECS/MATRICES
static struct vec temp1_v; 
static struct vec temp2_v;
static struct vec temp3_v;
static struct vec temp4_v;
static struct mat33 temp1_m;  

static struct vec P_effort;
static struct vec R_effort;

static struct mat33 RdT_R; // Rd' * R
static struct mat33 RT_Rd; // R' * Rd
static struct vec Gyro_dyn;



// MOTOR THRUSTS
static float f_thrust; // Motor thrust - Thrust [N]
static float f_roll;   // Motor thrust - Roll   [N]
static float f_pitch;  // Motor thrust - Pitch  [N]
static float f_yaw;    // Motor thrust - Yaw    [N]

static int32_t f_thrust_pwm; 
static int32_t f_roll_pwm;   
static int32_t f_pitch_pwm; 
static int32_t f_yaw_pwm;  


// INIT CTRL GAIN VECTORS
static struct vec Kp_p; // Pos. Proportional Gains
static struct vec Kd_p; // Pos. Derivative Gains
static struct vec Ki_p; // Pos. Integral Gains  

static struct vec Kp_R; // Rot. Proportional Gains
static struct vec Kd_R; // Rot. Derivative Gains
static struct vec Ki_R; // Rot. Integral Gains

static float dt = (float)(1.0f/RATE_500_HZ);

// CONTROLLER PARAMETERS
static bool attCtrlEnable = false;
static bool tumbled = false;
static bool motorstop_flag = false;
static bool errorReset = false;

// OPTICAL FLOW STATES
static float RREV = 0.0f; // [rad/s]
static float OF_x = 0.0f; // [rad/s]
static float OF_y = 0.0f; // [rad/s] 
static bool flip_flag = false;


// POLICY VARIABLES
static float RREV_thr = 0.0f;
static float G1 = 0.0f;
static float G2 = 0.0f;

static bool policy_armed_flag = false;


// MOTOR VARIABLES
static uint32_t M1_pwm = 0; 
static uint32_t M2_pwm = 0; 
static uint32_t M3_pwm = 0; 
static uint32_t M4_pwm = 0; 

static float MS1 = 0;
static float MS2 = 0;
static float MS3 = 0;
static float MS4 = 0;

// TRAJECTORY VARIABLES
static float s_0 = 0.0f;
static float v = 0.0f;
static float a = 0.0f;
static float t = 0.0f;
static float T = 0.0f;
static uint8_t traj_type = 0;
static bool execute_traj = false;



static struct {
    
    uint32_t OF_xy; // [milli-rad/s]
    int16_t RREV;   // [milli-rad/s]

    uint32_t Mxy;   // [N*um]
    uint32_t FMz;   // [mN | N*um]

    uint32_t MS12; // [rad/s*0.01]
    uint32_t MS34;

} miscStatesZ_GTC;



static struct {
    
    uint32_t xy;  // Compressed position [mm]
    int16_t z;

    uint32_t vxy; // Compressed velocities [mm/s]
    int16_t vz;

    uint32_t axy; // Compress accelerations [mm/s^2]
    int16_t az;

} setpointZ_GTC;





// EXPLICIT FUNTIONS
static inline int32_t thrust2PWM(float f) 
{
    // Conversion values calculated from self motor analysis
    float a = 2.98e-4;
    float b = -9.84e-1;

    float s = 1; // sign of value
    int32_t f_pwm = 0;

    s = f/fabsf(f);
    f = fabsf(f)*1000.0f/9.81f; // Convert thrust to grams
    
    f_pwm = s*(f-b)/a;

    return f_pwm;

}        

static inline float PWM2thrust(int32_t M_PWM) // Converts thrust in PWM to thrust in Newtons
{
    // Conversion values from new motors
    float a = 2.98e-4;
    float b = -9.84e-1;

    float f = (a*M_PWM + b); // Convert thrust to grams

    f = f*9.81/1000; // Convert thrust from grams to Newtons

    return f;
}



static void compressGTCSetpoint(){
    setpointZ_GTC.xy = compressXY(x_d.x,x_d.y);
    setpointZ_GTC.z = x_d.z * 1000.0f;

    setpointZ_GTC.vxy = compressXY(v_d.x,v_d.y);
    setpointZ_GTC.vz = v_d.z * 1000.0f;

    setpointZ_GTC.axy = compressXY(a_d.x,a_d.y);
    setpointZ_GTC.az = a_d.z * 1000.0f;
}

static void compressMiscStates(){

    
    miscStatesZ_GTC.OF_xy = compressXY(OF_x,OF_y);              // [milli-rad/s]
    miscStatesZ_GTC.RREV = RREV * 1000.0f;                      // [milli-rad/s]

    miscStatesZ_GTC.Mxy = compressXY((float)f_roll_pwm/65535.0f,(float)f_pitch_pwm/65535.0f);  // [mN | N*um]
    miscStatesZ_GTC.FMz = compressXY((float)f_thrust_pwm/65535.0f,(float)f_yaw_pwm/65535.0f);

    miscStatesZ_GTC.MS12 = compressXY(MS1*0.01f,MS2*0.01f);     // [rad/s*0.01]
    miscStatesZ_GTC.MS34 = compressXY(MS3*0.01f,MS4*0.01f);

}





#endif //__CONTROLLER_GTC_H__

