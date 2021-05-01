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
#include "controller_gtc.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "physicalConstants.h"

// SYSTEM PARAMETERS
static float m = CF_MASS;
static float g = GRAVITY_MAGNITUDE;
struct mat33 J; // Rotational Inertia Matrix [kg*m^2]

static float d = 0.040f;    // COM to Prop [m]
static float dp = 0.028284; // COM to Prop along x-axis [m]
                            // [dp = d*sin(45 deg)]

static float const kf = 2.2e-8f;    // Thrust Coeff [N/(rad/s)^2]
static float const c_tf = 0.0061f;  // Moment Coeff [Nm/N]


// LOGGING VARIABLES
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

// STATE ERRORS
struct vec e_x;     // Pos-error [m]
struct vec e_v;     // Vel-error [m/s]
struct vec e_R;     // Rotation-error [rad]
struct vec e_omega; // Omega-error [rad/s]

// STATE VALUES
struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

struct mat33 R; // Orientation as rotation matrix
struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]

// DESIRED STATES
struct vec x_d = {0.0f,0.0f,1.0f}; // Pos-desired [m]
struct vec v_d = {0.0f,0.0f,0.0f}; // Vel-desired [m/s]
struct vec a_d = {0.0f,0.0f,0.0f}; // Acc-desired [m/s^2]

struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

struct vec b1_d = {1.0f,0.0f,0.0f};    // Desired body x-axis in global coord.
struct vec b2_d;    // Desired body y-axis in global coord.
struct vec b3_d;    // Desired body z-axis in global coord.
struct vec b3;      // Current body z-axis in global coord.

struct mat33 R_d;   // Desired rotational matrix from b_d vectors

struct vec e_3 = {0.0f, 0.0f, 1.0f};    // Global z-axis

struct vec F_thrust_ideal;
struct vec M;
float F_thrust; // Thrust motor thrust [N]

// TEMPORARY CALC VECS/MATRICES
struct vec temp1_v; 
struct vec temp2_v;
struct vec temp3_v;
struct vec temp4_v;
struct vec temp5_v;


struct mat33 RdT_R; // Rd' * R
struct mat33 RT_Rd; // R' * Rd
struct mat33 temp1_m;  


// CONTROLLER GAINS
static float kp_x = 0.7f;   // Pos. Proportional Gain
static float kd_x = 0.25f;  // Pos. Derivative Gain
static float ki_x = 0.0f;   // Pos. Integral Gain

static float kp_R = 0.05f;  // Rot. Proportional Gain
static float kd_R = 0.008f;  // Rot. Derivative Gain
static float ki_R = 0.0f;   // Rot. Integral Gain



void controllerGTCInit(void)
{
    controllerGTCTest();
    controllerGTCReset();
    DEBUG_PRINT("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    DEBUG_PRINT("GTC Reset\n");
    // Reset errors to zero
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
    // SYSTEM PARAMETERS 
    J = mdiag(1.65717e-5f, 1.66556e-5f, 2.92617e-5f); // Rotational Inertia of CF [kg m^2]

    // // =========== State Definitions =========== //
    // statePos = mkvec(state->position.x*0, state->position.y*0, state->position.z);                      // [m]
    // stateVel = mkvec(state->velocity.x*0, state->velocity.y*0, state->velocity.z);                      // [m]
    // stateOmega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]
    // stateQuat = mkquat(state->attitudeQuaternion.x,
    //                    state->attitudeQuaternion.y,
    //                    state->attitudeQuaternion.z,
    //                    state->attitudeQuaternion.w);

    
    // // EULER ANGLES EXPRESSED IN YZX NOTATION
    // stateEul = quat2eul(stateQuat);
    // stateEul.x = degrees(stateEul.x);
    // stateEul.y = degrees(stateEul.y);
    // stateEul.z = degrees(stateEul.z);

    // // =========== State Setpoints =========== //
    // x_d = mkvec(setpoint->position.x*0, setpoint->position.y*0, setpoint->position.z);             // Pos-desired [m]
    // v_d = mkvec(setpoint->velocity.x*0, setpoint->velocity.y*0, setpoint->velocity.z);             // Vel-desired [m/s]
    // a_d = mkvec(setpoint->acceleration.x*0, setpoint->acceleration.y*0, setpoint->acceleration.z*0); // Acc-desired [m/s^2]

    // omega_d = mkvec(radians(setpoint->attitudeRate.roll)*0,
    //                 radians(setpoint->attitudeRate.pitch)*0,
    //                 radians(setpoint->attitudeRate.yaw)*0);         // Omega-desired [rad/s]
    // domega_d = mkvec(radians(0.0f), radians(0.0f), radians(0.0f));  // Omega-Accl. [rad/s^2]





    // =========== Rotation Matrix =========== //
    // R changes Body axes to be in terms of Global axes
    // https://www.andre-gaschler.com/rotationconverter/
    R = quat2rotmat(stateQuat); // Quaternion to Rotation Matrix Conversion
    b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 



    // =========== Translational Errors & Desired Body-Fixed Axes =========== //
    e_x = vsub(statePos, x_d); // [e_x = pos-x_d]
    e_v = vsub(stateVel, v_d); // [e_v = vel-v_d]

    /* [F_thrust_ideal = -kp_x*e_x + -kd_x*e_v + m*g*e_3 + m*a_d] */
    temp1_v = vscl(-kp_x, e_x);
    temp2_v = vscl(-kd_x, e_v);
    temp3_v = vscl(m*g, e_3);
    temp4_v = vscl(m, a_d);
    F_thrust_ideal = vadd4(temp1_v, temp2_v, temp3_v, temp4_v); 


    // =========== Rotational Errors =========== // 
    b3_d = vnormalize(F_thrust_ideal);
    b2_d = vnormalize(vcross(b3_d, b1_d)); // [b3_d x b1_d] | body-fixed horizontal axis
    temp1_v = vnormalize(vcross(b2_d, b3_d));
    R_d = mcolumns(temp1_v, b2_d, b3_d); // Desired rotation matrix from calculations

    RdT_R = mmul(mtranspose(R_d), R);    // [R_d'*R]
    RT_Rd = mmul(mtranspose(R), R_d);    // [R'*R_d]

    temp1_v = dehat(msub(RdT_R, RT_Rd)); // [dehat(R_d'*R - R'*R)]
    e_R = vscl(0.5f, temp1_v);           // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

    temp1_v = mvmul(RT_Rd, omega_d);     // [R.transpose()*R_d*omega_d]
    e_omega = vsub(stateOmega, temp1_v); // Ang. vel error | [e_omega = omega - R.transpose()*R_d*omega_d] 


    // =========== Control Equations =========== // 
    /* [M = -kp_R*e_R - kd_R*e_omega + Gyro_dyn] */

    temp1_v = vscl(-kp_R, e_R);                         // [-kp_R*e_R]
    temp2_v = vscl(-kd_R, e_omega);                     // [-kd_R*e_omega]
    temp3_v = vcross(stateOmega, mvmul(J, stateOmega)); // [omega x J*omega]

    /* [Gyro_dyn = omega.cross(J*omega) - J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d)] */
    temp1_m = mmul(hat(stateOmega), RT_Rd);
    temp4_v = mvmul(temp1_m, omega_d); // [hat(omega)*R.transpose()*R_d*omega_d]
    temp5_v = mvmul(RT_Rd, domega_d);
    temp4_v = mvmul(J, vsub(temp4_v, temp5_v));
    temp4_v = vneg(temp4_v); // -J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d)


    // // =========== Thrust and Moments [Force Notation] =========== // 
    F_thrust = vdot(F_thrust_ideal, b3);           // Project ideal thrust onto b3 vector [N]
    M = vadd4(temp1_v, temp2_v, temp3_v, temp4_v); // Control moments [Nm]

    if (tick%200 == 0)
    {   consolePrintf("======== Debug Start ======\n");
        consolePrintf("\n==== STATES ====\n");
        consolePrintf("Pos: ");
        printvec(statePos);
        consolePrintf("Vel: ");
        printvec(stateVel);
        consolePrintf("R: \n");
        printmat(R);
        consolePrintf("Eul: ");
        printvec(stateEul);
        consolePrintf("Omega: ");
        printvec(stateOmega);

        consolePrintf("\n==== SETPOINTS ====\n");
        consolePrintf("x_d: ");
        printvec(x_d);
        consolePrintf("v_d: ");
        printvec(v_d);
        consolePrintf("omega_d: ");
        printvec(omega_d);
        consolePrintf("R_d: \n");
        printmat(R_d);
        
        consolePrintf("\n==== TRANSLATION/THRUST ====\n");

        consolePrintf("F_thrust_ideal: ");
        printvec(F_thrust_ideal);

        consolePrintf("\n==== DESIRED ROTATION ====\n");

        consolePrintf("b3_d: ");
        printvec(b3_d);
        consolePrintf("b2_d: ");
        printvec(b2_d);
        consolePrintf("b1_d': ");
        printvec(temp1_v);

        consolePrintf("R_d: \n");
        printmat(R_d);

        consolePrintf("\n==== ROTATIONAL ERRORS ====\n");
        consolePrintf("e_R: ");
        printvec(e_R);
        consolePrintf("e_omega: ");
        printvec(e_omega);


        consolePrintf("\n==== CONTROL EQ. ====\n");
        consolePrintf("Thrust: %0.4f\n",F_thrust);
        consolePrintf("Moments: \n");
        printvec(M);

        // consolePrintf("f_thrust: %.3f\n",f_thrust_pwm);
        // consolePrintf("f_roll: %.3f\n",f_roll_pwm);
        // consolePrintf("f_pitch: %.3f\n",f_pitch_pwm);
        // consolePrintf("f_yaw: %.3f\n",f_yaw_pwm);

        // consolePrintf("MS1: %.3f \tMS2: %.3f \tMS3: %.3f \tMS4: %.3f",MS1,MS2,MS3,MS4);

        consolePrintf("\n");

    }


    
    

}


// PARAMETER GROUPS
PARAM_GROUP_START(GTC_Params)
PARAM_ADD(PARAM_FLOAT, X_kp, &kp_x)
PARAM_ADD(PARAM_FLOAT, X_kd, &kd_x)
PARAM_ADD(PARAM_FLOAT, X_ki, &ki_x) 
PARAM_ADD(PARAM_FLOAT, R_kp, &kp_R)
PARAM_ADD(PARAM_FLOAT, R_kd, &kd_R)
PARAM_ADD(PARAM_FLOAT, R_ki, &ki_R)
PARAM_GROUP_STOP(GTC_Params)

PARAM_GROUP_START(GTC_States)
PARAM_ADD(PARAM_FLOAT, Pos_X, &statePos.x)
PARAM_ADD(PARAM_FLOAT, Pos_Y, &statePos.y)
PARAM_ADD(PARAM_FLOAT, Pos_Z, &statePos.z)

PARAM_ADD(PARAM_FLOAT, Vel_X, &stateVel.x)
PARAM_ADD(PARAM_FLOAT, Vel_Y, &stateVel.y)
PARAM_ADD(PARAM_FLOAT, Vel_Z, &stateVel.z)

PARAM_ADD(PARAM_FLOAT, Quat_x, &stateQuat.x)
PARAM_ADD(PARAM_FLOAT, Quat_y, &stateQuat.y)
PARAM_ADD(PARAM_FLOAT, Quat_z, &stateQuat.z)
PARAM_ADD(PARAM_FLOAT, Quat_w, &stateQuat.w)

PARAM_ADD(PARAM_FLOAT, Omega_X, &stateOmega.x)
PARAM_ADD(PARAM_FLOAT, Omega_Y, &stateOmega.y)
PARAM_ADD(PARAM_FLOAT, Omega_Z, &stateOmega.z)


PARAM_GROUP_STOP(GTC_States)

PARAM_GROUP_START(GTC_Setpoints)
PARAM_ADD(PARAM_FLOAT, Pos_X, &x_d.x)
PARAM_ADD(PARAM_FLOAT, Pos_Y, &x_d.y)
PARAM_ADD(PARAM_FLOAT, Pos_Z, &x_d.z)

PARAM_ADD(PARAM_FLOAT, Vel_X, &v_d.x)
PARAM_ADD(PARAM_FLOAT, Vel_Y, &v_d.y)
PARAM_ADD(PARAM_FLOAT, Vel_Z, &v_d.z)

PARAM_ADD(PARAM_FLOAT, Omega_X, &omega_d.x)
PARAM_ADD(PARAM_FLOAT, Omega_Y, &omega_d.y)
PARAM_ADD(PARAM_FLOAT, Omega_Z, &omega_d.z)

// PARAM_ADD(PARAM_FLOAT, Roll, &eul_d.x)
// PARAM_ADD(PARAM_FLOAT, Pitch, &eul_d.y)
// PARAM_ADD(PARAM_FLOAT, Yaw, &eul_d.z)
PARAM_GROUP_STOP(GTC_Setpoints)


// LOGGING GROUPS

LOG_GROUP_START(GTC_State_Est)
LOG_ADD(LOG_FLOAT, Pos_X, &statePos.x)
LOG_ADD(LOG_FLOAT, Pos_Y, &statePos.y)
LOG_ADD(LOG_FLOAT, Pos_Z, &statePos.z)

LOG_ADD(LOG_FLOAT, Vel_X, &stateVel.x)
LOG_ADD(LOG_FLOAT, Vel_Y, &stateVel.y)
LOG_ADD(LOG_FLOAT, Vel_Z, &stateVel.z)

LOG_ADD(LOG_FLOAT, Att_X, &stateEul.x)
LOG_ADD(LOG_FLOAT, Att_Y, &stateEul.y)
LOG_ADD(LOG_FLOAT, Att_Z, &stateEul.z)

LOG_ADD(LOG_FLOAT, Omega_X, &stateOmega.x)
LOG_ADD(LOG_FLOAT, Omega_Y, &stateOmega.y)
LOG_ADD(LOG_FLOAT, Omega_Z, &stateOmega.z)
LOG_GROUP_STOP(GTC_State_Est)

LOG_GROUP_START(GTC_Setpoints)
LOG_ADD(LOG_FLOAT, Pos_X, &x_d.x)
LOG_ADD(LOG_FLOAT, Pos_Y, &x_d.y)
LOG_ADD(LOG_FLOAT, Pos_Z, &x_d.z)

LOG_ADD(LOG_FLOAT, Vel_X, &v_d.x)
LOG_ADD(LOG_FLOAT, Vel_Y, &v_d.y)
LOG_ADD(LOG_FLOAT, Vel_Z, &v_d.z)

LOG_ADD(LOG_FLOAT, Omega_X, &omega_d.x)
LOG_ADD(LOG_FLOAT, Omega_Y, &omega_d.y)
LOG_ADD(LOG_FLOAT, Omega_Z, &omega_d.z)

// LOG_ADD(LOG_FLOAT, Roll, &eul_d.x)
// LOG_ADD(LOG_FLOAT, Pitch, &eul_d.y)
// LOG_ADD(LOG_FLOAT, Yaw, &eul_d.z)
LOG_GROUP_STOP(GTC_Setpoints)




LOG_GROUP_START(F_M)
LOG_ADD(LOG_FLOAT, F_thrust1, &F_thrust)
LOG_ADD(LOG_FLOAT, M_roll, &M.x)
LOG_ADD(LOG_FLOAT, M_pitch, &M.y)
LOG_ADD(LOG_FLOAT, M_yaw, &M.z)
LOG_GROUP_STOP(F_M)