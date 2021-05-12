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
static float m = 0.030; // [g]
static float g = GRAVITY_MAGNITUDE;
struct mat33 J; // Rotational Inertia Matrix [kg*m^2]

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
static struct vec x_d = {0.0f,0.0f,0.0f}; // Pos-desired [m]
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

static struct vec F_thrust_ideal;   // Ideal thrust vector
static float F_thrust = 0.0f;              // Desired body thrust [N]
static float F_thrust_max = 0.64f;          // Max possible body thrust [N}]
static struct vec M;                // Desired body moments [Nm]


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

// XY POSITION PID
static float P_kp_xy = 0.4f;
static float P_kd_xy = 0.4f;
static float P_ki_xy = 0.05f;
static float i_range_xy = 0.5f;

// Z POSITION PID
static float P_kp_z = 0.9f;
static float P_kd_z = 0.35f;
static float P_ki_z = 0.3f;
static float i_range_z = 0.25f;

// XY ATTITUDE PID
static float R_kp_xy = 0.001f;
static float R_kd_xy = 0.0005f;
static float R_ki_xy = 0.0f;
static float i_range_R_xy = 1.0f;

// Z ATTITUDE PID
static float R_kp_z = 3e-4f;
static float R_kd_z = 5e-5f;
static float R_ki_z = 3e-5f;
static float i_range_R_z = 0.05f;

// CTRL FLAGS
static struct vec P_kp_flag = {1.0f,1.0f,1.0f};
static struct vec P_kd_flag = {1.0f,1.0f,1.0f};
static float R_kp_flag = 1.0f;
static float R_kd_flag = 1.0f;


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
static float RREV = 0.0f; // [1/s]
static float OF_x = 0.0f; // [rad/s]
static float OF_y = 0.0f; // [rad/s] 
static bool flip_flag = false;

static float h_ceiling = 2.50f; // [m]

static uint32_t M1_pwm = 0; 
static uint32_t M2_pwm = 0; 
static uint32_t M3_pwm = 0; 
static uint32_t M4_pwm = 0; 

static float MS1 = 0;
static float MS2 = 0;
static float MS3 = 0;
static float MS4 = 0;

#define limitThrust(VAL) limitUint16(VAL) // Limit PWM value to UINT16_MAX = 65,535

static struct {
    
    uint32_t OF_xy; // [milli-rad/s]
    int16_t RREV;   // [milli-rad/s]

    uint32_t Mxy;   // [N*um]
    uint32_t FMz;   // [mN | N*um]

    uint32_t MS12; // [rad/s*0.01]
    uint32_t MS34;

} miscStatesZ_GTC;

static void compressMiscStates(){

    
    miscStatesZ_GTC.OF_xy = compressXY(OF_x,OF_y);              // [milli-rad/s]
    miscStatesZ_GTC.RREV = RREV * 1000.0f;                      // [milli-rad/s]

    miscStatesZ_GTC.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f);  // [mN | N*um]
    miscStatesZ_GTC.FMz = compressXY(F_thrust,M.z*1000.0f);

    miscStatesZ_GTC.MS12 = compressXY(MS1*0.01f,MS2*0.01f);     // [rad/s*0.01]
    miscStatesZ_GTC.MS34 = compressXY(MS3*0.01f,MS4*0.01f);

}


static struct {
    
    uint32_t xy;  // Compressed position [mm]
    int16_t z;

    uint32_t vxy; // Compressed velocities [mm/s]
    int16_t vz;

    uint32_t axy; // Compress accelerations [mm/s^2]
    int16_t az;

} setpointZ_GTC;


static void compressGTCSetpoint(){
    setpointZ_GTC.xy = compressXY(x_d.x,x_d.y);
    setpointZ_GTC.z = x_d.z * 1000.0f;

    setpointZ_GTC.vxy = compressXY(v_d.x,v_d.y);
    setpointZ_GTC.vz = v_d.z * 1000.0f;

    setpointZ_GTC.axy = compressXY(a_d.x,a_d.y);
    setpointZ_GTC.az = a_d.z * 1000.0f;
}




void controllerGTCInit(void)
{
    controllerGTCTest();
    controllerGTCReset();
    consolePrintf("GTC Initiated\n");
}

void controllerGTCReset(void)
{
    consolePrintf("GTC Reset\n");
    // Reset errors to zero
    e_PI = vzero();
    e_RI = vzero();
}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   
    switch(setpoint->cmd_type){
        case 10: // (Home/Reset)
  
            x_d = mkvec(0.0f,0.0f,0.0f);
            v_d = mkvec(0.0f,0.0f,0.0f);
            
            P_kp_flag = mkvec(1.0f,1.0f,1.0f); // Turn on all control flags
            P_kd_flag = mkvec(1.0f,1.0f,1.0f);


            R_kp_flag = 1.0f;
            R_kd_flag = 1.0f;

            break;

        case 1: // Position
            x_d.x = setpoint->cmd_val1;
            x_d.y = setpoint->cmd_val2;
            x_d.z = setpoint->cmd_val3;
            break;

        case 11: // Position Ctrl Flags
            P_kp_flag.x = setpoint->cmd_val1;
            P_kp_flag.y = setpoint->cmd_val2;
            P_kp_flag.z = setpoint->cmd_val3;

            break;

        case 2: // Velocity
            v_d.x = setpoint->cmd_val1;
            v_d.y = setpoint->cmd_val2;
            v_d.z = setpoint->cmd_val3;
            break;

        case 22: // Velocity Ctrl Flags
            P_kd_flag.x = setpoint->cmd_val1;
            P_kd_flag.y = setpoint->cmd_val2;
            P_kd_flag.z = setpoint->cmd_val3;
            break;

        case 3: // Attitude
            break;

        case 4: // Tumble-Detection

            break;

        case 5: // Hard Set All Motorspeeds to Zero
            motorstop_flag = true;
            break;

        case 7: // Execute Moment-Based Flip

            break;

        case 8: // Arm Policy Maneuver

            break;

    }
    
    return 0;
}


void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
    if (setpoint->GTC_cmd_rec == true)
        {
            
            GTC_Command(setpoint);
            setpoint->GTC_cmd_rec = false;
        }

    if (errorReset){
        controllerGTCReset();
        errorReset = false;
        }

    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        // SYSTEM PARAMETERS 
        J = mdiag(1.65717e-5f, 1.66556e-5f, 2.92617e-5f); // Rotational Inertia of CF [kg m^2]

        // CONTROL GAINS
        Kp_p = mkvec(P_kp_xy,P_kp_xy,P_kp_z);
        Kd_p = mkvec(P_kd_xy,P_kd_xy,P_kd_z);
        Ki_p = mkvec(P_ki_xy,P_ki_xy,P_ki_z);

        Kp_R = mkvec(R_kp_xy,R_kp_xy,R_kp_z);
        Kd_R = mkvec(R_kd_xy,R_kd_xy,R_kd_z);
        Ki_R = mkvec(R_ki_xy,R_ki_xy,R_ki_z);
        

        // =========== STATE DEFINITIONS =========== //
        statePos = mkvec(state->position.x, state->position.y, state->position.z);                      // [m]
        stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);                      // [m]
        stateOmega = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));   // [rad/s]
        stateQuat = mkquat(state->attitudeQuaternion.x,
                        state->attitudeQuaternion.y,
                        state->attitudeQuaternion.z,
                        state->attitudeQuaternion.w);

        RREV = stateVel.z/(h_ceiling - statePos.z);
        OF_x = stateVel.y/(h_ceiling - statePos.z);
        OF_y = stateVel.x/(h_ceiling - statePos.z);
        
        // EULER ANGLES EXPRESSED IN YZX NOTATION
        stateEul = quat2eul(stateQuat);
        stateEul.x = degrees(stateEul.x);
        stateEul.y = degrees(stateEul.y);
        stateEul.z = degrees(stateEul.z);

        // =========== STATE SETPOINTS =========== //
        // x_d = mkvec(setpoint->position.x,setpoint->position.y,setpoint->position.z);             // Pos-desired [m]
        // v_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);             // Vel-desired [m/s]
        // a_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z); // Acc-desired [m/s^2]

        omega_d = mkvec(radians(setpoint->attitudeRate.roll),
                        radians(setpoint->attitudeRate.pitch),
                        radians(setpoint->attitudeRate.yaw));         // Omega-desired [rad/s]
        domega_d = mkvec(radians(0.0f), radians(0.0f), radians(0.0f));  // Omega-Accl. [rad/s^2]

        eul_d = mkvec(radians(setpoint->attitude.roll),
                        -radians(setpoint->attitude.pitch), 
                        radians(setpoint->attitude.yaw));
        quat_d = rpy2quat(eul_d); // Desired orientation from eul angles [ZYX NOTATION]





        // =========== ROTATION MATRIX =========== //
        // R changes Body axes to be in terms of Global axes
        // https://www.andre-gaschler.com/rotationconverter/
        R = quat2rotmat(stateQuat); // Quaternion to Rotation Matrix Conversion
        b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 

        // TUMBLE DETECTION
        if (b3.z <= 0){
            tumbled = true;
        }

        // =========== TRANSLATIONAL ERRORS & DESIRED BODY-FIXED AXES =========== //
        e_x = vsub(statePos, x_d); // [e_x = pos-x_d]
        e_v = vsub(stateVel, v_d); // [e_v = vel-v_d]

        // POS. INTEGRAL ERROR
        e_PI.x += (e_x.x)*dt;
        e_PI.x = clamp(e_PI.x, -i_range_xy, i_range_xy);

        e_PI.y += (e_x.y)*dt;
        e_PI.y = clamp(e_PI.y, -i_range_xy, i_range_xy);

        e_PI.z += (e_x.z)*dt;
        e_PI.z = clamp(e_PI.z, -i_range_z, i_range_z);


        /* [F_thrust_ideal = -kp_x*e_x*(kp_x_flag) + -kd_x*e_v + -kI_x*e_PI*(kp_x_flag) + m*g*e_3 + m*a_d] */
        temp1_v = veltmul(P_kp_flag,veltmul(vneg(Kp_p), e_x));
        temp2_v = veltmul(P_kd_flag,veltmul(vneg(Kd_p), e_v));
        temp3_v = veltmul(P_kp_flag,veltmul(vneg(Ki_p), e_PI));
        P_effort = vadd3(temp1_v,temp2_v,temp3_v);

        temp1_v = vscl(m*g, e_3); // Feed-forward term
        temp2_v = vscl(m, a_d);

        F_thrust_ideal = vadd3(P_effort, temp1_v,temp2_v); 

        // =========== ROTATIONAL ERRORS =========== // 
        b3_d = vnormalize(F_thrust_ideal);
        b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
        temp1_v = vnormalize(vcross(b2_d, b3_d));
        R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations

        // ATTITUDE CONTROL
        if (attCtrlEnable){ 
            R_d = quat2rotmat(quat_d); // Desired rotation matrix from att. control
        }


        RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
        RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]

        temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
        e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

        temp1_v = mvmul(RT_Rd, omega_d);        // [R.transpose()*R_d*omega_d]
        e_w = vsub(stateOmega, temp1_v);        // Ang. vel error | [e_w = omega - R.transpose()*R_d*omega_d] 

        // ROT. INTEGRAL ERROR
        e_RI.x += (-e_R.x)*dt;
        e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

        e_RI.y += (-e_R.y)*dt;
        e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

        e_RI.z += (-e_R.z)*dt;
        e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

        // =========== CONTROL EQUATIONS =========== // 
        /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

        temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
        temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
        temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
        R_effort = vadd3(temp1_v,temp2_v,temp3_v);

        

        /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d )] */
        temp1_v = vcross(stateOmega, mvmul(J, stateOmega)); // [omega x J*omega]


        temp1_m = mmul(hat(stateOmega), RT_Rd); //  hat(omega)*R.transpose()*R_d
        temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R.transpose()*R_d)*omega_d
        temp3_v = mvmul(RT_Rd, domega_d);       // (R.transpose()*R_d*domega_d)

        temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R.transpose()*R_d*omega_d - R.transpose()*R_d*domega_d)
        Gyro_dyn = vsub(temp1_v,temp4_v);


        // =========== THRUST AND MOMENTS [FORCE NOTATION] =========== // 
        if(!tumbled){
            F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
            F_thrust = clamp(F_thrust,0.0f,F_thrust_max*0.8f);
            M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]
        }

        if(tumbled){
            // consolePrintf("System Tumbled: \n");
            F_thrust = 0.0f;
            M.x = 0.0f;
            M.y = 0.0f;
            M.z = 0.0f;
        }

        // =========== CONVERT THRUSTS AND MOMENTS TO PWM =========== // 
        f_thrust = F_thrust/4.0f;
        f_roll = M.x/(4.0f*dp);
        f_pitch = M.y/(4.0f*dp);
        f_yaw = M.z/(4.0*c_tf);

        f_thrust_pwm = thrust2PWM(f_thrust);
        f_roll_pwm = thrust2PWM(f_roll);
        f_pitch_pwm = thrust2PWM(f_pitch);
        f_yaw_pwm = thrust2PWM(f_yaw);


        // =========== INSERT PWM VALUES INTO CONTROL STRUCT =========== //
        if(motorstop_flag){
            control->thrust = 0.0f;
            control->roll = 0;
            control->pitch = 0;
            control->yaw = 0;
        }
        else{
            control->thrust = f_thrust_pwm;
            control->roll = f_roll_pwm/2;
            control->pitch = f_pitch_pwm/2;
            control->yaw = f_yaw_pwm/2;
        }

        

        M1_pwm = limitThrust(f_thrust_pwm - f_roll_pwm - f_pitch_pwm - f_yaw_pwm); // Add respective thrust components and limit to (0 <= PWM <= 65,535)
        M2_pwm = limitThrust(f_thrust_pwm - f_roll_pwm + f_pitch_pwm + f_yaw_pwm);
        M3_pwm = limitThrust(f_thrust_pwm + f_roll_pwm + f_pitch_pwm - f_yaw_pwm);
        M4_pwm = limitThrust(f_thrust_pwm + f_roll_pwm - f_pitch_pwm + f_yaw_pwm);

        // Convert PWM to motor speeds (Forster: Eq. 3.4b)
        MS1 = 0.04077f*M1_pwm + 380.836f;
        MS2 = 0.04077f*M2_pwm + 380.836f;
        MS3 = 0.04077f*M3_pwm + 380.836f;
        MS4 = 0.04077f*M4_pwm + 380.836f;

        
        compressGTCSetpoint();
        compressMiscStates();

        // if(tick%20 == 0){
        //     // DEBUG_PRINT("M_z: %.3f | eR.z: %.3f | eRI.z: %.3f \n",M.z*1000,e_R.z,e_RI.z);
        //     DEBUG_PRINT("MS1: %.3f| MS2: %.3f | MS3: %.3f | MS4: %.3f \n",MS1,MS2,MS3,MS4);
            
        // }

    }

}


// PARAMETER GROUPS
PARAM_GROUP_START(GTC_Params)
PARAM_ADD(PARAM_FLOAT, P_kp_xy, &P_kp_xy)
PARAM_ADD(PARAM_FLOAT, P_kp_z,  &P_kp_z)
PARAM_ADD(PARAM_FLOAT, P_kd_xy, &P_kd_xy) 
PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)
PARAM_ADD(PARAM_FLOAT, P_kd_z,  &P_kd_z)
PARAM_ADD(PARAM_FLOAT, P_ki_xy, &P_ki_xy)
PARAM_ADD(PARAM_FLOAT, P_ki_z,  &P_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)

PARAM_ADD(PARAM_FLOAT, R_kp_xy, &R_kp_xy)
PARAM_ADD(PARAM_FLOAT, R_kd_xy, &R_kd_xy) 
PARAM_ADD(PARAM_FLOAT, R_ki_xy, &R_ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_R_xy, &i_range_R_xy)

PARAM_ADD(PARAM_FLOAT, R_kp_z,  &R_kp_z)
PARAM_ADD(PARAM_FLOAT, R_kd_z,  &R_kd_z)
PARAM_ADD(PARAM_FLOAT, R_ki_z,  &R_ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_R_z, &i_range_R_z)

PARAM_ADD(PARAM_FLOAT, b1_d_x, &b1_d.x)
PARAM_ADD(PARAM_FLOAT, b1_d_y, &b1_d.y)
PARAM_ADD(PARAM_FLOAT, b1_d_z, &b1_d.z)

PARAM_ADD(PARAM_FLOAT, CF_mass, &m)

PARAM_ADD(PARAM_UINT8, AttCtrl, &attCtrlEnable)
PARAM_ADD(PARAM_UINT8, Tumbled, &tumbled)
PARAM_ADD(PARAM_UINT8, Error_Reset, &errorReset)
PARAM_ADD(PARAM_UINT8, MotorStop, &motorstop_flag)
PARAM_GROUP_STOP(GTC_Params)


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

LOG_GROUP_START(setpointZ_GTC)
LOG_ADD(LOG_UINT32, xy, &setpointZ_GTC.xy)
LOG_ADD(LOG_INT16,  z, &setpointZ_GTC.z)

LOG_ADD(LOG_UINT32, vxy, &setpointZ_GTC.vxy)
LOG_ADD(LOG_INT16,  vz, &setpointZ_GTC.vz)

LOG_ADD(LOG_UINT32, axy, &setpointZ_GTC.axy)
LOG_ADD(LOG_INT16,  az, &setpointZ_GTC.az)
LOG_GROUP_STOP(setpointZ_GTC)

LOG_GROUP_START(miscStatesZ_GTC)
LOG_ADD(LOG_UINT32, OF_xy, &miscStatesZ_GTC.OF_xy)
LOG_ADD(LOG_INT16,  RREV, &miscStatesZ_GTC.RREV)

LOG_ADD(LOG_UINT32, M_xy, &miscStatesZ_GTC.Mxy)
LOG_ADD(LOG_UINT32, FM_z, &miscStatesZ_GTC.FMz)

LOG_ADD(LOG_UINT32, MS12, &miscStatesZ_GTC.MS12)
LOG_ADD(LOG_UINT32, MS34, &miscStatesZ_GTC.MS34)

LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
LOG_GROUP_STOP(miscStatesZ_GTC)





LOG_GROUP_START(F_M)
LOG_ADD(LOG_FLOAT, F_thrust, &F_thrust)
LOG_ADD(LOG_FLOAT, M_roll, &M.x)
LOG_ADD(LOG_FLOAT, M_pitch, &M.y)
LOG_ADD(LOG_FLOAT, M_yaw, &M.z)
LOG_GROUP_STOP(F_M)