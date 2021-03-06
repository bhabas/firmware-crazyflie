// CF HEADERS
#include "controller_gtc.h"


// XY POSITION PID
float P_kp_xy = 0.5f;
float P_kd_xy = 0.3f;
float P_ki_xy = 0.1f;
float i_range_xy = 0.3f;

// Z POSITION PID
float P_kp_z = 1.2f;
float P_kd_z = 0.35f;
float P_ki_z = 0.1f;
float i_range_z = 0.25f;

// XY ATTITUDE PID
float R_kp_xy = 0.004f;
float R_kd_xy = 0.0017f;
float R_ki_xy = 0.0f;
float i_range_R_xy = 1.0f;

// Z ATTITUDE PID
float R_kp_z = 0.003f;
float R_kd_z = 0.001f;
float R_ki_z = 0.002;
float i_range_R_z = 0.5f;



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

    x_d = mkvec(0.0f,0.0f,0.0f);
    v_d = mkvec(0.0f,0.0f,0.0f);
    a_d = mkvec(0.0f,0.0f,0.0f);
    

    tumbled = false;
    motorstop_flag = false;

    Moment_flag = false;
    policy_armed_flag = false;
    flip_flag = false;

    t = 0;
    execute_traj = false;




}

bool controllerGTCTest(void)
{
    return true;
}

void GTC_Command(setpoint_t *setpoint)
{   
    switch(setpoint->cmd_type){
        case 0: // Reset
            controllerGTCReset();
            break;


        case 1: // Position
            x_d.x = setpoint->cmd_val1;
            x_d.y = setpoint->cmd_val2;
            x_d.z = setpoint->cmd_val3;
            break;


        case 2: // Velocity
            v_d.x = setpoint->cmd_val1;
            v_d.y = setpoint->cmd_val2;
            v_d.z = setpoint->cmd_val3;
            break;


        case 3: // Acceleration
            a_d.x = setpoint->cmd_val1;
            a_d.y = setpoint->cmd_val2;
            a_d.z = setpoint->cmd_val3;
            break;

        case 4: // Tumble-Detection

            break;

        case 5: // Hard Set All Motorspeeds to Zero
            motorstop_flag = true;
            break;

        case 7: // Execute Moment-Based Flip

            M_d.x = setpoint->cmd_val1*1e-3;
            M_d.y = setpoint->cmd_val2*1e-3;
            M_d.z = setpoint->cmd_val3*1e-3;

            Moment_flag = setpoint->cmd_flag;
            break;

        case 8: // Arm Policy Maneuver
            RREV_thr = setpoint->cmd_val1;
            G1 = setpoint->cmd_val2;
            G2 = setpoint->cmd_val3;

            policy_armed_flag = setpoint->cmd_flag;

            break;
        case 9: // Trajectory Values

            s_0 = setpoint->cmd_val1;
            v = setpoint->cmd_val2;
            a = setpoint->cmd_val3;
            traj_type = setpoint->cmd_flag;

            t = 0.0f; // Reset t
            T = (a+fsqr(v))/(a*v); // Find trajectory manuever length [s]

            if(traj_type >= 0){
                execute_traj = true;
            }
            else{
                execute_traj = false;
            }

            break;

    }
    
    return 0;
}

void controllerGTCTraj()
{
    if(t<=v/a)
    {
        x_d.z = 0.5f*a*t*t + s_0;
        v_d.z = a*t;
        a_d.z = a;

    }

    // CONSTANT VELOCITY TRAJECTORY
    if(v/a < t)
    {
        x_d.z = v*t - fsqr(v)/(2.0f*a) + s_0;
        v_d.z = v;
        a_d.z = 0.0f;
    }

    // COMPLETE POSITION TRAJECTORY
    // if(v/a <= t && t < (T-v/a))
    // {
    //     x_d.z = v*t - fsqr(v)/(2.0f*a) + s_0;
    //     v_d.z = v;
    //     a_d.z = 0.0f;
    // }

    // if((T-v/a) < t && t <= T)
    // {
    //     x_d.z = (2.0f*a*v*T-2.0f*fsqr(v)-fsqr(a)*fsqr(t-T))/(2.0f*a) + s_0;
    //     v_d.z = a*(T-t);
    //     a_d.z = -a;
    // }

    t = t + dt;
    

    
}

void controllerGTC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

        if (setpoint->GTC_cmd_rec == true)
            {
                
                GTC_Command(setpoint);
                setpoint->GTC_cmd_rec = false;
            }

        if (errorReset){
            controllerGTCReset();
            errorReset = false;
            }

        if(execute_traj){
            controllerGTCTraj();
        }

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

        // =========== TRANSLATIONAL EFFORT =========== //
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
        temp1_v = veltmul(vneg(Kp_p), e_x);
        temp2_v = veltmul(vneg(Kd_p), e_v);
        temp3_v = veltmul(vneg(Ki_p), e_PI);
        P_effort = vadd3(temp1_v,temp2_v,temp3_v);

        temp1_v = vscl(m*g, e_3); // Feed-forward term
        temp2_v = vscl(m, a_d);

        F_thrust_ideal = vadd3(P_effort, temp1_v,temp2_v); 

        // =========== DESIRED BODY AXES =========== // 
        b3_d = vnormalize(F_thrust_ideal);
        b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
        temp1_v = vnormalize(vcross(b2_d, b3_d));
        R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations


        // ATTITUDE CONTROL
        if (attCtrlEnable){ 
            R_d = quat2rotmat(quat_d); // Desired rotation matrix from att. control
        }


        // =========== ROTATIONAL ERRORS =========== // 
        RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
        RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]

        temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
        e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]

        temp1_v = mvmul(RT_Rd, omega_d);        // [R'*R_d*omega_d]
        e_w = vsub(stateOmega, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

        // ROT. INTEGRAL ERROR
        e_RI.x += (e_R.x)*dt;
        e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

        e_RI.y += (e_R.y)*dt;
        e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

        e_RI.z += (e_R.z)*dt;
        e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

        // =========== CONTROL EQUATIONS =========== // 
        /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

        temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
        temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
        temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
        R_effort = vadd3(temp1_v,temp2_v,temp3_v);

        

        /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
        temp1_v = vcross(stateOmega, mvmul(J, stateOmega)); // [omega x J*omega]


        temp1_m = mmul(hat(stateOmega), RT_Rd); //  hat(omega)*R'*R_d
        temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
        temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

        temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
        Gyro_dyn = vsub(temp1_v,temp4_v);

        F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
        M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]

        // =========== THRUST AND MOMENTS [FORCE NOTATION] =========== // 
        if(!tumbled){
            
            if(policy_armed_flag == true){
                
                if(RREV >= RREV_thr && flip_flag == false){

                    flip_flag = true;

                    // UPDATE, RECORD, AND COMPRESS STATE VALUES AT FLIP TRIGGER
                    statePos_tr = statePos;
                    stateVel_tr = stateVel;
                    stateQuat_tr = stateQuat;
                    stateOmega_tr = stateOmega;

                    OF_y_tr = OF_y;
                    OF_x_tr = OF_x;
                    RREV_tr = RREV;

                    compressFlipStates();
                }

                if(flip_flag == true){
                    M_d.x = 0.0f;
                    M_d.y = -G1*1e-3;
                    M_d.z = 0.0f;

                    M = vscl(2.0f,M_d); // Need to double moment to ensure it survives the PWM<0 cutoff
                    F_thrust = 0.0f;

                }
            }
            else if(Moment_flag == true){
                M.x = M_d.x;
                M.y = M_d.y;
                M.z = M_d.z;

                M = vscl(2.0f,M_d); // Need to double moment to ensure it survives the PWM<0 cutoff
                F_thrust = 0.0f;

            }
            else{
                F_thrust = F_thrust;
                M = M;
            }
        }
        else if(tumbled){
            // consolePrintf("System Tumbled: \n");
            F_thrust = 0.0f;
            M.x = 0.0f;
            M.y = 0.0f;
            M.z = 0.0f;
        }


        // =========== CONVERT THRUSTS AND MOMENTS TO PWM =========== // 
        f_thrust = F_thrust/4.0f*Newton2g;
        f_roll = M.x/(4.0f*dp)*Newton2g;
        f_pitch = M.y/(4.0f*dp)*Newton2g;
        f_yaw = M.z/(4.0*c_tf)*Newton2g;


        // =========== INSERT PWM VALUES INTO CONTROL STRUCT =========== //
        if(motorstop_flag){
            control->thrust = 0.0f;
            control->roll = 0;
            control->pitch = 0;
            control->yaw = 0;
        }
        else{
            control->thrust = f_thrust;             // Need this value in PWM for onboard Extended Kalman Filter (Maybe? IDK...)
            control->roll = (int16_t)(f_roll*1e3f); // Convert Newtons into integer just for variable transfer
            control->pitch = (int16_t)(f_pitch*1e3f);
            control->yaw = (int16_t)(f_yaw*1e3f); 
        }

        
        

        // if (tick%200 ==  0)
        // {
            
        //     // printvec(F_thrust_ideal);
        //     DEBUG_PRINT("F_thrust: %.2f | M.x: %.2f | M.y: %.2f | M.z: %.2f\n",F_thrust,M.x*1e3f,M.y*1e3f,M.z*1e3f);
        //     DEBUG_PRINT("f_thrust: %.3f | f_roll: %.3f | f_pitch: %.3f | f_yaw: %.3f\n",f_thrust,f_roll,f_pitch,f_yaw);
        //     // DEBUG_PRINT("M1_pwm: %d | M2_pwm: %d | M3_pwm: %d | M4_pwm: %d\n",M1_pwm,M2_pwm,M3_pwm,M4_pwm);
        //     DEBUG_PRINT("thrust: %.1f | roll: %d | pitch: %d | yaw: %d\n",control->thrust,control->roll,control->pitch,control->yaw);

        // }
        
        compressEstStates();
        compressGTCSetpoint();
        compressMiscStates();

        

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

LOG_GROUP_START(StateEstZ_GTC)
LOG_ADD(LOG_UINT32, xy, &StateEstZ_GTC.xy)
LOG_ADD(LOG_INT16,  z, &StateEstZ_GTC.z)

LOG_ADD(LOG_UINT32, vxy, &StateEstZ_GTC.vxy)
LOG_ADD(LOG_INT16,  vz, &StateEstZ_GTC.vz)

LOG_ADD(LOG_UINT32, quat, &StateEstZ_GTC.quat)           

LOG_ADD(LOG_UINT32, wxy, &StateEstZ_GTC.wxy)
LOG_ADD(LOG_INT16,  wz, &StateEstZ_GTC.wz)

LOG_ADD(LOG_UINT32, OF_xy, &StateEstZ_GTC.OF_xy)
LOG_ADD(LOG_INT16,  RREV, &StateEstZ_GTC.RREV)
LOG_GROUP_STOP(StateEstZ_GTC)


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

LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
LOG_GROUP_STOP(miscStatesZ_GTC)



LOG_GROUP_START(FlipStatesZ_GTC)
LOG_ADD(LOG_UINT32, xy, &FlipStatesZ_GTC.xy)
LOG_ADD(LOG_INT16,  z, &FlipStatesZ_GTC.z)

LOG_ADD(LOG_UINT32, vxy, &FlipStatesZ_GTC.vxy)
LOG_ADD(LOG_INT16,  vz, &FlipStatesZ_GTC.vz)

LOG_ADD(LOG_UINT32, quat, &FlipStatesZ_GTC.quat)           

LOG_ADD(LOG_UINT32, wxy, &FlipStatesZ_GTC.wxy)
LOG_ADD(LOG_INT16,  wz, &FlipStatesZ_GTC.wz)

LOG_ADD(LOG_UINT32, OF_xy, &FlipStatesZ_GTC.OF_xy)
LOG_ADD(LOG_INT16,  RREV, &FlipStatesZ_GTC.RREV)

LOG_GROUP_STOP(FlipStatesZ_GTC)


LOG_GROUP_START(F_M)
LOG_ADD(LOG_FLOAT, F_thrust, &F_thrust)
LOG_ADD(LOG_FLOAT, M_roll, &M.x)
LOG_ADD(LOG_FLOAT, M_pitch, &M.y)
LOG_ADD(LOG_FLOAT, M_yaw, &M.z)
LOG_GROUP_STOP(F_M)

LOG_GROUP_START(Debug)
LOG_ADD(LOG_UINT8, Tumbled_Flag, &tumbled)
LOG_ADD(LOG_UINT8, Error_Reset_Flag, &errorReset)
LOG_ADD(LOG_UINT8, Motorstop_Flag, &motorstop_flag)

LOG_ADD(LOG_UINT8, Flip_Flag, &flip_flag)
LOG_ADD(LOG_UINT8, Trajectory_Flag, &execute_traj)
LOG_ADD(LOG_UINT8, Policy_Armed_Flag, &policy_armed_flag)
LOG_GROUP_STOP(Debug)
