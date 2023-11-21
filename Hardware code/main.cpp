#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"
#include "Matrix.h"
#include "MatrixMath.h"

#define BEZIER_ORDER_FOOT 4
#define NUM_INPUTS (17 + 2*(BEZIER_ORDER_FOOT + 1))
#define NUM_OUTPUTS 41

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA( PE_9, PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB( PA_5,  PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC( PC_6,  PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); //initialize the motor shield with a period of 12000 ticks or ~20kHZ
Ticker currentLoop;

// Variables for q1
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float angle1;
float velocity1;
float duty_cycle1;
float angle1_init;

// Variables for q2
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycle2;
float angle2_init;

// Variables for q3
float current3;
float current_des3 = 0;
float prev_current_des3 = 0;
float current_int3 = 0;
float angle3;
float velocity3;
float duty_cycle3;
float angle3_init;

// Variables for q4
float current4;
float current_des4 = 0;
float prev_current_des4 = 0;
float current_int4 = 0;
float angle4;
float velocity4;
float duty_cycle4;
float angle4_init;

// Fixed kinematic parameters
const float l_OA = .011; 
const float l_OB = .042; 
const float l_AC = .096; 
const float l_DE = .091;
const float m1 = .0393 + .2;
const float m2 = .0368; 
const float m3 = .00783;
const float m4 = .0155;
const float I1 = 0.0000251;  //25.1 * 10^-6;
const float I2 = 0.0000535;  //53.5 * 10^-6;
const float I3 = 0.00000925; //9.25 * 10^-6;
const float I4 = 0.0000222;  //22.176 * 10^-6;
const float l_O_m1 = 0.032;
const float l_B_m2 = 0.0344; 
const float l_A_m3 = 0.0622;
const float l_C_m4 = 0.0610;
const float N = 18.75;
const float Ir = 0.0035/pow(N, 2);

// New Kinematic Parameters
const float mU = 0.03; // 
const float lU = 0.2; // distance from wheel center to "user" CoM
const float I_U = 1.0/12*mU*pow(lU, 2);
const float m5 = 0.0424*6/2.205; // 6" length of 80/20 1010 profile
const float ma = 0.100; // 100 grams per wheel (6 oz. each) - weigh them!
const float mb = m5 + 2.0; // total guess; mass of motors plus mounting hardware
const float b = 0.267; // 12" 80/20
const float r = 0.05; // wheels radius
const float l_cb = 0.1524; // 6" (doesn't account for amount hanging off past connection points)
const float I_A = 0.5*ma*pow(r, 2); // thin solid disk
const float I_B = 1*pow(10, -3); // truly a random guess. Need to do more calcs

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;

// Control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;      
float K;
float D;

float t_swing;
float t_stance;
float phase_offset;
float ground_penetration;
float avgVel;
float nomHip[2];
float lenStride;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

// Current control interrupt function
void CurrentLoop()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f) - 15.0f);           // measure current
    velocity1 = encoderA.getVelocity()*PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf(fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*current_des1 + k_t*velocity1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycle1);
    if (absDuty1 > duty_max) {
        duty_cycle1 *= duty_max/absDuty1;
        absDuty1 = duty_max;
    }    
    if (duty_cycle1 < 0) { // backwards
        motorShield.motorAWrite(absDuty1, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty1, 0);
    }             
    prev_current_des1 = current_des1; 
    
    current2 = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f) - 15.0f);       // measure current
    velocity2 = encoderB.getVelocity()*PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf(fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max/absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorShield.motorBWrite(absDuty2, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty2, 0);
    }             
    prev_current_des2 = current_des2; 
    
    current3 = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f) - 15.0f);       // measure current
    velocity3 = encoderC.getVelocity()*PULSE_TO_RAD;                                  // measure velocity  
    float err_c3 = current_des3 - current3;                                             // current error
    current_int3 += err_c3;                                                             // integrate error
    current_int3 = fmaxf(fminf(current_int3, current_int_max), -current_int_max);      // anti-windup   
    float ff3 = R*current_des3 + k_t*velocity3;                                         // feedforward terms
    duty_cycle3 = (ff3 + current_Kp*err_c3 + current_Ki*current_int3)/supply_voltage;   // PI current controller
    
    float absDuty3 = abs(duty_cycle3);
    if (absDuty3 > duty_max) {
        duty_cycle3 *= duty_max/absDuty3;
        absDuty3 = duty_max;
    }    
    if (duty_cycle3 < 0) { // backwards
        motorShield.motorCWrite(absDuty3, 1);
    } else { // forwards
        motorShield.motorCWrite(absDuty3, 0);
    }             
    prev_current_des3 = current_des3; 

    current4 = -(((float(motorShield.readCurrentD())/65536.0f)*30.0f) - 15.0f);       // measure current
    velocity4 = -encoderD.getVelocity()*PULSE_TO_RAD;                                  // measure velocity  
    float err_c4 = current_des4 - current4;                                             // current error
    current_int4 += err_c4;                                                             // integrate error
    current_int4 = fmaxf(fminf(current_int4, current_int_max), -current_int_max);      // anti-windup   
    float ff4 = R*current_des4 + k_t*velocity4;                                         // feedforward terms
    duty_cycle4 = (ff4 + current_Kp*err_c4 + current_Ki*current_int4)/supply_voltage;   // PI current controller
    
    float absDuty4 = abs(duty_cycle4);
    if (absDuty4 > duty_max) {
        duty_cycle4 *= duty_max/absDuty4;
        absDuty4 = duty_max;
    }    
    if (duty_cycle4 < 0) { // backwards
        motorShield.motorDWrite(absDuty4, 1);
    } else { // forwards
        motorShield.motorDWrite(absDuty4, 0);
    }             
    prev_current_des4 = current_des4; 
}

int main (void)
{
// Object for 5th order Cartesian foot trajectory
    BezierCurve rDesFoot_bez(2, BEZIER_ORDER_FOOT);
    
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    pc.printf("%f", input_params[0]);
    
    while(1) {
        // If there are new inputs, this code will run
        if (server.getParams(input_params, NUM_INPUTS)) {
            
            // Get inputs from MATLAB          
            start_period                = input_params[0];    // First buffer time, before trajectory
            traj_period                 = input_params[1];    // Trajectory time/length
            end_period                  = input_params[2];    // Second buffer time, after trajectory
    
            angle1_init                 = input_params[3];    // Initial angle for left q1 (rad)
            angle2_init                 = input_params[4];    // Initial angle for left q2 (rad)
            angle3_init                 = input_params[5];    // Initial angle for right q1 (rad)
            angle4_init                 = input_params[6];    // Initial angle for right q2 (rad)
            
            K                           = input_params[7];    // Foot stiffness N/m // NOTE ME
            D                           = input_params[8];   // Foot damping N/(m/s) // NOTE ME
            duty_max                    = input_params[9];   // Maximum duty factor

            t_swing                     = input_params[10]; // Add other inputs
            t_stance                    = input_params[11];
            phase_offset                = input_params[12]; // this is as a fraction of a full stride, betwene 0 and 1
            ground_penetration          = input_params[13]; // a depth, in meters
            nomHip[0]                   = input_params[14]; // IDX
            nomHip[1]                   = input_params[15]; // IDX
            avgVel                      = input_params[16]; // IDX
            lenStride                   = avgVel*t_stance;

            // Get foot trajectory points
            float foot_pts[2*(BEZIER_ORDER_FOOT + 1)]; // this should be 10 points
            for(int i = 0; i < 2*(BEZIER_ORDER_FOOT+1); i++) {
              foot_pts[i] = input_params[17 + i]; // IDX    
            }
            rDesFoot_bez.setPoints(foot_pts);
            
            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop, current_control_period_us);
                        
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor A off
            motorShield.motorDWrite(0, 0); //turn motor B off
                         
            // Run experiment
            while(t.read() < start_period + traj_period + end_period) { 
                 
                // Read encoders to get motor states
                angle1 = encoderA.getPulses()*PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity()*PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses()*PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity()*PULSE_TO_RAD;      

                angle3 = encoderC.getPulses()*PULSE_TO_RAD + angle1_init;       
                velocity3 = encoderC.getVelocity()*PULSE_TO_RAD;    

                angle4 = encoderD.getPulses()*PULSE_TO_RAD + angle4_init;       
                velocity4 = encoderD.getVelocity()*PULSE_TO_RAD;         
                
                const float th1 = angle1;
                const float th2 = angle2;
                const float th3 = angle3;
                const float th4 = angle4;
                const float dth1 = velocity1;
                const float dth2 = velocity2;
                const float dth3 = velocity3;
                const float dth4 = velocity4;
 
                // Calculate the Jacobian
                float Jx_th1 = -l_AC*cos(th1 + th2) - l_DE*cos(th1) - l_OB*cos(th1);
                float Jx_th2 = -l_AC*cos(th1 + th2);
                float Jy_th1 =  l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1);
                float Jy_th2 =  l_AC*sin(th1 + th2);

                float Jx_th3 = l_AC*cos(th3 + th4) + l_DE*cos(th3) + l_OB*cos(th3);
                float Jx_th4 = l_AC*cos(th3 + th4);
                float Jy_th3 = l_AC*sin(th3 + th4) + l_DE*sin(th3) + l_OB*sin(th3);
                float Jy_th4 = l_AC*sin(th3 + th4);
                                
                // Calculate the forward kinematics (position and velocity)
                float xFoot1 = -l_OB*sin(th1) - l_AC*sin(th1 + th2) - l_DE*sin(th1);
                float yFoot1 = -l_OB*cos(th1) - l_AC*cos(th1 + th2) - l_DE*cos(th1);
                float dxFoot1 = Jx_th1*dth1 + Jx_th2*dth2;
                float dyFoot1 = Jy_th1*dth1 + Jy_th2*dth2;    

                float xFoot2 =  l_OB*sin(th3) + l_AC*sin(th3 + th4) + l_DE*sin(th3);
                float yFoot2 = -l_OB*cos(th3) - l_AC*cos(th3 + th4) - l_DE*cos(th3);
                float dxFoot2 = Jx_th3*dth3 + Jx_th4*dth4;
                float dyFoot2 = Jy_th3*dth3 + Jy_th4*dth4;   

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff1 = 0;
                float teff2 = 0;
                float vMult = 0;
                if (t < start_period) {
                    if (K > 0 || D > 0) {
                        K = 100; 
                        K = 10;
                    }
                    teff1 = 0;
                    teff2 = 0;
                }
                else if (t < start_period + traj_period)
                {
                    K                           = input_params[7];    // Foot stiffness N/m // NOTE ME
                    D                           = input_params[8];   // Foot damping N/(m/s) // NOTE ME
                    teff1 = fmod((t - start_period), (t_swing + t_stance));
                    teff2 = fmod((t - start_period + (t_swing + t_stance)*phase_offset), (t_swing + t_stance));
                    vMult = 1;
                }
                else
                {
                    teff1 = traj_period;
                    teff2 = traj_period;
                    vMult = 0;
                }
                
                // Get desired foot positions and velocities
                float rDesFoot1[2], vDesFoot1[2], aDesFoot1[2];
                if (teff1 < t_swing){
                    float swingPortion = teff1/t_swing;
                    rDesFoot_bez.evaluate(swingPortion, rDesFoot1);
                    rDesFoot_bez.evaluateDerivative(swingPortion, vDesFoot1);
                    rDesFoot_bez.evaluateDerivative2(swingPortion, aDesFoot1);
                    // vDesFoot1[0] /= t_swing;//traj_period;
                    // vDesFoot1[1] /= t_swing;
                    vDesFoot1[0] *= vMult;
                    vDesFoot1[1] *= vMult;
                }
                else{
                    float w = 3.1415;
                    float stancePortion = (teff1 - t_swing)/t_stance;
                    rDesFoot1[0] = 1 - stancePortion;
                    rDesFoot1[1] = -ground_penetration*sin(w*stancePortion);
                    vDesFoot1[0] = -1/t_stance;
                    vDesFoot1[1] = -w*ground_penetration*cos(w*stancePortion);
                    aDesFoot1[0] = 0;
                    aDesFoot1[1] = w*w*ground_penetration*sin(w*stancePortion);
                    
                    // Desired x is the end of the bezier traj so don't change
                }

                float rDesFoot2[2], vDesFoot2[2], aDesFoot2[2];
                if (teff2 < t_swing){
                    float swingPortion = teff2/t_swing;
                    rDesFoot_bez.evaluate(swingPortion, rDesFoot2);
                    rDesFoot_bez.evaluateDerivative(swingPortion, vDesFoot2);
                    rDesFoot_bez.evaluateDerivative2(swingPortion, aDesFoot2);
                    // vDesFoot2[0] /= t_swing;//traj_period;
                    // vDesFoot2[1] /= t_swing;
                    vDesFoot2[0] *= vMult;
                    vDesFoot2[1] *= vMult;
                }
                else{
                    float w = 3.1415;
                    float stancePortion = (teff2 - t_swing)/t_stance;
                    rDesFoot2[0] = 1 - stancePortion;
                    rDesFoot2[1] = -ground_penetration*sin(w*stancePortion);
                    vDesFoot2[0] = -1/t_stance;
                    vDesFoot2[1] = -w*ground_penetration*cos(w*stancePortion);
                    aDesFoot2[0] = 0;
                    aDesFoot2[1] = w*w*ground_penetration*sin(w*stancePortion);
                }
                
                rDesFoot1[0] -= nomHip[0]; // adjust for the hip position
                rDesFoot1[1] -= nomHip[1];
                rDesFoot2[0] -= nomHip[0];
                rDesFoot2[1] -= nomHip[1];

                rDesFoot1[0] *= lenStride; // adjust for the stride length
                rDesFoot2[0] *= lenStride;
                vDesFoot1[0] *= lenStride;
                vDesFoot2[0] *= lenStride;
                aDesFoot1[0] *= lenStride;
                aDesFoot2[0] *= lenStride;
                
                // Calculate error variables
                float e_x1 = -rDesFoot1[0] - xFoot1;
                float e_y1 = rDesFoot1[1] - yFoot1;
                float de_x1 = -vDesFoot1[0] - dxFoot1;
                float de_y1 = vDesFoot1[1] - dyFoot1;

                float e_x2 = -rDesFoot2[0] - xFoot2;
                float e_y2 = rDesFoot2[1] - yFoot2;
                float de_x2 = -vDesFoot2[0] - dxFoot2;
                float de_y2 = vDesFoot2[1] - dyFoot2;
        
                // Calculate virtual force on foot
                float fx1 = -aDesFoot1[0] + K*e_x1 + D*de_x1;
                float fy1 = aDesFoot1[1] + K*e_y1 + D*de_x1;

                float fx2 = -aDesFoot2[0] + K*e_x2 + D*de_x2;
                float fy2 = aDesFoot2[1] + K*e_y2 + D*de_x2;

                current_des1 = (Jx_th1*fx1 + Jy_th1*fy1)/k_t;          
                current_des2 = (Jx_th2*fx1 + Jy_th2*fy1)/k_t;  

                current_des3 = (Jx_th3*fx2 + Jy_th3*fy2)/k_t;          
                current_des4 = (Jx_th4*fx2 + Jy_th4*fy2)/k_t;
                
                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read();
                // motor 1 state
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = duty_cycle1;
                // motor 2 state
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= duty_cycle2;
                // motor 3 state
                output_data[11] = angle3;
                output_data[12] = velocity3;
                output_data[13] = current3;
                output_data[14] = current_des3;
                output_data[15]= duty_cycle3;
                // motor 4 state
                output_data[16] = angle4;
                output_data[17] = velocity4;
                output_data[18] = current4;
                output_data[19] = current_des4;
                output_data[20]= duty_cycle4;
                // foot state
                output_data[21] = xFoot1;
                output_data[22] = yFoot1;
                output_data[23] = dxFoot1;
                output_data[24] = dyFoot1;
                output_data[25] = rDesFoot1[0];
                output_data[26] = rDesFoot1[1];
                output_data[27] = vDesFoot1[0];
                output_data[28] = vDesFoot1[1];
                output_data[29] = aDesFoot1[0];
                output_data[30] = aDesFoot1[1];

                output_data[31] = xFoot2;
                output_data[32] = yFoot2;
                output_data[33] = dxFoot2;
                output_data[34] = dyFoot2;
                output_data[35] = rDesFoot2[0];
                output_data[36] = rDesFoot2[1];
                output_data[37] = vDesFoot2[0];
                output_data[38] = vDesFoot2[1];
                output_data[39] = aDesFoot2[0];
                output_data[40] = aDesFoot2[1];
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0); //turn motor C off
            motorShield.motorDWrite(0, 0); //turn motor D off
        
        } // end if
        
    } // end while
    
} // end main

