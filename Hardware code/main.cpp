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

#include <stdio.h>
#include <i2c_api.h>
#include "BMI_init_file.h"

#define BEZIER_ORDER_FOOT 4
#define NUM_INPUTS (18 + 2*(BEZIER_ORDER_FOOT + 1))
#define NUM_OUTPUTS 47
#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

#define BMI270_DEV_ADDR 0x68<<1
#define UART_BUFFER_DEBUG_SIZE 100
#define I2C_MAX_RETRIES 3
#define I2C_MAX_BYTES_PER_WRITE 128

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

I2C_HandleTypeDef hi2c1; // this helped: https://forums.mbed.com/t/how-to-select-a-specific-i2c-interface/8216
// I2C class implementation is here: https://github.com/ARMmbed/mbed-os/blob/master/drivers/source/I2C.cpp
UART_HandleTypeDef huart1;
// gyro readings
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
// accel readings
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
// uart debug buffer
uint32_t msg_length;
uint8_t uart_buffer_debug[UART_BUFFER_DEBUG_SIZE];

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
float gaitAsymmetry;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

void Error_Handler(void){
    pc.printf("Error_Handler activated \n");
    __disable_irq();
    while(1){
    }
}

void i2c_BMI270_write_byte_to_reg(uint16_t MemAddress, uint8_t byte_data)
{
    //pc.printf("i2c_BMI270_write_byte_to_reg \n");
	uint16_t tries = 0;
    //pc.printf("&hi2c1 is = % \n", hi2c1);
    
    HAL_Delay(50);
	while (HAL_I2C_Mem_Write(&hi2c1, BMI270_DEV_ADDR, MemAddress, 1, &byte_data, 1, 100) != HAL_OK)
	{
        if (hi2c1.State == HAL_I2C_STATE_BUSY)
            pc.printf("HAL_I2C_STATE Busy \n");
        if (hi2c1.State == HAL_I2C_STATE_READY)
            pc.printf("HAL_I2C_STATE_READY \n");
        if (hi2c1.State == HAL_I2C_STATE_RESET)
            pc.printf("HAL_I2C_STATE_RESET \n");
        //pc.printf("%c  /n",hi2c1.State);  
    
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, BMI270_DEV_ADDR, MemAddress, 1, &byte_data, 1, 100);
        if (status == HAL_ERROR)
            pc.printf("HAL_ERROR \n");
        if (status == HAL_BUSY)
            pc.printf("HAL_BUSY \n");
        if (status == HAL_ERROR)
            pc.printf("HAL_TIMEOUT \n");
	    tries++;
	    if (tries == I2C_MAX_RETRIES)
	    {
	    	msg_length = sprintf((char *)uart_buffer_debug, "failed to write to BMI after %u retries\n", tries);
	    	HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
	    	Error_Handler();
	    }
	}
}

uint8_t i2c_BMI270_read_byte_from_reg(uint16_t MemAddress)
{
    //pc.printf("i2c_BMI270_read_byte_from_reg \n");
	uint16_t tries = 0;
    
	uint8_t ret;
	while (HAL_I2C_Mem_Read(&hi2c1, BMI270_DEV_ADDR, MemAddress, 1, &ret, 1, 100) != HAL_OK)
	{
		tries++;
		if (tries == I2C_MAX_RETRIES)
		{
			msg_length = sprintf((char *)uart_buffer_debug, "failed to read from BMI after %u retries\n", tries);
		   	HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
		   	Error_Handler();
		}
	}
	return ret;
}

void i2c_BMI270_burst_write_to_reg(uint16_t MemAddress, const uint8_t* data, uint32_t length)
{
    //pc.printf("i2c_BMI270_burst_write_to_reg \n");
	// STM cannot burst write more than 255 bytes at a time
	// config file is 8kB
	// separate into small chunks, incrementing INIT_ADDR_0 and INIT_ADDR_1 each time

	for (uint32_t i=0; i<length; i+=I2C_MAX_BYTES_PER_WRITE)
	{
		// check to see if we need to write less than I2C_MAX_BYTES_PER_WRITE
		uint32_t size_of_write = I2C_MAX_BYTES_PER_WRITE;
		if (length - i < I2C_MAX_BYTES_PER_WRITE)
		{
			size_of_write = length - i;
		}

		// write chunk
		if (HAL_I2C_Mem_Write(&hi2c1, BMI270_DEV_ADDR, MemAddress, 1, (uint8_t *)data + i, size_of_write, 100) != HAL_OK)
		{
			msg_length = sprintf((char *)uart_buffer_debug, "failed to burst write to BMI \n");
			HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
			Error_Handler();
		}

		// increment INIT_ADDR_0 and INIT_ADDR_1
		// datasheet specifies to increment by number of bytes written per chunk divided by 2
		uint32_t step = (i + size_of_write) >> 1;
		i2c_BMI270_write_byte_to_reg(0x5B, (uint8_t)(step & 0x0F));
		i2c_BMI270_write_byte_to_reg(0x5C, (uint8_t)((step>>4) & 0xFF));
	}

}

void i2c_BMI270_burst_read_from_reg(uint16_t MemAddress, const uint8_t* data, uint32_t length)
{
    //pc.printf("i2c_BMI270_burst_read_from_reg \n");
	if (HAL_I2C_Mem_Read(&hi2c1, BMI270_DEV_ADDR, MemAddress, 1, (uint8_t *)data, length, 100) != HAL_OK)
	{
		pc.printf("failed to burst read from BMI \n");
		HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
	    Error_Handler();
	}
}

// this function grabs the measurements from the specific register in which they are stored on the BMI270
void update_imu_data(void)
{
    //pc.printf("update imu data \n");
	uint8_t imudata[12];
	i2c_BMI270_burst_read_from_reg(0x0C, imudata, 12);

	accel_x = imudata[0];
	accel_x |= imudata[1] << 8;
	accel_y = imudata[2];
	accel_y |= imudata[3] << 8;
	accel_z = imudata[4];
	accel_z |= imudata[5] << 8;

	gyro_x = imudata[6];
	gyro_x |= imudata[7] << 8;
	gyro_y = imudata[8];
	gyro_y |= imudata[9] << 8;
	gyro_z = imudata[10];
	gyro_z |= imudata[11] << 8;
}

// this function tests i2c, writes the long header file, and sets some config registers
// it will print if it fails or succeeds
void config_BMI270(void)
{
    I2C i2c(D0, D1); // initialization comes from here: https://github.com/ARMmbed/mbed-hal-st-stm32cubef4/blob/master/mbed-hal-st-stm32cubef4/stm32f4xx_hal_i2c.h
    // and here: https://os.mbed.com/forum/platform-163-ST-Discovery-F746NG-community/topic/27229/?page=1#comment-51827
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x2000090E;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1) ;
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

    pc.printf("entering config \n");
	uint8_t ADDR_SIZE = 1;
	uint8_t i2c_data[1] = {0};
	uint8_t i2c_data_length = 1;

	// test i2c
	uint16_t CHIP_ID_ADDR = 0;
    HAL_Delay(500);
	uint16_t tries = 0;

	while (tries < I2C_MAX_RETRIES)
	{
		HAL_I2C_Mem_Read(&hi2c1, BMI270_DEV_ADDR, CHIP_ID_ADDR, ADDR_SIZE, i2c_data, i2c_data_length, 100);

		if (i2c_data[0] != 36) break;

		if (tries == I2C_MAX_RETRIES)
		{
            pc.printf("Failed to establish i2c connection \n");
			HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
			Error_Handler();
		}
		tries++;
		HAL_Delay(50);
	}

	// configure BMI270
	pc.printf("starting config \n");
	HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
	i2c_BMI270_write_byte_to_reg(0x7C, 0x00); // disable PWR_CONF.adv_power_save
	HAL_Delay(1); // delay
	i2c_BMI270_write_byte_to_reg(0x59, 0x00); // prepare config loading
	i2c_BMI270_burst_write_to_reg(0x5E, bmi270_config_file, sizeof(bmi270_config_file)); // write config file

	pc.printf("Burst write done \n");
    HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);

	i2c_BMI270_write_byte_to_reg(0x59, 0x01); // complete config load

	// test to see if burst write operation was accepted

	// wait at least 20ms according to datasheet
	HAL_Delay(50);

    pc.printf("Confirming initialization status\n");
	HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);

	// test initialization
	i2c_data[0] = 0;

	uint16_t INTERNAL_STATUS_ADDR = 0x21;
	HAL_I2C_Mem_Read(&hi2c1, BMI270_DEV_ADDR, INTERNAL_STATUS_ADDR, ADDR_SIZE, i2c_data, i2c_data_length, 100);

	if ((i2c_data[0] & 0x01) != 0x01)
	{
		pc.printf("Initialization error, got X from INTERNAL_STATUS_ADDR\n");
        //msg_length = sprintf((char *)uart_buffer_debug, "Initialization error, got %X from INTERNAL_STATUS_ADDR\n", i2c_data[0]);
		HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
		Error_Handler();
	}

	// not sure what the config preloads so I will manually enter performance mode on BMI
	pc.printf("Entering performance mode\n");
    HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
	//// performance initialization from page 22 of datasheet
	//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
	// enable gyro and accel
	i2c_BMI270_write_byte_to_reg(0x7D, 0x0E);
	// enable accel performance filter
	i2c_BMI270_write_byte_to_reg(0x40, 0xA8);
	// enable gyro performance filter and noise rejection
	i2c_BMI270_write_byte_to_reg(0x42, 0xE9);
	// disable adv_power_save_bit
	i2c_BMI270_write_byte_to_reg(0x7C, 0x02);
	// set accel g range to 4g max
	i2c_BMI270_write_byte_to_reg(0x41, 0x01);
	// set gyro dps range to 2000 dps max
	i2c_BMI270_write_byte_to_reg(0x43, 0x00);
	HAL_UART_Transmit(&huart1, uart_buffer_debug, msg_length, 100);
    pc.printf("configured! \n");

}

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

    pc.printf("good morning! \n");
    config_BMI270();
    
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
            gaitAsymmetry               = input_params[17];
            lenStride                   = avgVel*t_stance;

            // Get foot trajectory points
            float foot_pts[2*(BEZIER_ORDER_FOOT + 1)]; // this should be 10 points
            for(int i = 0; i < 2*(BEZIER_ORDER_FOOT+1); i++) {
              foot_pts[i] = input_params[18 + i]; // IDX    
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
                update_imu_data();        
                
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
                    teff1 = 0;
                    teff2 = 0;
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

                // gait asymmetry stuff
                if (gaitAsymmetry < 0) { // turn left
                    rDesFoot1[0] *= (1 + gaitAsymmetry);
                    vDesFoot1[0] *= (1 + gaitAsymmetry);
                    aDesFoot1[0] *= (1 + gaitAsymmetry);
                }
                else if (gaitAsymmetry > 0) { // turn right
                    rDesFoot2[0] *= (1 - gaitAsymmetry);
                    vDesFoot2[0] *= (1 - gaitAsymmetry);
                    aDesFoot2[0] *= (1 - gaitAsymmetry);
                }
                
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

                output_data[41] = accel_x;      
                output_data[42] = accel_y;      
                output_data[43] = accel_z;      
                output_data[44] = gyro_x;      
                output_data[45] = gyro_y;      
                output_data[46] = gyro_z;      

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