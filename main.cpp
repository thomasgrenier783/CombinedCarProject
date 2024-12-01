/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "Tickers.h"
// #include <chrono>


#define TI 0.001    //1kHz sample time
#define DELAY 1ms
#define SERVO_PERIOD 0.02f
#define REFERENCE 0.0f  //car should stay in center, so the reference should be zero


InterruptIn button(PC_13);  //PC13 is the pin dedicated to the blue user push button
DigitalOut led(PA_5); // Will be used to indicate whether we are in wait, stop, or go

PwmOut servo_control_signal(PB_10);     //Define D6 as steering PWM output
PwmOut motor_control_signal(PB_4);      //Define D5 as motor PWM output pin

AnalogOut brake_control_signal(PA_6);   // PA_6 is input to pin D12

// AnalogIn proportional_gain_input(PA_0);
// AnalogIn derivative_gain_input(PA_1);
// AnalogIn integral_gain_input(PA_4);          //PA_4 is pin A2

AnalogIn left_motor_speed_input(PA_1);      // PA_1 is input to pin A1
AnalogIn right_motor_speed_input(PA_4);     // PA_4 is input to pin A2
AnalogIn right_position_sensor_input(PC_1); // PC_1 is input to pin A4
AnalogIn left_position_sensor_input(PB_0);  // PB_0 is input to pin A3


AnalogIn Battery_input_signal(PC_0);        //PC_0 is pin A5
AnalogIn Left_Bumper_input_signal(PA_8);    //PA_8 is pin D8
AnalogIn Right_Bumper_input_signal(PA_9);   //PA_9 is pin D7


static int mode = 0;
static int prevMode = 0;

//Steering Variables
float feedback = 0;
float KP = 0;
float KD = 0;
float u = 0.075;
float KI = 0;


//Motor Variables
const float threshold_voltage = 2.0;   // Set threshold voltage (in Volts) for "high"
int high_count = 0;                    // Counts switches from low to high
bool is_high = false;                  // Tracks if the signal is currently above threshold
const int sampling_interval_us = 10; // Sampling every 0.001ms (10us)
const float period_20kHz = 0.00005f;   // 20 kHz period (50 us)
float d = 0;

float sk = 0.5f;           // Steering constant (range: 0.0 to 1.0) 
float kp = 1.0f;           // Proportional gain constant 
float ki = 0.5f;           // Integral gain constant 
float integralMax = 0.3f;  // Maximum integral term for anti-windup 
float integralMin = -0.3f; // Minimum integral term for anti-windup 

float motorDutyCycle;

// Independent variables 
float BLV = 0;
float BUMP = 0;
float LDLED = 0;

// Variables for PI controller 
float integral = 0.0f; 
float previousError = 0.0f; 

void steeringCalculateControl();
void steeringControlUpdate();
void motorControlUpdate();
void motorCalculateControl();
void modeIndicator();

//Ticker setup
Tickers steering_calculate_control_ticker(steeringCalculateControl, 1); 
Tickers steering_control_update_ticker(steeringControlUpdate, 25);
Tickers motor_calculate_control_ticker(motorControlUpdate, 1);
Tickers motor_control_update_ticker(motorCalculateControl,100); 
Tickers mode_indicator(modeIndicator, 100); 

// Function to map a range 
float mapRange(float value, float inMin, float inMax, float outMin, float outMax) { 
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin; 
} 

// Function to control PWM duty cycle
void setPWMDutyCycle(float duty_cycle) {
    // Ensure the duty cycle is between 0% and 100%
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    
    // Set the PWM period to 20 kHz (50 us)
    motor_control_signal.period(period_20kHz);

    // Set the PWM pulse width based on the duty cycle
    motor_control_signal.pulsewidth(period_20kHz * duty_cycle);
}


// Function to calculate duty cycle with proportional gain
float calculateDutyCycle(float sk, float kp) {
    // Scale sk by kp to influence duty cycle mapping
    float scaledSk = sk * kp;
 
    // Ensure scaledSk is clamped between 0 and 1
    if (scaledSk > 1.0f) scaledSk = 1.0f;
    if (scaledSk < 0.0f) scaledSk = 0.0f;
 
    // Map scaledSk to the duty cycle range (20% to 80%)
    return 0.2f + (scaledSk * 0.6f);
}

void modeIndicator()
{
    if (mode==0)
    {
        led=0;
    }
    else if (mode == 1)
    {
        led=!led;
    }
    else if (mode == 2)
    {
        led=1;
        static int count = 0;
        if (right_position_sensor_input.read() <= 0.005 & left_position_sensor_input.read() <= 0.005)
        {
            
            if (count == 5)
            {
                mode = 1;
                count = 0;
            }
            else {
                count = count+1;
            }

        }
        else {
        count=0;
        }
    }
    else
    {
        mode = 0;
    }

}

void calculateFeedback()
{
   feedback = (left_position_sensor_input - right_position_sensor_input);
}

void readProportionalGain()
{
    KP = 0.3*proportional_gain_input.read();
}

void readDerivativeGain()
{
    KD = 0.3*derivative_gain_input.read();
}

void readIntegralGain()
{
    KI = 0.3*integral_gain_input.read();
}

void readBLV()
{
  BLV = 7.2*Battery_input_signal.read()/0.85; //Battery used is 7.2V and the voltage divider used outputs 2.8 volts at full voltage. Divided by 3.3, that yields 0.85. 
}
// void readBUMP=


void steeringCalculateControl()
{
    static float area_prior=0;
    static float error_prior=0;

    calculateFeedback();
    float error_current = REFERENCE-feedback;
    // readProportionalGain();
    // readDerivativeGain();
    // readIntegralGain();
    float errorChange = error_prior * (error_current-error_prior)/TI;
    float area_current = TI*error_current+area_prior;
    u = KP*error_current + KD*errorChange + KI*area_current;
    // printf("Controller Output: %.4f\n\n",controllerOutput);
    
    error_prior=error_current;
    area_prior=area_current;

    u=(u+0.075);

    if(u<=0.05){
        u=0.05;
    }
    else if(u>=0.10){
        u=0.1;
    }
    else {
        u = u;
    }

    sk = 0.5*(0.025 - abs(u-0.075))/0.025;
    if (sk<=0.2)
        sk=0.2;
    if (sk>=0.8)
        sk=0.8;
}



void steeringControlUpdate(void){
  
    servo_control_signal=u;
    
}

void motorCalculateControl()
{
    // Calculate the duty cycle based on sk and kp
    motorDutyCycle = calculateDutyCycle(sk, kp);
}

void motorControlUpdate()
{
    // Update the PWM output with the calculated duty cycle 
    motor_control_signal.write(motorDutyCycle); 
    d = motor_control_signal;
}

void modeSwitch()
{
    if (mode==0)
    {
        mode=1;
    }
    else if (mode == 1)
    {
        mode=2;
    }
    else if (mode == 2)
    {
        mode=0;
    }
    else
    {
        mode = 0;
    }
}

int main()
{
    servo_control_signal.period(SERVO_PERIOD);
    
    button.rise(&modeSwitch);
    steering_calculate_control_ticker.start(); 
    steering_control_update_ticker.start();
    motor_calculate_control_ticker.start();
    motor_control_update_ticker.start(); 
    mode_indicator.start();  

    
    
    // Set the PWM frequency to 20 kHz 
    motor_control_signal.period(0.00005f); // 50 microseconds = 20 kHz frequency 
    // Variables for PI controller 
    float integral = 0.0f; 
    float previousError = 0.0f; 
  
    printf("\nKP\tKD\tKI\tPosition\tControl\tMotor\tBatt V.\n");

    while (true) {
        //Following information printed so it can be copied into a csv file
        mode_indicator.update();
        readBLV();
        
    	

	
        //Run mode loop
        if(mode==2)
        {
            if(BLV<4)
            {
                printf("Low Voltage");
                mode=1;	
            }

            if (Left_Bumper_input_signal.read()<0.5 | Right_Bumper_input_signal.read()<0.5)
            {
                printf("Bumper Stop");
                mode=1;
            }

            steering_calculate_control_ticker.update(); 
            steering_control_update_ticker.update();
            motor_calculate_control_ticker.update();
            motor_control_update_ticker.update();
            if (prevMode==1){ 
                prevMode=2;
                printf("\nRun Mode\nKP\tKD\tPosition\tControl\tMotor\n"); 
            }
            printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", KP,KD,KI,feedback*3.3,u,d,right_position_sensor_input.read(),left_position_sensor_input.read(),BLV);

        }
        else if(mode==0)
        {
            servo_control_signal = 0.075;
            motor_control_signal = 0.0;
            d=0;
            // steering_calculate_control_ticker.update(); 
            // motor_calculate_control_ticker.update();
            if (prevMode==2){
                prevMode=0;
                printf("\nStop Mode\nKP\tKD\tPosition\tControl\tMotor\n");
            }
        }
       //Wait mode loop
        else if(mode==1)
        {
            servo_control_signal = 0.075;
            motor_control_signal = 0.0;
            // steering_calculate_control_ticker.update(); 
            // motor_calculate_control_ticker.update();
            readProportionalGain();
            readDerivativeGain();
            readIntegralGain();
            if (prevMode==0 | prevMode==2){ 
                prevMode=1;
                printf("\nWait Mode\nKP\tKD\tPosition\tControl\tMotor\n"); 
            }
            printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", KP,KD,KI,feedback*3.3,u,d,right_position_sensor_input.read(),left_position_sensor_input.read(),BLV);

            
        }
        else
        {
            mode=0;
        }

    }
    //  ThisThread::sleep_for(DELAY);
}