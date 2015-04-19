#include <Wire.h>
#include "defines.h"
#include "AK8975.h"
#include "MPU6050.h"
#include "I2Cdev.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t cx, cy, cz;

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define SampleAveraging_8    0x03
#define DataOutputRate_75HZ   0x06
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00

float mag_heading_x;
float mag_heading_y;

#define Kp_YAW 1.0 // Yaw Porportional Gain  
#define Ki_YAW 0.00005 // Yaw Integrator Gain
float errorYaw[3]= {0,0,0};

#include <Servo.h>

#define CH1  3  // Pin numbers //av gauche
#define CH2  5  //ar droit
#define CH3  6  //ar gauche
#define CH4  7  //av droit

// I2C address 0x69 could be 0x68 depending on setup??.
int MPU9150_I2C_ADDRESS = 0x68;

volatile unsigned long startPeriod; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
volatile int rc[7];

int led = 13;

///////////////// DCM Variables /////////////////
//will be used for the computation
float DCM_Matrix[3][3]= {
  {1,0,0}
  ,{0,1,0}
  ,{0,0,1}
}; 

//will be used for the computation
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}};


//will be used for the computation
float Temporary_Matrix[3][3]={
  {0,0,0}
  ,{0,0,0}
  ,{0,0,0}
};

float Accel_Vector[3]= {0,0,0}; //accel values in m/s-2

float Gyro_Vector[3]= {0,0,0};//gyro values in rad/s

#define GRAVITY 8192
//#define Kp_ROLLPITCH 0.015
//#define Ki_ROLLPITCH 0.000010

#define Kp_ROLLPITCH 1.515/GRAVITY
#define Ki_ROLLPITCH 0.00101/GRAVITY

#define Gyro_Gain_X 0.0609
#define Gyro_Gain_Y 0.0609
#define Gyro_Gain_Z 0.0609
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in ra

float Omega_Vector[3]= {
  0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {
  0,0,0};//Omega Proportional correction
float Omega_I[3]= {
  0,0,0};//Omega Integrator
float Omega[3]= {
  0,0,0};//Omega

float G_Dt=0.02;    // Integration time for the gyros (DCM algorithm)

float errorRollPitch[3]= {
  0,0,0};
float roll=0;
float pitch=0;
float yaw=0;

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

uint8_t sensors[6] = {0,1,2,3,4,5};  
int SENSOR_SIGN[] = {1,-1,-1,-1,1,1,-1,1,-1};


///////////////// End DCM Variables /////////////////


long timer=0; //general purpose timer 
long timer_old;

long timer1=0; //general purpose timer 
long timer_old1;

//IMU Variables
int temp;
double dT;
int16_t C_X, C_Y, C_Z, G_X, G_Y, G_Z, A_X, A_Y, A_Z; //raw sensor data
float G_x, G_y, G_z, A_x, A_y, A_z; //calibrated sensor data


float command_pitch;
float err_pitch;
float pid_pitch;
float pitch_I;
float pitch_D;
float err_pitch_old;

float command_roll;
float err_roll;
float pid_roll;
float roll_I;
float roll_D;
float err_roll_old;

float throttle;

float err_yaw;
float pid_yaw;
float yaw_I;

float kp = 4.0; //3.0 //4.0
float ki = 0.0; //0.9
float kd = 0.6;

Servo Servo_1;
Servo Servo_2;
Servo Servo_3;
Servo Servo_4;


 
void setup()
{  
  pinMode(3, OUTPUT);  
  pinMode(9, OUTPUT);  
  pinMode(10, OUTPUT);  
  pinMode(11, OUTPUT); 
  
  attachInterrupt(0,calcInput,FALLING); 
  
  Serial.begin(115200);
  Wire.begin(0);
  
  pinMode(led, OUTPUT);  
  
  accelgyro.setSleepEnabled(false);
  
  accelgyro.setFullScaleGyroRange(3); //Gyro scale 2000deg/s
  delay(1);
  accelgyro.setFullScaleAccelRange(1);//Accel scale 4g
  delay(1);
  accelgyro.setClockSource(3);// Select GyroZ clock
  delay(1);
  accelgyro.setDLPFMode(4);// set bandwidth of both gyro and accelerometer to ~20 Hz
  delay(1);
  
  //IMU calibration
  calib_gyro(); //Bias computed once and values stored in program


  
  timer = micros();
 
  delay(20);
  }
  
  
  void loop()
{

  // Execute the fast loop
  // ---------------------
  if((micros()-timer)>=10000)   // 10ms => 100 Hz loop rate 
  { 
    timer_old = timer;
    timer=micros();
    G_Dt = (timer-timer_old)/1000000.0;      // Real time of loop run 

    fast_Loop();
 

  }
}


void fast_Loop()
{

  imu_Valget (); // read sensors
    

  //IMU Computation
 
  Matrix_update(); 
  Renormalization();
  Drift_cancellation();
  Euler_angles();


  command_pitch = -(rc[1]-1200.0)/15;
  err_pitch = command_pitch - ToDeg(pitch);
  pitch_D = -ToDeg(Omega[0]);
  pitch_I += (float)err_pitch*G_Dt; 
  //pitch_I = constrain(pitch_I,-50,50);
  pid_pitch = err_pitch*4.0+pitch_I*0.0+pitch_D*0.6; //P=10 I=15 D=5 was good //D=8 the limit //P=15 I=30 D=5
  
  
  //ROLL
  command_roll = (rc[0]-1200.0)/15;
  err_roll = command_roll - ToDeg(roll);
  roll_D = -ToDeg(Omega[1]);
  roll_I += (float)err_roll*G_Dt; 
  //roll_I = constrain(roll_I,-50,50);
  pid_roll = err_roll*4.0+roll_I*0.0+roll_D*0.8;

  //IMU_print();
  
  //YAW
  err_yaw = ToDeg(Omega[2]);
  yaw_I += (float)err_yaw*G_Dt; 
  pid_yaw = err_yaw*1.0+yaw_I;
  // pid_yaw=0;
  //Throttle

    throttle = constrain((rc[0]-1140)/2.0,0,255);
  //fail safe
  if(rc[1]>1900)
  {
    throttle=0;
  }

  if(throttle < 25)
  {
    pid_pitch=0;
    pid_roll=0;
    pid_yaw=0;

    pitch_I=0;
    roll_I=0;
    yaw_I=0;
  }
  
  IMU_print ();
  

  analogWrite(3, constrain(throttle+pid_roll-pid_pitch+pid_yaw,0,255));//arrière droit
  analogWrite(9, constrain(throttle-pid_roll+pid_pitch+pid_yaw,0,255));//avant gauche
  analogWrite(10, constrain(throttle-pid_roll-pid_pitch-pid_yaw,0,255));//arrière gauche
  analogWrite(11, constrain(throttle+pid_roll+pid_pitch-pid_yaw,0,255));//avant droit


/*
analogWrite(3, 0);//arrière droit
analogWrite(9, 0);//avant gauche
analogWrite(10, 0);//arrière gauche
analogWrite(11, 0);//avant droit
*/
}

void calcInput()
{
  //static variables are not reset when we exit the function
  static unsigned int pulseIn;
  static int channel;
  
      //length of current pulse
      pulseIn = (int)(micros() - startPeriod);
      
      //remember the time for next loop
      startPeriod = micros();

      //channel detector
      if(pulseIn >2000){
        channel = 0;
      }
      //store value
      else
      {
        rc[channel]=pulseIn;
        channel++; //increment channel for next time
      }
}
