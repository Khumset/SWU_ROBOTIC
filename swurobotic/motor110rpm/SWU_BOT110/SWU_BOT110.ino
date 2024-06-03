/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 ********************************************************************************
 * DESCRIPTION:
 *
 * This example shows how to drive 2 motors using 4 PWM pins (2 for each motor)
 * with 2-channel motor driver.
 * 
 * 
 * CONNECTIONS:
 * 
 * Arduino D3  - Motor Driver PWM 1A Input
 * Arduino D9  - Motor Driver PWM 1B Input
 * Arduino D10 - Motor Driver PWM 2A Input
 * Arduino D11 - Motor Driver PWM 2B Input
 * Arduino GND - Motor Driver GND
 *
 *
 * AUTHOR   : Kong Wai Weng
 * COMPANY  : Cytron Technologies Sdn Bhd
 * WEBSITE  : www.cytron.io
 * EMAIL    : support@cytron.io
 *
 *******************************************************************************/
#include "Arduino.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <Arduino_FreeRTOS.h>
#include "CytronMotorDriver.h"

// ROS node handle
ros::NodeHandle nh;
std_msgs::Float32 right_msg;
std_msgs::Float32 left_msg;
std_msgs::Float32 s_right_msg;
std_msgs::Float32 s_left_msg;
ros::Publisher right_pub("right", &right_msg);
ros::Publisher left_pub("left", &left_msg);


#define ENCA_M1 21  // encoder A to Interrupt pin  Uno(2,3)
#define ENCB_M1 22  // encoder B to Digitalpin

#define ENCA_M2 20  // encoder A to Interrupt pin  Uno(2,3)
#define ENCB_M2 23  // encoder B to Digitalpin
#define LILIM_PWM 255

// Paraeter robot
#define Wheelbase_Width 0.182  //18.2 cm to m
#define WheelDimeter 0.068       //68 mm to m

// Configure the Gain value
#define kp 1
#define ki 6
//motor1 >> rightmotor
//motor2 >> leftmotor
float v_m1 = 0;  //setpoint_motor1
float v_m2 = 0;  //setpoint_motor2
long prevT1 = 0;
long prevT2 = 0;
int posPrev_m1 = 0;
int posPrev_m2 = 0;
volatile int pos_m1 = 0;
volatile int pos_m2 = 0;
float velocity_m1 = 0;
float velocity_m2 = 0;

//command speed to drive MDD3A
int set_m1 = 0;
int set_m2 = 0;

float eintegral_m1 = 0;
float eintegral_m2 = 0;

float v1filt = 0;
float v1prev = 0;
float v2filt = 0;
float v2prev = 0;

float v_linear = 0.0;
float v_angular = 0.0;

// Configure the motor driver.
CytronMD motor1(PWM_PWM, 6, 7);  // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor2(PWM_PWM, 8, 9);  // PWM 2A = Pin 10, PWM 2B = Pin 11.

// Twist message callback function
void cmdVelCallback(const geometry_msgs::Twist &msg) {
  //Subscriber linear,_angular velocity  Differential Drive Robot
   v_linear = msg.linear.x;    //unit m/s
   v_angular = msg.angular.z;  //unit rad/s

  //Calculating Wheel Velocities for a Differential Drive Robot
  float vr = v_linear + (v_angular * Wheelbase_Width) / 2;
  float vl = v_linear - (v_angular * Wheelbase_Width) / 2;

  //speed unit converter m/s to rpm >> rpm = (60/PI*WheelDimeter) * v(m/s)
  float v_wl = 60 / (PI * WheelDimeter) * vl;
  float v_wr = 60 / (PI * WheelDimeter) * vr;
  //set_poin speed motor
  v_m1 =  v_wr;
  v_m2 =  v_wl;

}
// ROS subscriber for cmd_vel topic
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", cmdVelCallback);


void TaskMotor1(void *pvParameters);
void TaskMotor2(void *pvParameters);

void setup() {
  Serial.begin(57600);
  nh.initNode();
  //Advertised these publishers,subscribers
  nh.subscribe(cmdVelSub);
  nh.advertise(right_pub);
  nh.advertise(left_pub);
  pinMode(ENCA_M1, INPUT);
  pinMode(ENCB_M1, INPUT);
  pinMode(ENCA_M2, INPUT);
  pinMode(ENCB_M2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_M1), Encoder_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), Encoder_m2, RISING);

  xTaskCreate(TaskMotor1, "Motor1", 128, NULL, 1, NULL);
  xTaskCreate(TaskMotor2, "Motor2", 128, NULL, 1, NULL);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

void TaskMotor1(void *pvParameters) {
  long currT;
  float deltaT;
  float rpm_m1, e1, output_m1;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; 

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    currT = micros();
    deltaT = ((float)(currT - prevT1)) / 1.0e6;
    velocity_m1 = (pos_m1 - posPrev_m1) / deltaT;
    posPrev_m1 = pos_m1;
    prevT1 = currT;

    // Convert count/s to RPM
    rpm_m1 = velocity_m1 / 990.0 * 60.0;
    v1filt = 0.854 * v1filt + 0.0728 * rpm_m1 + 0.0728 * v1prev;
    v1prev = rpm_m1;

    // PI Control M1
    e1 = v_m1 - v1filt;                         // error = setpoint - speedmotor
    eintegral_m1 = eintegral_m1 + e1 * deltaT;  // integral error
    output_m1 = kp * e1 + ki * eintegral_m1;

    set_m1 = (int)output_m1;

    if (set_m1 > LILIM_PWM) set_m1 = LILIM_PWM;

    motor1.setSpeed(set_m1);
    // Convert RPM to m/s 
    float v_right = (v1filt * PI * WheelDimeter) / 60;
    // publishers right wheel speed
    right_msg.data =  v_right;
    right_pub.publish(&right_msg);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskMotor2(void *pvParameters) {
  long currT;
  float deltaT;
  float rpm_m2, e2, output_m2;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; 

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    currT = micros();
    deltaT = ((float)(currT - prevT2)) / 1.0e6;
    velocity_m2 = (pos_m2 - posPrev_m2) / deltaT;
    posPrev_m2 = pos_m2;
    prevT2 = currT;

    // Convert count/s to RPM
    rpm_m2 = velocity_m2 / 990.0 * 60.0;
    v2filt = 0.854 * v2filt + 0.0728 * rpm_m2 + 0.0728 * v2prev;
    v2prev = rpm_m2;

    // PI Control M2
    e2 = v_m2 - v2filt;                         // error = setpoint - speedmotor
    eintegral_m2 = eintegral_m2 + e2 * deltaT;  // integral error
    output_m2 = kp * e2 + ki * eintegral_m2;

    set_m2 = (int)output_m2;

    if (set_m2 > LILIM_PWM) set_m2 = LILIM_PWM;

    motor2.setSpeed(set_m2);
    // Convert RPM to m/s 
    float v_left = (v2filt * PI * WheelDimeter) / 60;
    // Publish left wheel speed
    left_msg.data = v_left;
    left_pub.publish(&left_msg);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void Encoder_m1() {
  int b = digitalRead(ENCB_M1);
  int increment = (b > 0) ? 1 : -1;
  pos_m1 = pos_m1 + increment;
}

void Encoder_m2() {
  int b = digitalRead(ENCB_M2);
  int increment = (b > 0) ? 1 : -1;
  pos_m2 = pos_m2 + increment;
}
