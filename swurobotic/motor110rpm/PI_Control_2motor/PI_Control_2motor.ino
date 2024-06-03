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

#include "CytronMotorDriver.h"

#define ENCA_M1 21  // encoder A to Interrupt pin  Uno(2,3)
#define ENCB_M1 22  // encoder B to Digitalpin

#define ENCA_M2 20  // encoder A to Interrupt pin  Uno(2,3)
#define ENCB_M2 23 // encoder B to Digitalpin

#define LILIM_PWM 255

// Configure the Gain value
#define kp 1
#define ki 9
float v_m1 = 50;  //setpoin_motor1
float v_m2 = -50;  //setpoin_motor2
long prevT = 0;
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

// Configure the motor driver.
CytronMD motor1(PWM_PWM, 6, 7);  // PWM 1A = Pin 3, PWM 1B = Pin 9.
CytronMD motor2(PWM_PWM, 8, 9);  // PWM 2A = Pin 10, PWM 2B = Pin 11.


void setup() {
  Serial.begin(9600);
  pinMode(ENCA_M1, INPUT);
  pinMode(ENCB_M1, INPUT);
  pinMode(ENCA_M2, INPUT);
  pinMode(ENCB_M2, INPUT);

  motor1.setSpeed(0);
  motor2.setSpeed(0);

  attachInterrupt(digitalPinToInterrupt(ENCA_M1), Enncoder_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), Enncoder_m2, RISING);

}


void loop() {
  Serial.print(pos_m1);
  Serial.print(",");
  Serial.print(pos_m2);
  Serial.print(",");
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  velocity_m1 = (pos_m1 - posPrev_m1) / deltaT;
  velocity_m2 = (pos_m2 - posPrev_m2) / deltaT;
  posPrev_m1 = pos_m1;
  posPrev_m2 = pos_m2;
  prevT = currT;

  // Convert count/s to RPM
  float rpm_m1 = velocity_m1 / 990.0 * 60.0;
  float rpm_m2 = velocity_m2 / 990.0 * 60.0;
  v1filt = 0.854 * v1filt + 0.0728 * rpm_m1 + 0.0728 * v1prev;
  v2filt = 0.854 * v2filt + 0.0728 * rpm_m2 + 0.0728 * v2prev;
  v1prev = rpm_m1;
  v2prev = rpm_m2;

  //PI Control M1  setpoin > sum_error > PI controller (kp*e + ki*eintegral) > output to motor > speed and feedback speed to sum_error
  float e1 = v_m1 - v1filt;                   // erroe = setpoin - speedmotor
  eintegral_m1 = eintegral_m1 + e1 * deltaT;  // integral error
  float output_m1 = kp * e1 + ki * eintegral_m1;
  //PI Control M2
  float e2 = v_m2 - v2filt;
  eintegral_m2 = eintegral_m2 + e2 * deltaT;
  float output_m2 = kp * e2 + ki * eintegral_m2;

  set_m1 = (int)output_m1;
  set_m2 = (int)output_m2;

  if (set_m1 > LILIM_PWM)
    set_m1 = LILIM_PWM;

  if (set_m2 > LILIM_PWM)
    set_m2 = LILIM_PWM;

  motor1.setSpeed(set_m1);
  motor2.setSpeed(set_m2);

  Serial.print(v_m1);
  Serial.print(",");
  Serial.print(rpm_m1);
  Serial.print(",");
  Serial.print(v1filt);
  Serial.print(",");
  Serial.print(v_m2);
  Serial.print(",");
  Serial.print(rpm_m2);
  Serial.print(",");
  Serial.print(v2filt);
  Serial.println(",");
  //delay(1);
}

void Enncoder_m1() {
  int b = digitalRead(ENCB_M1);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  } else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_m1 = pos_m1 + increment;
}

void Enncoder_m2() {
  int b = digitalRead(ENCB_M2);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  } else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_m2 = pos_m2 + increment;
}
