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

#define ENCA_M0 2
#define ENCB_M0 4
#define LILIM_PWM 255
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
int pwm = 0;
float eintegral = 0;
float v1filt = 0;
float v1prev = 0;
float v2filt = 0;
float v2prev = 0;

// Configure the motor driver.
CytronMD motor1(PWM_PWM, 6, 7);   // PWM 1A = Pin 3, PWM 1B = Pin 9.
//CytronMD motor2(PWM_PWM, 10, 11); // PWM 2A = Pin 10, PWM 2B = Pin 11.


// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(9600);
  pinMode(ENCA_M0, INPUT);
  pinMode(ENCB_M0, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_M0),Enncoder_m0,RISING);
}



void loop() {

  int pos = 0;
  pos = pos_i;
  Serial.print(pos);
  Serial.print(",");
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/990.0*60.0;
  v1filt = 0.854*v1filt + 0.0728*v1 + 0.0728*v1prev;
  v1prev = v1;
  //v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  //v2Prev = v2;

  float vt = 0;
  float kp = 1;
  float ki = 9;
  float e = vt-v1filt;
  eintegral = eintegral + e*deltaT;

  float u = kp*e + ki*eintegral;
  
  pwm = (int) u;
  if (pwm > LILIM_PWM){
    pwm  = LILIM_PWM;
  }
  motor1.setSpeed(pwm);

  Serial.print(vt);
  Serial.print(",");
  Serial.print(v1);
  Serial.print(",");
  Serial.print(v1filt);
  Serial.println(",");
  //delay(1);

}

void Enncoder_m0(){
  int b = digitalRead(ENCB_M0);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;


}