#include "Arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define wheel_base 0.175 //17.5 cm to m
#define WheelDimeter 0.068       //68 mm to m

#define ENCA_ML 2
#define ENCB_ML 4
#define PWM_ML 9
#define IN1_ML 7
#define IN2_ML 8

#define ENCA_MR 3
#define ENCB_MR 5
#define PWM_MR 10
#define IN1_MR 11
#define IN2_MR 12

#define Kp 2
#define Ki 7

float eintegral_ml = 0;
float eintegral_mr = 0;
int pos_mr = 0;
int pos_ml = 0;
long prevT = 0;
int posPrev_ml = 0;
int posPrev_mr = 0;
float v_ml = 0;
float v_mr = 0;

float v1filt = 0;
float v1prev = 0;
float v2filt = 0;
float v2prev = 0;

float set_ml = 0;
float set_mr = 0;
float setpoin_ml = 0;
float setpoin_mr = 0;
float vr,vl;
unsigned long lastTime = 0;
const unsigned long interval = 33; // 
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 33; // 30Hz (1000ms / 30)
ros::NodeHandle nh;

// Function prototypes
void readEncoder_ml();
void readEncoder_mr();
void control_ml(int direction, float setpoin_ml);
void control_mr(int direction, float setpoin_mr);
void cmdVelCallback(const geometry_msgs::Twist& msg);
void publishWheelSpeeds();


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);
geometry_msgs::Twist wheel_speeds;
ros::Publisher pub_wheel_speeds("wheel_speeds", &wheel_speeds);

void setup() {
  Serial.begin(115200);
  
  for(int i = 2; i < 6; i++) {
    pinMode(i, INPUT);
  }
  for(int i = 7; i < 13; i++) {
    pinMode(i, OUTPUT);
  }
  
  attachInterrupt(digitalPinToInterrupt(ENCA_ML), readEncoder_ml, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_MR), readEncoder_mr, RISING);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_wheel_speeds);
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    v_ml = (pos_ml - posPrev_ml) / deltaT;
    v_mr = (pos_mr - posPrev_mr) / deltaT;
    posPrev_ml = pos_ml;
    posPrev_mr = pos_mr;
    prevT = currT;

    float rpm_ml = v_ml / 1085 * 60;
    float rpm_mr = v_mr / 1085 * 60;
    v1filt = 0.854 * v1filt + 0.0728 * rpm_ml + 0.0728 * v1prev;
    v2filt = 0.854 * v2filt + 0.0728 * rpm_mr + 0.0728 * v2prev;
    v1prev = rpm_ml;
    v2prev = rpm_mr;

    float error_ml = setpoin_ml - v1filt;
    float error_mr = setpoin_mr - v2filt;
    
    eintegral_ml = eintegral_ml + error_ml * deltaT;
    eintegral_mr = eintegral_mr + error_mr * deltaT;
    
    float set_ml = Kp * error_ml + Ki * eintegral_ml;
    float set_mr = Kp * error_mr + Ki * eintegral_mr;
    int dir_ml = 1;
    int dir_mr = 1;

    if (set_ml < 0) {
      dir_ml = -1;
    }
    if (set_mr < 0) {
      dir_mr = -1;
    }
    
    int setup_ml = (int)fabs(set_ml);
    int setup_mr = (int)fabs(set_mr);
    
    if (setup_ml > 255) {
      setup_ml = 255;
    }
    if (setup_mr > 255) {
      setup_mr = 255;
    }

    control_ml(dir_ml, setup_ml);
    control_mr(dir_mr, setup_mr);
    publishWheelSpeeds() ;
  }
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  vl = msg.linear.x - (msg.angular.z * wheel_base) / 2.0;
  vr = msg.linear.x + (msg.angular.z * wheel_base) / 2.0;
  setpoin_ml = 60/(PI * WheelDimeter) *vl * -1;
  setpoin_mr = 60/(PI * WheelDimeter) *vr * -1;
  nh.spinOnce();
}

void readEncoder_mr() {
  int b = digitalRead(ENCB_MR);
  int increment = (b > 0) ? 1 : -1;
  pos_mr += increment;
}

void readEncoder_ml() {
  int b = digitalRead(ENCB_ML);
  int increment = (b > 0) ? 1 : -1;
  pos_ml += increment;
}

void control_ml(int direction, float setpoin_ml) {
  if (direction == -1) {
    digitalWrite(IN1_ML, 1);
    digitalWrite(IN2_ML, 0);
  } else {
    digitalWrite(IN1_ML, 0);
    digitalWrite(IN2_ML, 1);
  }
  analogWrite(PWM_ML, abs(setpoin_ml));
}

void control_mr(int direction, float setpoin_mr) {
  if (direction == 1) {
    digitalWrite(IN1_MR, 1);
    digitalWrite(IN2_MR, 0);
  } else {
    digitalWrite(IN1_MR, 0);
    digitalWrite(IN2_MR, 1);
  }
  analogWrite(PWM_MR, abs(setpoin_mr));
}


void publishWheelSpeeds() {
  // Populate the Twist message with wheel speeds
  float linear  = (v1filt + v2filt) /2;
  float angular = (v2filt - v1filt) /wheel_base;
  wheel_speeds.linear.x =  -(linear *  PI * WheelDimeter) / 60.0; // convert RPM to m/s vl
  wheel_speeds.angular.z = -(angular * PI * WheelDimeter) / 60.0; // convert RPM to m/s vr
  
  // Publish the wheel speeds
  pub_wheel_speeds.publish(&wheel_speeds);
  nh.spinOnce();
}