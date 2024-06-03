
#define PWM_ML 9
#define IN1_ML 7
#define IN2_ML 8

#define PWM_MR 10
#define IN1_MR 11
#define IN2_MR 12


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  /*for (int i = 2; i < 6; i++) {
    pinMode(i, INPUT);
  }*/
  for (int i = 7; i < 13; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int dir = 1;
  if (Serial.available() > 0) {
    int number = Serial.parseInt();
    Serial.println(number);
    if(number<0){
      dir = -dir ;
    }
    control_motor(1, dir, abs(number));
    control_motor(2, dir, abs(number));
  }
}

//motor1 >> left
//motor2 >> right
void control_motor(int num_motor, int direction, float set_poin) {
  if (num_motor == 1) {
    if (direction == 1) {
      digitalWrite(IN1_ML, 1);
      digitalWrite(IN2_ML, 0);
      analogWrite(PWM_ML, set_poin);
    }
    if (direction == -1) {
      digitalWrite(IN1_ML, 0);
      digitalWrite(IN2_ML, 1);
      analogWrite(PWM_ML, set_poin);
    }
  } else if (num_motor == 2) {
    if (direction == 1) {
      digitalWrite(IN1_MR, 0);
      digitalWrite(IN2_MR, 1);
      analogWrite(PWM_MR, set_poin);
    }
    if (direction == -1) {
      digitalWrite(IN1_MR, 1);
      digitalWrite(IN2_MR, 0);
      analogWrite(PWM_MR, set_poin);
    }
  }
}
