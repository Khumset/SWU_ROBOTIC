#define PWM_ML 9
#define IN1_ML 7
#define IN2_ML 8

#define PWM_MR 10
#define IN1_MR 11
#define IN2_MR 12


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  for (int i = 7; i < 13; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() {

    control_motor(1,255));  
  }
}

void control_motor(int num_motor, int direction, float set_poin) {
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
}
