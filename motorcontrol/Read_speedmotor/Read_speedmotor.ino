#define ENCA_ML 2
#define ENCB_ML 4
#define PWM_ML 9
#define IN1_ML 7
#define IN2_ML 8

int posPrev_m = 0;
long prevT = 0;
int pos_m = 0;
float v_m = 0;
float v1filt = 0;
float v1prev = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 2; i < 6; i++) {
    pinMode(i, INPUT);
  }
  for (int i = 7; i < 13; i++) {
    pinMode(i, OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(ENCA_ML), readEncoder_m, RISING);
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
    control_motor(dir, abs(number));
  }
  
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  v_m = (pos_m - posPrev_m) / deltaT;
  posPrev_m = pos_m;
  prevT = currT;
  float rpm_m = v_m / 1085 * 60;
  v1filt = 0.854 * v1filt + 0.0728 * rpm_m + 0.0728 * v1prev;
  v1prev = rpm_m;
  
  Serial.print(rpm_m);
  Serial.print(",");
  Serial.print(v1filt);
  Serial.print(",");
  Serial.println();
}

void readEncoder_m() {
  int b = digitalRead(ENCB_ML);
  int increment = (b > 0) ? 1 : -1;
  pos_m += increment;
}

//motor1 >> left
//motor2 >> right
void control_motor(int direction, float set_poin) {
    if (direction == 1) {
      digitalWrite(IN1_ML, 1);
      digitalWrite(IN2_ML, 0);
      analogWrite(PWM_ML, set_poin);
    }
    else if (direction == -1) {
      digitalWrite(IN1_ML, 0);
      digitalWrite(IN2_ML, 1);
      analogWrite(PWM_ML, set_poin);
    }

}
