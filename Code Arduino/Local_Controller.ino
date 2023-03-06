//#include <SimpleKalmanFilter.h>
#include <AutoPID.h>

// PID
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 17.34
#define KI 33.32
#define KD 4.03

//#define KP 17.34
//#define KI 33.32
//#define KD 4.03

//Motor 1
#define Motor1        1
#define MOTOR_A1_PIN  7
#define MOTOR_B1_PIN  8
#define PWM_MOTOR_1   5
#define EN_PIN_1      A0

//Motor 2
#define Motor2        2
#define MOTOR_A2_PIN  4
#define MOTOR_B2_PIN  9
#define PWM_MOTOR_2   6
#define EN_PIN_2      A1

//Encoder
#define encodPinA1      2 
#define encodPinB1      10
#define encodPinA2      3 
#define encodPinB2      11
#define sample_delay    50
#define pulses_per_turn   374

//Dinh nghia bien
volatile long pulseCount1 ,pulseCount2,
              pulses1, pulses2;
unsigned long lasttime, currenttime;

String inString = "", chuoi1, chuoi2;
double v_encoder1=0, v1, vantoc1, output1,
       v_encoder2=0, v2, vantoc2, output2;
int direct1, direct2;
byte moc;

AutoPID myPID1(&v_encoder1, &vantoc1, &output1, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID myPID2(&v_encoder2, &vantoc2, &output2, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
//SimpleKalmanFilter Loc1(1, 1, 0.05);
//SimpleKalmanFilter Loc2(1, 1, 0.05); 

void setup() { 
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  pinMode(encodPinA2, INPUT);
  pinMode(encodPinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encodPinA1), encoder1, FALLING);
  attachInterrupt(digitalPinToInterrupt(encodPinA2), encoder2, FALLING);
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
  myPID1.setTimeStep(5);
  myPID2.setTimeStep(5);
  Serial.begin(9600);
}


void loop() {
  nhandulieu();
  myPID1.run();
  myPID2.run();
  moveMotor(Motor1, direct1, output1);
  moveMotor(Motor2, direct2, output2);
  ReadEncoder();
  //v1 = Loc1.updateEstimate(v_encoder1);
  //v2 = Loc2.updateEstimate(v_encoder2);
}

void ReadEncoder() {
  if (((unsigned long)millis() - lasttime) >= sample_delay) {
    noInterrupts();
    pulses1 = pulseCount1;
    pulses2 = pulseCount2;
    pulseCount1 = 0;
    pulseCount2 = 0;
    interrupts();
    //currenttime = (unsigned long)millis();
    v_encoder1 = abs((pulses1*1000*PI*2)/(((unsigned long)millis()-lasttime)*pulses_per_turn));
    v_encoder2 = abs((pulses2*1000*PI*2)/(((unsigned long)millis()-lasttime)*pulses_per_turn));
    lasttime = (unsigned long)millis();
  }
}

void nhandulieu() {
  while (Serial.available()>0) {
  char inChar = Serial.read();
  if (inChar != '\n') {
    inString += inChar;
  }
  else {
    for (int i=0; i< inString.length(); i++){
      if (inString.charAt(i) == ',') moc = i;
    }
    chuoi1 = inString;
    chuoi2 = inString;
    chuoi1.remove(moc);
    chuoi2.remove(0,moc + 1);
    vantoc1 = chuoi1.toDouble();
    vantoc2 = chuoi2.toDouble();
    if (vantoc1 > 0) direct1 = 1;
    else if (vantoc1 == 0) direct1 = 0;
    else {
      direct1 = 2;
      vantoc1 = abs(vantoc1);
    }
    if (vantoc2 > 0) direct2 = 1;
    else if (vantoc2 == 0) direct2 = 0;
    else {
      direct2 = 2;
      vantoc2 = abs(vantoc2);
    }
    inString = "";
    Serial.println((String)v_encoder1+","+(String)v_encoder2);
  }
  }
}

void moveMotor(uint8_t motor, uint8_t direct,uint8_t pwm) {
  if (motor == Motor1) {
    if (direct == 1) {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    else if (direct == 2) {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_1, pwm);
  }
  else if (motor == Motor2) {
    if (direct == 1) {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    else if (direct == 2) {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_2, pwm);
  }
}

void encoder1() {
  if (PINB & 0b00000100) pulseCount1++;
  else pulseCount1--;
}

void encoder2() {
  if (PINB & 0b00001000) pulseCount2++;
  else pulseCount2--;
}
