#include <Arduino.h>

// Motor pins
#define mr1_1 PB9
#define mr1_2 PB5
#define pwm1 PA1

#define mr2_1 PB3
#define mr2_2 PA15
#define pwm2 PA2

#define mr3_1 PB15
#define mr3_2 PB14
#define pwm3 PA3

#define mr4_1 PB13
#define mr4_2 PB12
#define pwm4 PB1

// Encoder pins
const uint8_t encoder1PinA = PB8;
const uint8_t encoder1PinB = PB7;
/*
const uint8_t encoder2PinA = PB4; 
const uint8_t encoder2PinB = PB6;*/

// Encoder values

volatile int encoder1Value = 0;
//volatile int encoder2Value = 0;

// PID parameters
float Kp = 2.0;
float Ki = 5.0;
float Kd = 1.0;

// Setpoint and input/output
int setpoint = 100; // Desired speed
float input1, input2;
float output1, output2;

// Function declarations
void encoder1ISR();
//void encoder2ISR();
float computePID(float input, int setpoint);

void setup() {
  Serial.begin(115200);

  pinMode(mr1_1, OUTPUT);
  pinMode(mr1_2, OUTPUT);
  pinMode(pwm1, OUTPUT);

  pinMode(mr2_1, OUTPUT);
  pinMode(mr2_2, OUTPUT);
  pinMode(pwm2, OUTPUT);

  pinMode(mr3_1, OUTPUT);
  pinMode(mr3_2, OUTPUT);
  pinMode(pwm3, OUTPUT);

  pinMode(mr4_1, OUTPUT);
  pinMode(mr4_2, OUTPUT);
  pinMode(pwm4, OUTPUT);

  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  /*pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);*/

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, FALLING);
}

void encoder1ISR() {
  if (digitalRead(encoder1PinB)) {
    encoder1Value++;
  } else {
    encoder1Value--;
  }
}

/*
void encoder2ISR() {
  if (digitalRead(encoder2PinB)) {
    encoder2Value++;
  } else {
    encoder2Value--;
  }
}*/

void front() {
  digitalWrite(mr1_1, LOW);
  digitalWrite(mr1_2, HIGH);
  digitalWrite(mr2_1, LOW);
  digitalWrite(mr2_2, HIGH);
  digitalWrite(mr3_1, LOW);
  digitalWrite(mr3_2, HIGH);
  digitalWrite(mr4_1, LOW);
  digitalWrite(mr4_2, HIGH);

  analogWrite(pwm1,100);
  analogWrite(pwm2,100);
  analogWrite(pwm3,100);
  analogWrite(pwm4,100);
}

void loop() {
  // Calculate speed from encoder values
  //input1 = encoder1Value;
  //input2 = encoder2Value;

  // Calculate PID output
  //output1 = computePID(input1, setpoint);
  //output2 = computePID(input2, setpoint);

  // Apply PWM values to motors
 /* analogWrite(pwm1, constrain(output1, 0, 255));
  analogWrite(pwm2, constrain(output2, 0, 255));
  analogWrite(pwm3, constrain(output1, 0, 255));
  analogWrite(pwm4, constrain(output2, 0, 255));*/

  // Print encoder values
  Serial.print("Encoder 1: ");
  Serial.println(encoder1Value);
  //Serial.print(" | Encoder 2: ");
  //Serial.println(encoder2Value);

  // Reset encoder values
  //encoder1Value = 0;
  //encoder2Value = 0;

  // Move forward
  front();

  delay(1);
}


/*
float computePID(float input, int setpoint) {
  static float lastError = 0;
  static float integral = 0;

  float error = setpoint - input;
  integral += error;
  float derivative = error - lastError;
  lastError = error;

  return Kp * error + Ki * integral + Kd * derivative;
}
*/