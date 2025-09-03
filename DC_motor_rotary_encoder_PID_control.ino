#include "DualG2HighPowerMotorShield.h"

DualG2HighPowerMotorShield24v14 md;

// ---------------- Motor encoder wiring ----------------
// A channel on interrupt-capable pin, B on regular digital pin.
const int encA = 3;  // yellow – interrupt pin
const int encB = 5;  // green  – read inside ISR

// Counts per output-shaft revolution (motor encoder 16 CPR × 70:1 gearbox).
const long ROT_STEP = 1120;

// ---------------- Rotary (hand) encoder wiring ----------------
const int clk = 8;   // rotary A (interrupt pin)
const int dt  = 4;   // rotary B
const int sw  = 12;  // rotary pushbutton

// global variables needed for position control
unsigned long prevT = 0;
volatile long setPoint = 0;
volatile long motorPosition = 0;
volatile long prevMotorPosition = 0;

float controlSignal = 0;
float error = 0;
float eint = 0;
float edot = 0;
float prevError = 0;

float kp = 10;
float kd = 0;
float ki = 0.0;


const int PWM_MAX = 380;
const int DEADBAND = 20; // was 19

// global variables for rotary encoder
unsigned long currRotTime = 0;
volatile long rotaryValue = 0;
unsigned long rotaryTime = 0;



void setup() {
  Serial.begin(115200);

  md.init();
  md.enableDrivers();
  delay(10);

  // -------- Motor encoder pins & ISR --------
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA), checkMotorEncoder, RISING);

  // -------- Rotary encoder pins & ISR --------
  pinMode(clk, INPUT_PULLUP);
  pinMode(dt,  INPUT_PULLUP);
  pinMode(sw,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(clk), checkRotaryEncoder, RISING);

  prevT = micros();
  rotaryTime = millis();
  
  startTime = millis();


}

void loop() {

  checkRotaryButton();
  calcPID();
  driveMotor((int)controlSignal);
  
  static unsigned long t0 = millis();
  if (millis() - t0 < 20000UL){
    printValues();
  }
  

}

// Halt if the driver reports a fault (over-current, etc.).
void stopIfFault() {
  if (md.getM1Fault()) {
    md.disableDrivers();
    delay(1);
    Serial.println("M1 Fault");
    while (1) { /* halt */ }
  }
}

// motor encoder ISR
void checkMotorEncoder() {

  if(digitalRead(encB) == LOW){
    motorPosition ++; // CCW
  }
  else{
    motorPosition --; // CW
  }
}
  
void calcPID(){
  unsigned long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;

  // evaluate the control signal
  error = (float)(setPoint - motorPosition);
  edot = (error - prevError)/deltaT;
  eint +=  error*deltaT;
  prevError = error;

  controlSignal = (kp*error) + (kd*edot) + (ki*eint);
  

}

void driveMotor(int cmd){
  
  if (cmd > PWM_MAX){
    cmd = PWM_MAX;
  }

  if (cmd < -PWM_MAX){
    cmd = -PWM_MAX;
  }

  if (cmd > -DEADBAND && cmd < DEADBAND) cmd = 0;


  md.setM1Speed(cmd);
  //stopIfFault();

}

// rotary encoder ISR
void checkRotaryEncoder(){
  static unsigned long prevRotTime = 0;
  currRotTime = micros();

  // debouncing
  if (currRotTime - prevRotTime > 3000){
    prevRotTime = currRotTime;
    if (digitalRead(dt) == LOW){
      rotaryValue -= ROT_STEP; 
    }
    else{
      rotaryValue += ROT_STEP;
    }
  }
  
}

void checkRotaryButton(){
  bool pressed = (digitalRead(sw) == LOW); 
  if (pressed && millis() - rotaryTime > 1000) {
    setPoint = rotaryValue; // set new target
    rotaryTime = millis();
  }
}

void printValues(){
  float ref = (float)setPoint;
  float y = (float)motorPosition;

  Serial.print(ref,5);
  Serial.print('\t');
  Serial.println(y,5);
}




