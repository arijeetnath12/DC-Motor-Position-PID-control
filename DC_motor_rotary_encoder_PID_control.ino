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

// ---------------- Control & state ----------------
// Position target in *motor counts* (multiple of ROT_STEP when using knob).
volatile long setPoint = 0;
// Measured position in *motor counts* (updated by motor encoder ISR).
volatile long motorPosition = 0;

// PID gains (position control, runs every loop).
float kp = 1;
float ki = 0.0;
float kd = 0.00;

// Controller output (mapped to Pololu speed command in driveMotor).
float controlSignal = 0;

// PID bookkeeping
uint32_t prevTime = 0;   // for delta-t (micros)
float    prevError = 0;  // for derivative term
float    errorIntegral = 0; // for integral term
const int PWM_MAX = 300;
const int DEADBAND = 12; // was 19

// Rotary encoder state
volatile long rotaryValue = 0;  // “dialed” position target in counts
unsigned long rotaryTime = 0;   // debounce / rate limit for the pushbutton

void setup() {
  Serial.begin(115200);

  // Pololu driver set-up (enable MOSFETs). The 10 ms pause helps after power-up.
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

  // Initialize timing bases so first delta-t isn’t huge.
  prevTime   = micros();
  rotaryTime = millis();
}

void loop() {
  // Position PID -> controlSignal
  calculatePID();

  // Map controlSignal to Pololu speed command and send to driver.
  driveMotor();

  // If pushbutton is pressed (with rate limit), latch knob position into setPoint.
  checkRotaryButton();

  // Periodic serial output for monitoring/plotting.
  printValues();
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

// -------- Motor encoder ISR --------
// Triggered on rising edge of encoder A. Direction from B:
//  - If B == LOW at A rising: CCW -> +counts
//  - If B == HIGH at A rising: CW  -> −counts
// NOTE: If your observed direction is flipped, swap the ++/-- below.
void checkMotorEncoder() {
  if (digitalRead(encB) == LOW) {
    motorPosition++;   // CCW
  } else {
    motorPosition--;   // CW
  }
}

void calculatePID() {
  // Compute loop delta-t in seconds (from micros()).
  uint32_t currTime = micros();
  float deltaT = (currTime - prevTime) * 1e-6f;
  if (deltaT <= 0) return;   // guard for timer wrap or identical timestamps
  prevTime = currTime;

  // Position error (counts). Do math in float after the integer difference
  // to avoid precision loss on close values.
  long  e_counts = setPoint - motorPosition;
  float error    = (float)e_counts;

  // Derivative term (counts/s).
  float edot = (error - prevError) / deltaT;
  prevError = error;

  // PD first
  float u_pd = kp*error + kd*edot;

  // predict current unsaturated output
  float u_unsat = u_pd + ki*errorIntegral;

  // if saturated and the error will push us further into the rail, skip integrating
  float u_clamped = constrain(u_unsat,-PWM_MAX,PWM_MAX);
  bool sat_hi  = (u_clamped >= PWM_MAX) && (error > 0);
  bool sat_lo = (u_clamped <= -PWM_MAX ) && (error < 0);
  bool block_int = sat_hi || sat_lo;

  if (!block_int){
    errorIntegral += error*deltaT;
    u_unsat = u_pd + ki*errorIntegral;
  }
  

  // PID sum -> controlSignal (will be constrained and deadbanded in driveMotor()).
  controlSignal = u_unsat;
}

void driveMotor() {
  // Pololu command is −400..+400, but we use a narrower range initially.

  int cmd = (int)constrain(controlSignal, -PWM_MAX, PWM_MAX);
  int db;
  long err = setPoint - motorPosition;

  if (abs(err) < (ROT_STEP/4)){
    db = 10;
  }
  else{
    db = DEADBAND;
  }

  // Deadband compensation: jump over low-PWM region where motor won’t move.
  if (cmd > 0 && cmd < db)      cmd = db;
  else if (cmd < 0 && cmd > -db) cmd = -db;

  md.setM1Speed(cmd);
  stopIfFault();
}

// -------- Rotary encoder ISR --------
// Simple 1× decoding on CLK rising with ~3 ms debounce.
// Direction from DT: DT LOW => negative step; DT HIGH => positive step.
// Each step moves the target by one output-shaft revolution (±ROT_STEP counts).
void checkRotaryEncoder() {
  static uint32_t prevRotTime = 0;
  uint32_t currRotTime = micros();

  // Debounce window: ignore edges that arrive <3 ms apart.
  if (currRotTime - prevRotTime < 3000) return;
  prevRotTime = currRotTime;

  if (digitalRead(dt) == LOW) {
    rotaryValue -= ROT_STEP;
  } else {
    rotaryValue += ROT_STEP;
  }
}

// Latch rotaryValue into setPoint when the button is pressed,
// with a 1 s guard to avoid repeated latching while held.
void checkRotaryButton() {
  bool pressed = (digitalRead(sw) == LOW);  // INPUT_PULLUP -> LOW when pressed
  if (pressed && millis() - rotaryTime > 1000) {
    setPoint   = rotaryValue;   // new target (in counts)
    rotaryTime = millis();

    // reset integral/derivative memory
    errorIntegral = 0.0f;
    prevError = 0.0f;
  }
}

// Periodic print for Arduino Serial Plotter / MATLAB, scaled to revolutions.
// Uses tab '\t' between series names to show multiple traces.
void printValues() {

  // Convert counts to revolutions (absolute value for a tidy plot).
  float ref = fabs((float)setPoint      / (float)ROT_STEP);
  float y   = fabs((float)motorPosition / (float)ROT_STEP);

  Serial.print(ref, 3);
  Serial.print('\t');
  Serial.println(y, 3);
}
