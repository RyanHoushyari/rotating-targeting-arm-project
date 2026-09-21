// Ryan Houshyari
// 9/20/2026

// This is a motorized arm that uses closed-loop PID, gravity feedforward, and a secondary PD loop for 
// a momentum assist state to move towards and maintain a specific angle.
// It uses an ESP32-S3, AS5600 magnetic encoder, and DRV8833 motor driver + TT motor to rotate an
// attached, weighted metal straw into various angles.

// libraries
#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

// create helper objects
AS5600 as5600;

// pin definitions
const uint8_t I2C_SDA = 8;
const uint8_t I2C_SCL = 9;
const uint8_t AIN1 = 11;
const uint8_t AIN2 = 12;

// FSM states and momentum variables
enum motorState {STATE_TARGETING, STATE_MOMENTUM}; 
motorState currentState = STATE_TARGETING;
unsigned long stateTimer = 0;
unsigned long momentumCooldown = 0;
int momDir = 0;
bool targetTimerOn = false;
unsigned long targetTimer = 0;

// ESP32 PWM variables
const int pwmFreq = 20000;
const int pwmResolution = 8;
const int ch1 = 0;
const int ch2 = 1;

// gravity feedforward before setup
float kG = 165.0;
int pos = 0;
unsigned long lastPrintTime = 0;

// PID before setup
long prevT = 0;
float ePrev = 0.0;
float rawDedt = 0.0;
float filteredDedt = 0.0;
float eIntegral = 0.0;
unsigned long lastVelocityTime = 0.0;

// adjustable constants
float target = 90.0;
float kp = 2.5;
float kd = 0.7;
float ki = 5.0;

// function that allows users to change the target and PID values while the program runs.
// users enter the shortened variable name followed by a colon and a value, so "t:180.0" or "kp:3.0" for example.
void checkSerialCommand() {
  // checks if serial is available
  if (Serial.available() == 0) {
    return;
  }

  // reads next serial line and assigns to a string, then trims it by removing spaces
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // function returns nothing if no text or colon is found post trimming
  if (cmd.length() == 0) {
    return;
  }
  int colonIdx = cmd.indexOf(':');
  if (colonIdx == -1) {
    return;
  }

  // isolates the variable name and value from the inputted serial text, then applies them into program
  String key = cmd.substring(0, colonIdx);
  float value = cmd.substring(colonIdx + 1).toFloat();
  if (key == "t") { 
    target = value;
    eIntegral = 0;
    currentState = STATE_TARGETING;
    targetTimerOn = true;
    targetTimer = millis();

  } else if (key == "kp") { 
    kp = value; 
  } else if (key == "kd") {
    kd = value; 
  } else if (key == "ki") {
    ki = value; 
  } else if (key == "kg") { 
    kG = value; 
  } else {
    Serial.print("Unknown key: "); Serial.println(key);
    return;
  }

  // displays the change in serial monitor
  Serial.print("Set "); Serial.print(key); Serial.print(" = "); Serial.println(value);
}

void setup() {
  // initialize serial, wire function, and as5600 module
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting ESP32-S3 AS5600 Gravity Feedforward");
  Wire.begin(I2C_SDA, I2C_SCL);
  if (as5600.begin()) {
    Serial.println("AS5600 Sensor Found");
  } else {
    Serial.println("Sensor not detected");
  }
  delay(1000);

  // ledc setup
  ledcSetup(ch1, pwmFreq, pwmResolution);
  ledcSetup(ch2, pwmFreq, pwmResolution);
  
  ledcAttachPin(AIN1, ch1);
  ledcAttachPin(AIN2, ch2);

  //PID setup
  prevT = micros();
}

void loop() {
  // reads position from the as5600 magnetic encoder, ranging values from 0-4096
  pos = as5600.readAngle();
  float currAngleDegrees = 0.0;

  // 8 point angle segmenting:
  // slightly misaligned magnet causes raw as5600 readings to not properly scale to accurate angles,
  // so separate linear equations are made for each 45 degree interval to process angles more correctly.
  const int adcS  = 49;    // 0°
  const int adcSW = 1672;  // 45°
  const int adcW  = 1763;  // 90°
  const int adcNW = 1890;  // 135°
  const int adcN  = 2303;  // 180°
  const int adcNE = 3446;  // 225°
  const int adcE  = 3720;  // 270°
  const int adcSE = 3852;  // 315°

  if (pos >= adcS && pos <= adcSW) {
    currAngleDegrees = 0.0 + ((float)(pos - adcS) * 45.0) / (float)(adcSW - adcS);
  }
  else if (pos > adcSW && pos <= adcW) {
    currAngleDegrees = 45.0 + ((float)(pos - adcSW) * 45.0) / (float)(adcW - adcSW);
  }
  else if (pos > adcW && pos <= adcNW) {
    currAngleDegrees = 90.0 + ((float)(pos - adcW) * 45.0) / (float)(adcNW - adcW);
  }
  else if (pos > adcNW && pos <= adcN) {
    currAngleDegrees = 135.0 + ((float)(pos - adcNW) * 45.0) / (float)(adcN - adcNW);
  }
  else if (pos > adcN && pos <= adcNE) {
    currAngleDegrees = 180.0 + ((float)(pos - adcN) * 45.0) / (float)(adcNE - adcN);
  }
  else if (pos > adcNE && pos <= adcE) {
    currAngleDegrees = 225.0 + ((float)(pos - adcNE) * 45.0) / (float)(adcE - adcNE);
  }
  else if (pos > adcE && pos <= adcSE) {
    currAngleDegrees = 270.0 + ((float)(pos - adcE) * 45.0) / (float)(adcSE - adcE);
  }
  else if (pos > adcSE) {
    currAngleDegrees = 315.0 + ((float)(pos - adcSE) * 45.0) / ((float)(4096 - adcSE) + adcS);
  }
  else { // pos < adcS
    currAngleDegrees = 315.0 + ((float)((4096 - adcSE) + pos) * 45.0) / ((float)(4096 - adcSE) + adcS);
  }

  // ensures values below 0 and above 360 are accounted for in case of an angle read error
  if (currAngleDegrees < 0.0) {
    currAngleDegrees += 360.0;
  }
  if (currAngleDegrees >= 360.0) {
    currAngleDegrees -= 360.0;
  }

  // feedforward and PID variable declarations
  float uFeedforward = 0.0;
  float uPid = 0.0;
  float u = 0.0;
  float e = 0.0;
  float deltaT = 0.0;
  float currAngleRadians = currAngleDegrees * (PI / 180.0);
  float alpha = 0.04;

  checkSerialCommand();

  // time calculation
  long currT = micros();
  deltaT = ((float)(currT - prevT)) / 1000000.0;
  prevT = currT;
  if (deltaT <= 0.0) {
    deltaT = 0.0001;
  }

  // error calculation
  e = target - currAngleDegrees;
  if (e > 180) {
    e -= 360;
  } else if (e < -180) {
    e += 360;
  }
  
  // subtimer and exponential moving average for calculating derivative term, and setting previous error:
  // the derivative term must be measured in slower, filtered intervals so it calculates with more gradual values.
  // this reduces sporadic jumps in readings, which then mess up the derivative calculation
  if (millis() - lastVelocityTime >= 10) {
    // subtimer time calculation
    float dtVel = (float)(millis() - lastVelocityTime) / 1000.0;
    lastVelocityTime = millis();

    // calculates delta e, then adjusts it based off applied angle wraparound
    float de = e - ePrev;
    if (de > 180) {
      de -= 360;
    } else if (de < -180) {
      de += 360;
    }

    // rest of derivative calculation + ema filter
    rawDedt = de / dtVel;
    filteredDedt = alpha * rawDedt + (1 - alpha) * filteredDedt;

    // setting previous error
    ePrev = e;
  }

  switch (currentState) {
    // this STATE_TARGETING case calculates the gravity feedforward, integral term of the loop, and the final PID value,
    // it then adds feedforward and PID together get ready for speed conversion.
    // a check is done here too to switch into STATE_MOMENTUM.
    case STATE_TARGETING: {
      // feedforward calculation
      uFeedforward = kG * sin(currAngleRadians);

      // integral calculations
      // makes sure the integral is calculated when not too far and not too close from target,
      // so that arm stays in angle with no more ki buildup.
      if (abs(e) < 3.5) {
        // friction is more prevalent in the lower 2 quadrants of the arm, so integral calculation resumes there.
        if (currAngleDegrees > 90 && currAngleDegrees < 270) {
          eIntegral = 0;
        }

        //but not as prevalent near the bottom due south, so integral calculation stops there too.
        if (currAngleDegrees < 10 || currAngleDegrees > 350) {
          eIntegral = 0;
        }
      } else if (abs(e) < 100.0) {
        eIntegral = eIntegral + (e * deltaT);
        eIntegral = constrain(eIntegral, -1000.0, 1000.0);
      } else {
        eIntegral = 0;
      }

      // PID and added feedforward value calculations
      uPid = (kp * e) + (kd * filteredDedt) + (ki * eIntegral);
      u = uPid + uFeedforward;

      // momentum check to switch states
      // 1. arm must be near max speed
      // 2. arm must be barely moving
      // 3. arm must not be at target
      // 4. must wait 0.9 seconds after a target change
      bool commandSaturated = abs(u) > 240.0;
      bool notMoving = abs(filteredDedt) < 8.0; //8.0
      bool notAtTarget = abs(e) > 7.0;
      if (targetTimerOn && millis() - targetTimer > 900) {
        targetTimerOn = false;
      }

      if (commandSaturated && notMoving && notAtTarget && millis() > momentumCooldown && !targetTimerOn) {
        currentState = STATE_MOMENTUM;
        stateTimer = millis();

        //checks quadrant the arm is in so it knows the opposite way to go aroung
        if (currAngleDegrees > 0.0 && currAngleDegrees < 180.0) {
          momDir = -1;
        } else {
          momDir = 1;
        }
        break;
      }

      break;
    }

    // this STATE_MOMENTUM case uses its own PD calculations (of PID) to move the arm to 180 degrees in the opposite direction.
    // it occurs when the arm is stuck at an angle near 90 or 270 at max power trying to go up,
    // torque force is at its strongest here so it must go the opposite way and overcome it through momentum.
    case STATE_MOMENTUM: {
      // variable declaration
      const float captureBand = 50.0;
      const float kpMom = 3.5;
      const float kdMom = 0.2;

      // finds shortest distance to 180, and applies a sign change so the arm goes the other, longer way
      float distanceToUpright = 180.0 - currAngleDegrees;
      float eMom = momDir * abs(distanceToUpright);

      // feedforward and PD calculations
      float uFfMom = kG * sin(currAngleRadians);
      u = (kpMom * eMom) + (kdMom * filteredDedt) + uFfMom;

      // switches back to STATE_TARGETING if within 50 degrees of 180,
      // max torque force is overcome so it can switch back to regular PID loop.
      // cooldown in place so it doesn't immediately switch back to STATE_MOMENTUM.
      if (abs(distanceToUpright) < captureBand) {
        currentState = STATE_TARGETING;
        momentumCooldown = millis() + 400;
      }
      break;
    }
  }

  // constraining combines feedforward and PID value + speed conversion
  int speed = constrain((int)u, -255, 255);
  if (speed > 0) {
    ledcWrite(ch1, speed);
    ledcWrite(ch2, 0);
  } else if (speed < 0) {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, abs(speed));
  } else {
    ledcWrite(ch1, 0);
    ledcWrite(ch2, 0);
  }

  // printing status every 0.2 seconds
  if (millis() - lastPrintTime >= 200) {
    Serial.print("ADC Value: "); Serial.print(pos);
    Serial.print(" | Deg: "); Serial.print(currAngleDegrees);
    Serial.print(" | FF Val: "); Serial.print(uFeedforward);
    Serial.print(" | PID Val: "); Serial.print(uPid);
    Serial.print(" | Error: "); Serial.print(e);
    Serial.print(" | Kp: "); Serial.print(kp * e);
    Serial.print(" | Kd: "); Serial.print(kd * filteredDedt);
    Serial.print(" | Filtered Dedt: "); Serial.print(filteredDedt);
    Serial.print(" | Raw Dedt: "); Serial.print(rawDedt);
    Serial.print(" | Ki: "); Serial.print(ki * eIntegral);
    Serial.print(" | Loop cycle "); Serial.print(deltaT * 1000); Serial.print(" ms");
    Serial.print(" | Speed: "); Serial.print(speed);
    Serial.print( "| Case: "); Serial.println(motorState(currentState));
    lastPrintTime = millis();
  }  
}