#include <TMC2209.h>
#include <AccelStepper.h>
// MultiStepper.pde
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2009 Mike McCauley
// $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#define Z_STEP_PIN 11
#define Z_DIR_PIN 10
#define Z_ENABLE_PIN 9

#define X_STEP_PIN 27
#define X_DIR_PIN 29
#define X_ENABLE_PIN 26

// Define some steppers and the pins the will useAccelStepper stepper2(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
AccelStepper stepperH(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperV(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

HardwareSerial& serial_stream = Serial3;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 5;
// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;
const int32_t VELOCITY = 1500;
const uint8_t STALL_GUARD_THRESHOLD = 40;

// current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t LOOPS_BEFORE_TOGGLING = 3;
const TMC2209::CurrentIncrement CURRENT_INCREMENT = TMC2209::CURRENT_INCREMENT_8;
const TMC2209::MeasurementCount MEASUREMENT_COUNT = TMC2209::MEASUREMENT_COUNT_1;
const uint32_t COOL_STEP_DURATION_THRESHOLD = 2000;
const uint8_t COOL_STEP_LOWER_THRESHOLD = 0;
const uint8_t COOL_STEP_UPPER_THRESHOLD = 2;
// Instantiate TMC2209
TMC2209 venstreStepper;
TMC2209 hojreStepper;

byte adresse = 0b00000001;

long gennemsnitVenstre;
long gennemsnitHojre;
byte process = 0;
#define ANTAL_MAALINGER 10
#define FART 1000
#define ACCEL 200

#define VENSTRE_STALL_PIN 18
#define HOJRE_STALL_PIN 19

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

const int venstreThreshold = 50;
const int hojreThreshold = 50;
void setup() {
  // initialize serial:
  Serial2.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(VENSTRE_STALL_PIN, INPUT_PULLUP);
  pinMode(HOJRE_STALL_PIN, INPUT_PULLUP);

  hojreStepper.setReplyDelay(5);
  hojreStepper.setup(serial_stream, 115200, 0);
  hojreStepper.setHardwareEnablePin(9);
  hojreStepper.setRunCurrent(RUN_CURRENT_PERCENT);
  hojreStepper.enableStealthChop();
  hojreStepper.setMicrostepsPerStepPowerOfTwo(4);
  hojreStepper.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  hojreStepper.enable();
  hojreStepper.enableInverseMotorDirection(); /*
  hojreStepper.setCoolStepCurrentIncrement(CURRENT_INCREMENT);
  hojreStepper.setCoolStepMeasurementCount(MEASUREMENT_COUNT);
  hojreStepper.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  hojreStepper.enableCoolStep(COOL_STEP_LOWER_THRESHOLD, COOL_STEP_UPPER_THRESHOLD);*/
  hojreStepper.moveAtVelocity(VELOCITY);
  //hojreStepper.moveUsingStepDirInterface();
  hojreStepper.enableAutomaticCurrentScaling();
  Serial.println(hojreStepper.getVersion());
  delay(100);

  venstreStepper.setReplyDelay(5);
  venstreStepper.setup(serial_stream, 115200, 1);
  venstreStepper.setHardwareEnablePin(26);
  venstreStepper.setRunCurrent(RUN_CURRENT_PERCENT);
  venstreStepper.enableStealthChop();
  venstreStepper.setMicrostepsPerStepPowerOfTwo(4);
  venstreStepper.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  venstreStepper.enableStealthChop();
  /*
  venstreStepper.setCoolStepCurrentIncrement(CURRENT_INCREMENT);
  venstreStepper.setCoolStepMeasurementCount(MEASUREMENT_COUNT);
  venstreStepper.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  venstreStepper.enableCoolStep(COOL_STEP_LOWER_THRESHOLD, COOL_STEP_UPPER_THRESHOLD);*/
  venstreStepper.enable();
  venstreStepper.enableInverseMotorDirection();
  venstreStepper.moveAtVelocity(VELOCITY);
  Serial.println(venstreStepper.getVersion());
  //venstreStepper.moveUsingStepDirInterface();
  venstreStepper.enableAutomaticCurrentScaling();
 /* stepperH.setMaxSpeed(FART);
  stepperH.setAcceleration(ACCEL);
  //stepperH.moveTo(-5000);
  stepperV.setMaxSpeed(FART);
  stepperV.setAcceleration(ACCEL);*/
  Serial.println("hallo");
}

bool retning;
long gennemsnit;
long startTid = millis();
float afstand = 20;
bool roterende = false;
float afstande[500];
int processAfstand = 0;

void loop() {
  uint16_t stall_guard_result = hojreStepper.getStallGuardResult();
  delay(1);
  uint16_t stall_guard_result2 = venstreStepper.getStallGuardResult();
  //Serial.print("stall guard result : ");
  Serial.print(stall_guard_result);
  Serial.print(" : ");
  Serial.print(stall_guard_result2);
  Serial.println(" : ");
  afstand = afstand + (11 * (afstand == -1));
  if (startTid + 1500 < millis()) {
    if ((stall_guard_result < hojreThreshold || stall_guard_result2 < venstreThreshold || afstand < 10) && stall_guard_result != 0) {
      roterende = true;
      hojreStepper.disableInverseMotorDirection();
      delay(1);
      venstreStepper.disableInverseMotorDirection();
      speedOp();
      delay(2000);
      speedNed();
      int randomTal = random(-2000, 2000);
      if (randomTal < 0) {
        venstreStepper.enableInverseMotorDirection();
      } else {
        hojreStepper.enableInverseMotorDirection();
      }
      speedOp();
      afstandDelay(random(0,6500));
      speedNed();
      hojreStepper.enableInverseMotorDirection();
      venstreStepper.enableInverseMotorDirection();

      startTid = millis();
      roterende = false;
      for (int i = 0; i < processAfstand; i++) {
        Serial.println(afstande[i]);
      }
      //Serial.println(processAfstand);
      speedOp();
    }
  }
  readAfstand();
}

void readAfstand() {

  if (stringComplete) {
    //Serial.println(inputString);
    // clear the string:
    stringComplete = false;
    afstand = inputString.toFloat();
    //Serial.println(afstand);
    inputString = "";
    if (roterende == true) {
      afstande[processAfstand] = afstand;
      processAfstand++;
    } else {
      processAfstand = 0;
    }
  }
  while (Serial2.available()) {
    // get the new byte:
    char inChar = (char)Serial2.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void speedOp() {
  for (int i = 0; i <= VELOCITY; i += 2) {
    readAfstand();
    venstreStepper.moveAtVelocity(i);
    afstandDelay(1);
    hojreStepper.moveAtVelocity(i);
    afstandDelay(1);
  }
}

void speedNed() {
  for (int i = VELOCITY; i >= 0; i = i - 2) {
    venstreStepper.moveAtVelocity(i);
    afstandDelay(1);
    hojreStepper.moveAtVelocity(i);
    afstandDelay(1);
  }
}
void afstandDelay(int forsinkelse) {
  long tid = millis();
  while (tid + forsinkelse > millis()) {
    readAfstand();
  }
}
