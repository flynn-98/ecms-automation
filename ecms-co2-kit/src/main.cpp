#include <Servo.h>
#include <AccelStepper.h>

#include <DFRobot_PH.h>
#include <EEPROM.h>

// Pins for pump stepper motors, see https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide/all
const int PUMP_1_STEP = 2;
const int PUMP_1_DIR = 3;

const int PUMP_2_STEP = 4;
const int PUMP_2_DIR = 5;

const int PUMP_3_STEP = 6;
const int PUMP_3_DIR = 7;

// Pump 1 = Electrolyte into Cell, Pump 2 = Electrolyte out of Cell, Pump 3 = Cleaning Solution into Cell

// Pins for 4th stepper motor
const int PUMP_4_STEP = 8;
const int PUMP_4_DIR = 9;

// Define Arduino pins for each function
const int SERVO_PIN = 10;

// Define relay pin
const byte RELAY_PIN = A0;

const byte PH_PIN = A1;
const byte ANALOG_2 = A2;

// Parameters for Valve (Servo)
const int servoClosed = 0;
const int servoOpen = 90;

// Motor speed and acceleration parameters, stepper motors have 200 steps / revolution.
// Microsteps (per step) used for increased positional accuracy and smoother stepping
const float STEPS_REV = 200.0;
const float MICROSTEPS = 4.0;
const float GEAR_RATIO = 1.0;

const float PUMP_SPEED = 600.0 * MICROSTEPS * GEAR_RATIO; //microsteps/s
const float ECMS_SPEED = 300.0 * MICROSTEPS * GEAR_RATIO; //microsteps/s
const float MAX_ACCEL = 200.0 * MICROSTEPS * GEAR_RATIO; //microsteps/s2

// For Pump stepper motor
const float ML_REV = 0.1; //ml/revs

float voltage;
float phValue; 
float temperature = 25;

DFRobot_PH ph;

// Define steppers with pins (STEP, DIR)
AccelStepper PUMP_1(AccelStepper::DRIVER, PUMP_1_STEP, PUMP_1_DIR); 
AccelStepper PUMP_2(AccelStepper::DRIVER, PUMP_2_STEP, PUMP_2_DIR);
AccelStepper PUMP_3(AccelStepper::DRIVER, PUMP_3_STEP, PUMP_3_DIR); 
AccelStepper PUMP_4(AccelStepper::DRIVER, PUMP_4_STEP, PUMP_4_DIR);

// Create servo instance
Servo valve;

const float motorDir = 1;

String action;

unsigned long duration;
float volume = 0;

// put function declarations here:
void relayOn();
void relayOff();
long volToSteps(float vol);
void closeValve();
void openValve();

void releaseCO2(unsigned long duration);
void addChemical(float vol);
void addWater(float vol);
void transferToECMS(float vol);
float getPH();

void setup() {
  // put your setup code here, to run once:
  pinMode(PUMP_1_STEP, OUTPUT);
  pinMode(PUMP_1_DIR, OUTPUT);

  pinMode(PUMP_2_STEP, OUTPUT);
  pinMode(PUMP_2_DIR, OUTPUT);

  pinMode(PUMP_3_STEP, OUTPUT);
  pinMode(PUMP_3_DIR, OUTPUT);

  pinMode(PUMP_4_STEP, OUTPUT);
  pinMode(PUMP_4_DIR, OUTPUT);

  // Set motor speeds / acceleration
  PUMP_1.setAcceleration(MAX_ACCEL);
  PUMP_1.setMaxSpeed(PUMP_SPEED);

  PUMP_2.setAcceleration(MAX_ACCEL);
  PUMP_2.setMaxSpeed(PUMP_SPEED);

  PUMP_3.setAcceleration(MAX_ACCEL);
  PUMP_3.setMaxSpeed(PUMP_SPEED);

  PUMP_4.setAcceleration(MAX_ACCEL);
  PUMP_4.setMaxSpeed(ECMS_SPEED);

  pinMode(SERVO_PIN, OUTPUT);
  valve.attach(SERVO_PIN);

  pinMode(RELAY_PIN, OUTPUT);

  Serial.begin(9600);
  //ph.begin();

  valve.write(servoClosed);
  
  closeValve();
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("CO2 Kit Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Main code here, to run repeatedly on a loop 
    delay(500);

    // Wait until data received from PC, via Serial (USB)
    if (Serial.available() > 0) {
        // data structure to receive = action(var1, var2..)

        // Read until open bracket to extract action, continue based on which action was requested
        action = Serial.readStringUntil('(');

        if (action == "releaseCO2") {
            // Extract variables spaced by commas, then last variable up to closed bracket
            duration = Serial.readStringUntil(')').toInt();
            
            // Call action using received variables
            releaseCO2(duration);
        }
        else if (action == "addChemical") {
            volume = Serial.readStringUntil(')').toFloat();
            
            addChemical(volume);
        }
        else if (action == "addWater") {
            volume = Serial.readStringUntil(')').toFloat();
            
            addWater(volume);
        }
        else if (action == "transferToECMS") {
            volume = Serial.readStringUntil(')').toFloat();
            
            transferToECMS(volume);
        }
        else if (action == "getPH") {
            volume = Serial.readStringUntil(')').toFloat();
            
            getPH();
        }
        else {
            // Report back to PC if confused
            Serial.println("Unknown command");
        }
    }
}

long volToSteps(float vol) {
    return floor(motorDir * MICROSTEPS * STEPS_REV * vol * GEAR_RATIO / ML_REV);
};

void relayOn() {
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
};

void relayOff() {
    digitalWrite(RELAY_PIN, LOW);
};

// put function definitions here:
void closeValve() {
    valve.write(servoClosed);
}

void openValve() {
    valve.write(servoOpen);
}

void releaseCO2(unsigned long duration) {
    openValve();
    delay(floor(duration * 1000)); // Convert seconds to milliseconds
    closeValve();

    Serial.println("Valve opened for " + String(duration) + "s");
}

void addChemical(float vol) {
    relayOn();

    // No limits for Pump
    PUMP_1.move(volToSteps(vol));

    // Run until complete
    PUMP_1.runToPosition();

    // Report back to PC
    Serial.println("Pumping complete");
    relayOff();
};

void addWater(float vol) {
    relayOn();

    // No limits for Pump
    PUMP_2.move(volToSteps(vol));

    // Run until complete
    PUMP_2.runToPosition();

    // Report back to PC
    Serial.println("Pumping complete");
    relayOff();
};

void transferToECMS(float vol) {
    relayOn();

    // No limits for Pump
    PUMP_3.move(volToSteps(vol));

    // Run until complete
    PUMP_3.runToPosition();

    // Report back to PC
    Serial.println("Pumping complete");
    relayOff();
};

float getPH() {
    voltage = analogRead(PH_PIN);
    voltage = voltage * (3300 / 1023.0); // Convert to voltage
    phValue = ph.readPH(voltage, temperature);

    Serial.print(phValue);
}