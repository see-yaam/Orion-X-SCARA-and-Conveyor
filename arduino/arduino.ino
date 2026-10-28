#include <AccelStepper.h>
#include <Servo.h>

/*
  ⚡ SCARA Main Firmware - Upload this to COM9 🎯
*/

#define MOTOR_INTERFACE_TYPE 1
const int STEP_PIN_1 = 8;  const int DIR_PIN_1  = 9;
const int STEP_PIN_2 = 10; const int DIR_PIN_2  = 11;
const int STEP_PIN_3 = 12; const int DIR_PIN_3  = 13;
const int STEP_PIN_4 = 6;  const int DIR_PIN_4  = 7;
const int JAW_SERVO_PIN = 5;

// 🎯 Limit Switch Pins & Power (SWAPPED HERE)
const int LIMIT_POWER_PIN = 4; // Acts as 5V source
const int LIMIT_PIN_1 = 3;     // Elbow 1 Limit Switch (Swapped to 3)
const int LIMIT_PIN_2 = 2;     // Elbow 2 Limit Switch (Swapped to 2)

// 🎯 Homing Speeds
const float HOMING_SPEED_1 = 150.0; 
const float HOMING_SPEED_2 = 150.0;

const float STEPS_PER_DEGREE = 3200.0 / 360.0;
const float MOTOR_1_MULTIPLIER = 3.0;
const float MOTOR_2_MULTIPLIER = 3.0;
const float MOTOR_3_MULTIPLIER = 1.0;
const float MOTOR_4_MULTIPLIER = 3.0;

const float MAX_SPEED_1 = 1000.0; const float ACCEL_1 = 500.0;
const float MAX_SPEED_2 = 1000.0; const float ACCEL_2 = 500.0;
const float MAX_SPEED_3 = 1000.0; const float ACCEL_3 = 500.0;
const float MAX_SPEED_4 = 600.0;  const float ACCEL_4 = 300.0;

const float JAW_OPEN_DEG   = 0.0;
const float JAW_CLOSED_DEG = 110.0; 

const int POST_MOVE_DELAY_MS = 250;

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, STEP_PIN_3, DIR_PIN_3);
AccelStepper stepper4(MOTOR_INTERFACE_TYPE, STEP_PIN_4, DIR_PIN_4);
Servo jawServo;

const byte SERIAL_BUFFER_SIZE = 96;
char serialBuffer[SERIAL_BUFFER_SIZE];

long degreeToSteps(float degrees, float multiplier) {
  return lround(degrees * STEPS_PER_DEGREE * multiplier);
}

float clampJawAngle(float jaw) {
  if (jaw < JAW_OPEN_DEG) return JAW_OPEN_DEG;
  if (jaw > JAW_CLOSED_DEG) return JAW_CLOSED_DEG;
  return jaw;
}

void configureStepper(AccelStepper &stepper, float maxSpeed, float acceleration) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
}

bool parseMoveCommand(char *line, float &j1, float &j2, float &z, float &jaw) {
  char *token = strtok(line, ",");
  if (token == NULL || strcmp(token, "MOVE") != 0) return false;
  token = strtok(NULL, ","); if (token == NULL) return false; j1 = atof(token);
  token = strtok(NULL, ","); if (token == NULL) return false; j2 = atof(token);
  token = strtok(NULL, ","); if (token == NULL) return false; z = atof(token);
  token = strtok(NULL, ","); if (token == NULL) return false; jaw = atof(token);
  return true;
}

void executeMove(float j1, float j2, float z, float jaw) {
  float j4Deg = j2 - j1;
  long target1 = degreeToSteps(j1, MOTOR_1_MULTIPLIER);
  long target2 = degreeToSteps(j2, MOTOR_2_MULTIPLIER);
  long target3 = degreeToSteps(z, MOTOR_3_MULTIPLIER);
  long target4 = degreeToSteps(j4Deg, MOTOR_4_MULTIPLIER);
  
  stepper1.moveTo(target1);
  stepper2.moveTo(target2);
  stepper3.moveTo(target3);
  stepper4.moveTo(target4);
  
  while (
    stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 ||
    stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0
  ) {
    stepper1.run(); stepper2.run(); stepper3.run(); stepper4.run();
  }
  
  float safeJaw = clampJawAngle(jaw);
  jawServo.write((int)safeJaw);
  delay(POST_MOVE_DELAY_MS);
  Serial.println("DONE");
}

void setup() {
  Serial.begin(115200);
  
  // 🎯 Make Pin 4 output HIGH (5V) for the first limit switch
  pinMode(LIMIT_POWER_PIN, OUTPUT);
  digitalWrite(LIMIT_POWER_PIN, HIGH);

  // 🎯 Standard INPUT 
  pinMode(LIMIT_PIN_1, INPUT);
  pinMode(LIMIT_PIN_2, INPUT);

  configureStepper(stepper1, MAX_SPEED_1, ACCEL_1);
  configureStepper(stepper2, MAX_SPEED_2, ACCEL_2);
  configureStepper(stepper3, MAX_SPEED_3, ACCEL_3);
  configureStepper(stepper4, MAX_SPEED_4, ACCEL_4);
  
  jawServo.attach(JAW_SERVO_PIN);
  jawServo.write((int)JAW_OPEN_DEG);

  Serial.println("READY"); 
}

void loop() {
  if (!Serial.available()) return;
  
  size_t length = Serial.readBytesUntil('\n', serialBuffer, SERIAL_BUFFER_SIZE - 1);
  serialBuffer[length] = '\0';
  for (byte i = 0; i < length; i++) {
    if (serialBuffer[i] == '\r') {
      serialBuffer[i] = '\0';
      break;
    }
  }

  // 🎯 Run Homing only when Python sends "HOME_LIMIT"
  if (strcmp(serialBuffer, "HOME_LIMIT") == 0) {
    stepper1.setSpeed(HOMING_SPEED_1);
    stepper2.setSpeed(HOMING_SPEED_2);
    
    bool j1_homed = false;
    bool j2_homed = false;

    while (!j1_homed || !j2_homed) {
      if (!j1_homed) {
        if (digitalRead(LIMIT_PIN_1) == LOW) { 
          j1_homed = true; 
          stepper1.setSpeed(0); 
        } else {
          stepper1.runSpeed(); 
        }
      }
      if (!j2_homed) {
        if (digitalRead(LIMIT_PIN_2) == LOW) { 
          j2_homed = true; 
          stepper2.setSpeed(0); 
        } else {
          stepper2.runSpeed(); 
        }
      }
    }
    
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0); 
    
    Serial.println("HOMED_DONE");
  } else {
    // Normal Move Commands
    float j1 = 0.0, j2 = 0.0, z = 0.0, jaw = 0.0;
    if (parseMoveCommand(serialBuffer, j1, j2, z, jaw)) {
      executeMove(j1, j2, z, jaw);
    } else {
      Serial.println("ERROR");
    }
  }
  
  while (Serial.available() > 0) {
    Serial.read();
  }
}
