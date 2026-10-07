// // =====================================================================
// // ESP32 PINS CONFIGURATION
// // =====================================================================
// #define STEP_PIN     27   // Conveyor Stepper Motor STEP Pin
// #define DIR_PIN      14   // Conveyor Stepper Motor DIR Pin
// #define IR_POWER_PIN 32   // IR Sensor Power (VCC) Pin
// #define IR_OUT_PIN   35   // IR Sensor Signal Output Pin

// // =====================================================================
// // CONVEYOR SPEED & IR SETTINGS
// // 🎯 stepDelay = 24300 dile speed hobe approx 28.0 mm/sec (Python er sathe synced)
// // =====================================================================
// const int stepDelay = 24300; 

// // Ekta object IR par howar somoy jate bar bar trigger na hoy tar cooldown time (ms)
// const unsigned long IR_COOLDOWN_MS = 1500;

// bool conveyorRunning = false;
// bool lastIrState = HIGH;
// unsigned long lastTriggerTime = 0;

// void setup() {
//   Serial.begin(115200);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
  
//   pinMode(IR_POWER_PIN, OUTPUT);
//   digitalWrite(IR_POWER_PIN, HIGH);
  
//   pinMode(IR_OUT_PIN, INPUT);
//   digitalWrite(DIR_PIN, LOW); // Belt ulto dike ghurle HIGH ke LOW kore diben
  
//   Serial.println("ESP32_READY");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     String cmd = Serial.readStringUntil('\n');
//     cmd.trim();
    
//     if (cmd == "START") {
//       conveyorRunning = true;
//       lastIrState = digitalRead(IR_OUT_PIN); 
//       Serial.println("CONVEYOR_STARTED");
//     } else if (cmd == "STOP") {
//       conveyorRunning = false;
//       Serial.println("CONVEYOR_STOPPED");
//     }
//   }
  
//   if (conveyorRunning) {
//     digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(stepDelay);
//     digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(stepDelay);
    
//     bool currentIrState = digitalRead(IR_OUT_PIN);
//     unsigned long now = millis();
    
//     if (currentIrState == LOW && lastIrState == HIGH) {
//       if (now - lastTriggerTime > IR_COOLDOWN_MS) {
//         Serial.println("OBJECT_DETECTED");
//         lastTriggerTime = now;
//       }
//     }
//     lastIrState = currentIrState;
//   }
// }




// =====================================================================
// ESP32 PINS CONFIGURATION
// =====================================================================
#define STEP_PIN     27   // Conveyor Stepper Motor STEP Pin
#define DIR_PIN      14   // Conveyor Stepper Motor DIR Pin
#define IR_POWER_PIN 32   // IR Sensor Power (VCC) Pin 
#define IR_OUT_PIN   35   // IR Sensor Signal Output Pin

// =====================================================================
// CONVEYOR SPEED & IR SETTINGS
// =====================================================================
const int stepDelay = 20000; // 🎯 5 sec e 170 mm jay[cite: 3]

const unsigned long IR_COOLDOWN_MS = 1500;

bool conveyorRunning = false;
bool lastIrState = HIGH;
unsigned long lastTriggerTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
 
  pinMode(IR_POWER_PIN, OUTPUT);
  digitalWrite(IR_POWER_PIN, HIGH);
  
  pinMode(IR_OUT_PIN, INPUT);
  digitalWrite(DIR_PIN, LOW); 
  
  Serial.println("ESP32_READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "START") {
      conveyorRunning = true;
      lastIrState = digitalRead(IR_OUT_PIN); 
      Serial.println("CONVEYOR_STARTED");
    } else if (cmd == "STOP") {
      conveyorRunning = false;
      Serial.println("CONVEYOR_STOPPED");
    }
  }
  
  if (conveyorRunning) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
    
    bool currentIrState = digitalRead(IR_OUT_PIN);
    unsigned long now = millis();
    
    if (currentIrState == LOW && lastIrState == HIGH) {
      if (now - lastTriggerTime > IR_COOLDOWN_MS) {
        Serial.println("OBJECT_DETECTED");
        lastTriggerTime = now;
      }
    }
    lastIrState = currentIrState;
  }
}