#include <Servo.h>
#include <meArm.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(2, 3); // RX=2, TX=3

// Contact sensor on pin 9
// HIGH (5V) = Contact made = NO object in gripper → FAIL
// LOW (0V) = No contact = Object in gripper → SUCCESS
#define CONTACT_SENSOR_PIN 9

// Sensor check parameters
#define SENSOR_CHECK_DURATION 50  // Check sensor for 50ms
#define SENSOR_SAMPLE_INTERVAL 5   // Sample every 5ms

meArm arm(145, 49, -PI/4, PI/4,
          135, 35, PI/4, 3*PI/4,
          135, 45, PI/4, -PI/4,
          14, 145, PI/2, 0);

// Drop zones
const float UNKNOWN_DROP_X = -150;
const float UNKNOWN_DROP_Y = -10;
const float UNKNOWN_DROP_Z = 0;

const float CLASS_DROP_X = 150;
const float CLASS_DROP_Y = 10;
const float CLASS_DROP_Z = 0;

const float HOME_X = 0;
const float HOME_Y = 90;
const float HOME_Z = 20;

// Safe height for navigation and failed picks
const float SAFE_TRAVEL_HEIGHT = 70;
const float FAIL_SAFE_HEIGHT = 60;

// Safety limits
const float MIN_X = -130;
const float MAX_X = 130;
const float MIN_Y = 0;
const float MAX_Y = 225;
const float MIN_Z = -30;
const float MAX_Z = 80;

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  espSerial.begin(38400);
  
  // DON'T configure pin 9 - just read voltage directly
  pinMode(LED_BUILTIN, OUTPUT);
  
  delay(500);
  Serial.println("ARDUINO START - CONTINUOUS PICKING MODE");
  
  arm.begin(5, 6, 11, 10);
  delay(100);
  Serial.println("SERVOS OK");
  
  goHome();
  Serial.println("READY - Will pick continuously without home");
}

void loop() {
  // Read contact sensor and update LED
  int pinState = digitalRead(CONTACT_SENSOR_PIN);
  digitalWrite(LED_BUILTIN, !pinState);

  // Handle ESP communication
  while (espSerial.available() > 0) {
    char c = espSerial.read();
    if (c == '<') {
      inputBuffer = "";
    } else if (c == '>') {
      if (inputBuffer.length() > 0) {
        processSingleObject(inputBuffer);
      }
      inputBuffer = "";
    } else if (inputBuffer.length() < 200) {
      inputBuffer += c;
    }
  }
  delay(10);
}

void processSingleObject(String cmd) {
  Serial.print("RECV: ");
  Serial.println(cmd);

  // Parse: name,x,y
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  
  if (firstComma == -1 || secondComma == -1) {
    Serial.println("ERR:FORMAT");
    espSerial.println("FAIL");
    return;
  }

  String name = cmd.substring(0, firstComma);
  String xStr = cmd.substring(firstComma + 1, secondComma);
  String yStr = cmd.substring(secondComma + 1);

  name.trim(); 
  xStr.trim(); 
  yStr.trim();

  float robot_x = xStr.toFloat();   // Already in robot base frame
  float robot_y = yStr.toFloat();

  Serial.print("Picking: ");
  Serial.print(name);
  Serial.print(" @ (");
  Serial.print(robot_x);
  Serial.print(",");
  Serial.print(robot_y);
  Serial.println(")");

  float robot_z = -30;

  if (!isPositionSafe(robot_x, robot_y, robot_z)) {
    Serial.println("ERR:UNSAFE");
    espSerial.println("FAIL");
    return;
  }

  // Attempt pick
  bool pickSuccess = pickAndPlace(name, robot_x, robot_y, robot_z);
  
  if (pickSuccess) {
    Serial.println("✓ PICK SUCCESS");
    espSerial.println("OK");
  } else {
    Serial.println("✗ PICK FAILED");
    espSerial.println("FAIL");
  }
}

bool isPositionSafe(float x, float y, float z) {
  if (x < MIN_X || x > MAX_X || 
      y < MIN_Y || y > MAX_Y || 
      z < MIN_Z || z > MAX_Z) {
    Serial.println("ERR:BOUNDS");
    return false;
  }
  return true;
}

bool checkSensorStable() {
  // Check sensor over SENSOR_CHECK_DURATION ms
  // Return true if object is detected (LOW state is stable)
  // Return false if no object (HIGH state is stable)
  
  unsigned long startTime = millis();
  int lowCount = 0;
  int highCount = 0;
  int totalSamples = 0;
  
  Serial.print("Checking sensor for ");
  Serial.print(SENSOR_CHECK_DURATION);
  Serial.print("ms... ");
  
  while (millis() - startTime < SENSOR_CHECK_DURATION) {
    int sensorValue = digitalRead(CONTACT_SENSOR_PIN);
    
    if (sensorValue == LOW) {
      lowCount++;
    } else {
      highCount++;
    }
    totalSamples++;
    
    delay(SENSOR_SAMPLE_INTERVAL);
  }
  
  // Calculate percentages
  float lowPercent = (lowCount * 100.0) / totalSamples;
  float highPercent = (highCount * 100.0) / totalSamples;
  
  Serial.print("Samples: ");
  Serial.print(totalSamples);
  Serial.print(" | LOW: ");
  Serial.print(lowCount);
  Serial.print(" (");
  Serial.print(lowPercent, 1);
  Serial.print("%) | HIGH: ");
  Serial.print(highCount);
  Serial.print(" (");
  Serial.print(highPercent, 1);
  Serial.println("%)");
  
  // If majority of readings are LOW (object detected), return true
  // If majority of readings are HIGH (no object), return false
  if (lowCount > highCount) {
    Serial.println("OBJECT DETECTED (LOW dominant)");
    return true;
  } else {
    Serial.println("NO OBJECT (HIGH dominant)");
    return false;
  }
}

bool pickAndPlace(String name, float x, float y, float z) {
  // === PHASE 1: APPROACH & PICK ===
  
  // Move to safe height above target
  Serial.println("Moving to safe height above target");
  arm.gotoPoint(x, y, SAFE_TRAVEL_HEIGHT);
  arm.openGripper();
  delay(300);
  
  // Move down closer
  Serial.println("Descending to pickup height");
  arm.gotoPoint(x, y, z + 20);
  delay(200);
  
  // Move to object
  Serial.println("Moving to object");
  arm.gotoPoint(x, y, z);
  delay(200);
  
  // Close gripper
  Serial.println("Closing gripper");
  arm.closeGripper();
  delay(100);
  
  // === PHASE 2: CHECK SUCCESS WITH STABLE READING ===
  
  bool objectDetected = checkSensorStable();
  
  // If NO object detected (HIGH was dominant) = FAIL
  if (!objectDetected) {
    Serial.println("PICK FAILED - Gripper empty");
    
    // Release gripper
    arm.openGripper();
    delay(200);
    
    // Move up to safe height first
    Serial.println("Lifting to safe height");
    arm.gotoPoint(x, y, FAIL_SAFE_HEIGHT);
    delay(300);
    
    // Go HOME to clear camera view for re-detection
    Serial.println("Going HOME to clear camera view");
    goHome();
    
    return false;  // Pick failed
  }
  
  // === PHASE 3: SUCCESSFUL PICK - CONTINUOUS MODE ===
  
  Serial.println("OBJECT GRIPPED - Continuing...");
  
  // Lift object to safe travel height
  Serial.println("Lifting object");
  arm.gotoPoint(x, y, SAFE_TRAVEL_HEIGHT);
  delay(300);
  
  // === PHASE 4: NAVIGATE TO DROP ZONE ===
  
  float drop_x, drop_y, drop_z;
  
  if (name == "Unknown" || name == "unknown") {
    Serial.println("Going to UNKNOWN drop zone (LEFT)");
    drop_x = UNKNOWN_DROP_X;
    drop_y = UNKNOWN_DROP_Y;
    drop_z = UNKNOWN_DROP_Z;
  } else {
    Serial.print("→ Going to ");
    Serial.print(name);
    Serial.println(" drop zone (RIGHT)");
    drop_x = CLASS_DROP_X;
    drop_y = CLASS_DROP_Y;
    drop_z = CLASS_DROP_Z;
  }
  
  // Move to drop zone at safe height
  Serial.println("Moving to drop zone");
  arm.gotoPoint(drop_x, drop_y, drop_z + 20);
  delay(300);
  
  // Lower to drop height
  Serial.println("Lowering to drop");
  arm.gotoPoint(drop_x, drop_y, drop_z);
  delay(200);
  
  // Release object
  Serial.println("Releasing object");
  arm.openGripper();
  delay(300);
  
  // Lift gripper slightly after drop
  Serial.println("Lifting after drop");
  arm.gotoPoint(drop_x, drop_y, drop_z + 20);
  delay(200);
  
  // === PHASE 5: READY FOR NEXT PICK ===
  
  // DON'T GO HOME - Stay at drop zone ready for next coordinates
  Serial.println("DROP COMPLETE - Ready for next pick!");
  Serial.println("(Not going home - continuous mode)");
  
  return true;  // Pick successful
}

void goHome() {
  Serial.println("Going HOME");
  arm.gotoPoint(HOME_X, HOME_Y, HOME_Z);
  arm.openGripper();
  delay(200);
  Serial.println("At HOME position");
}