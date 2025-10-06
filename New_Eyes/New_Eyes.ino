#include <Servo.h>

// Servo objects
Servo baseServo;
Servo leftVertical;
Servo rightVertical;
Servo leftHorizontal;
Servo rightHorizontal;
Servo leftEye;
Servo rightEye;

// --- Pin Definitions ---
const int MODE_BUTTON_PIN = A1; // Physical button to cycle modes

// --- Mode Control ---
int currentMode = 0; // 0=Auto, 1=Gamepad, 2=OpenCV
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
bool lastButtonState = HIGH;

// --- Serial Communication ---
String inputString = "";
bool stringComplete = false;

// --- Movement & Smoothing ---
float targetBase = 90, currentBase = 90;
float targetLeftVert = 90, currentLeftVert = 90;
float targetRightVert = 90, currentRightVert = 90;
float targetLeftHoriz = 90, currentLeftHoriz = 90;
float targetRightHoriz = 90, currentRightHoriz = 90;
float targetLeftEye = 90, currentLeftEye = 90;
float targetRightEye = 90, currentRightEye = 90;

float smoothingFactor = 0.04; // Smoother factor for general movement
float eyeSmoothingFactor = 0.07; // Slightly faster for eyes
unsigned long lastServoUpdate = 0;

// --- Autonomous Mode State ---
unsigned long lastAutoAction = 0;
unsigned long autoActionInterval = 3000;
unsigned long nextBlinkTime = 0;

// MODIFIED: New eye closing limits to prevent servo noise
const int LEFT_EYE_CLOSE = 25;
const int RIGHT_EYE_CLOSE = 155;
const int EYE_OPEN = 90;


void setup() {
  Serial.begin(115200);
  
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  
  baseServo.attach(2);
  leftVertical.attach(3);
  rightVertical.attach(4);
  leftHorizontal.attach(5);
  rightHorizontal.attach(6);
  leftEye.attach(7);
  rightEye.attach(8);
  
  // Initialize to neutral and set first blink time
  resetToNeutral();
  updateAndWriteServos();
  // MODIFIED: Increased initial blink time for a more natural start
  nextBlinkTime = millis() + random(4000, 8000); 
  
  delay(500);
  sendModeStatus();
}

void loop() {
  unsigned long currentTime = millis();
  
  handleSerial();
  handleButtonPress();

  if (currentMode == 0) {
    runInternalAutoMode(currentTime);
  }

  // Smoothly move servos towards their target positions (Linear Interpolation)
  if (currentTime - lastServoUpdate >= 20) {
    updateAndWriteServos();
    lastServoUpdate = currentTime;
  }
}

void handleButtonPress() {
  bool buttonState = digitalRead(MODE_BUTTON_PIN);
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      // Cycle through modes: 0 -> 1 -> 2 -> 0
      currentMode = (currentMode + 1) % 3;
      resetToNeutral();
      sendModeStatus();
    }
  }
  lastButtonState = buttonState;
}

void runInternalAutoMode(unsigned long currentTime) {
  // Handle autonomous blinking
  if (currentTime > nextBlinkTime) {
    targetLeftEye = LEFT_EYE_CLOSE;
    targetRightEye = RIGHT_EYE_CLOSE;
    // MODIFIED: Made blink interval longer and more random
    nextBlinkTime = currentTime + random(5000, 12000); 
  } else if (abs(currentLeftEye - LEFT_EYE_CLOSE) < 5) {
    // If eyes are closed, open them again
    targetLeftEye = EYE_OPEN;
    targetRightEye = EYE_OPEN;
  }

  // MODIFIED: More dynamic and wider range of autonomous movements
  if (currentTime - lastAutoAction > autoActionInterval) {
    int movementType = random(0, 7); // Increased variety
    switch(movementType) {
      case 0: // Look left
        targetBase = random(110, 130);
        targetLeftHoriz = random(70, 80);
        targetRightHoriz = random(70, 80);
        break;
      case 1: // Look right
        targetBase = random(50, 70);
        targetLeftHoriz = random(100, 110);
        targetRightHoriz = random(100, 110);
        break;
      case 2: // Look up
        targetLeftVert = random(60, 75);
        targetRightVert = 180 - targetLeftVert;
        break;
      case 3: // Look down
        targetLeftVert = random(105, 120);
        targetRightVert = 180 - targetLeftVert;
        break;
      case 4: // Curious tilt
        targetBase = random(80, 100);
        targetLeftVert = random(70, 85);
        targetRightVert = 180 - random(95, 110);
        break;
      default: // Center / Idle
        resetToNeutral();
        break;
    }
    lastAutoAction = currentTime;
    autoActionInterval = random(2500, 6000); // More varied timing
  }
}


void updateAndWriteServos() {
  // This is the Linear Interpolation logic you wanted
  currentBase = (targetBase * smoothingFactor) + (currentBase * (1.0 - smoothingFactor));
  currentLeftVert = (targetLeftVert * smoothingFactor) + (currentLeftVert * (1.0 - smoothingFactor));
  currentRightVert = (targetRightVert * smoothingFactor) + (currentRightVert * (1.0 - smoothingFactor));
  currentLeftHoriz = (targetLeftHoriz * smoothingFactor) + (currentLeftHoriz * (1.0 - smoothingFactor));
  currentRightHoriz = (targetRightHoriz * smoothingFactor) + (currentRightHoriz * (1.0 - smoothingFactor));
  currentLeftEye = (targetLeftEye * eyeSmoothingFactor) + (currentLeftEye * (1.0 - eyeSmoothingFactor));
  currentRightEye = (targetRightEye * eyeSmoothingFactor) + (currentRightEye * (1.0 - eyeSmoothingFactor));
  
  baseServo.write((int)currentBase);
  leftVertical.write((int)currentLeftVert);
  rightVertical.write((int)currentRightVert);
  leftHorizontal.write((int)currentLeftHoriz);
  rightHorizontal.write((int)currentRightHoriz);
  leftEye.write((int)currentLeftEye);
  rightEye.write((int)currentRightEye);
}

void setMode(int mode) {
  if (mode != currentMode) {
    currentMode = mode;
    resetToNeutral();
    sendModeStatus();
  }
}

void sendModeStatus() {
  Serial.print("MODE:");
  Serial.println(currentMode);
}

void resetToNeutral() {
  targetBase = 90;
  targetLeftVert = 90;
  targetRightVert = 90;
  targetLeftHoriz = 90;
  targetRightHoriz = 90;
  targetLeftEye = EYE_OPEN;
  targetRightEye = EYE_OPEN;
}

void handleSerial() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') stringComplete = true;
    else inputString += inChar;
  }
  
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void processSerialCommand(String command) {
  command.trim();
  if (command.startsWith("MODE:")) {
    if (command.substring(5) == "?") sendModeStatus();
    else setMode(command.substring(5).toInt());
  } else if (command.startsWith("SERVO:")) {
    if (currentMode == 1 || currentMode == 2) {
      parseServoCommand(command.substring(6));
    }
  }
}

void parseServoCommand(String data) {
  int lastIndex = 0;
  float values[7];
  int valueIndex = 0;
  for (int i = 0; i < data.length() && valueIndex < 7; i++) {
    if (data.charAt(i) == ',') {
      values[valueIndex++] = data.substring(lastIndex, i).toFloat();
      lastIndex = i + 1;
    }
  }
  values[valueIndex] = data.substring(lastIndex).toFloat();

  if (valueIndex == 6) {
    targetBase = values[0];
    targetLeftVert = values[1];
    targetRightVert = values[2];
    targetLeftHoriz = values[3];
    targetRightHoriz = values[4];
    targetLeftEye = values[5];
    targetRightEye = values[6];
  }
}

