#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- Pin Definitions ---
#define ENC_A 19      // Encoder Channel A stays on D19
#define ENC_B 4       // Move Channel B to D4 (Bypassing the shorted D18 pin)
#define POT_PIN 34    // Potentiometer Center Pin remains on D34

#define IN1 25        // L298N IN1
#define IN2 26        // L298N IN2
#define ENA 27        // L298N ENA (PWM Speed Control)

// --- HARDWARE CALIBRATION ---
const float TICKS_PER_REVOLUTION = 462.0; 

// --- I2C LCD Configuration ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- PWM Properties ---
const int pwmFreq = 5000;
const int pwmResolution = 8; 

// --- System Global Variables ---
volatile long encoderTicks = 0;
float currentAngle = 0.0;
float targetAngle = 0.0;

float lastDisplayedTarget = -9999;
float lastDisplayedCurrent = -9999;
unsigned long lastDisplayUpdateTime = 0;

// --- Smooth PID Tuning Parameters (Now dynamic!) ---
float Kp = 1.5;  
float Ki = 0.15;     
float Kd = 0.2;

float integral = 0, lastError = 0;
unsigned long lastTime = 0;

// --- Quadrature Interrupt Service Routine (ISR) ---
void IRAM_ATTR readEncoder() {
  int aState = digitalRead(ENC_A);
  int bState = digitalRead(ENC_B);
  
  if (aState == bState) {
    encoderTicks--;
  } else {
    encoderTicks++;
  }
}

void setup() {
  Serial.begin(115200); // Ensure your MATLAB script matches this baud rate

  // 1. Hardware Stabilization Delay
  delay(500); 

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PID Tuning Mode");
  delay(1000);
  lcd.clear();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // 2. Clear out any baseline float errors
  encoderTicks = 0; 
  currentAngle = 0.0;

  // 3. Read the absolute analog state AFTER the initial voltage sag has cleared
  int initialPot = analogRead(POT_PIN);
  targetAngle = map(initialPot, 0, 4095, -360, 360);

  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);
  ledcAttach(ENA, pwmFreq, pwmResolution);

  lastTime = millis();
}

void loop() {
  // --- NEW: LISTEN FOR MATLAB TUNING COMMANDS ---
  // Expects a string format: "Kp,Ki,Kd\n" (e.g., "2.5,0.1,0.4\n")
  if (Serial.available() > 0) {
    String incomingStr = Serial.readStringUntil('\n');
    int firstComma = incomingStr.indexOf(',');
    int secondComma = incomingStr.indexOf(',', firstComma + 1);
    
    // Validate that we received a properly formatted string with two commas
    if (firstComma != -1 && secondComma != -1) {
      Kp = incomingStr.substring(0, firstComma).toFloat();
      Ki = incomingStr.substring(firstComma + 1, secondComma).toFloat();
      Kd = incomingStr.substring(secondComma + 1).toFloat();
    }
  }

  // 1. Read Potentiometer and map strictly from -360 to +360 degrees
  int potValue = analogRead(POT_PIN);
  targetAngle = map(potValue, 0, 4095, -360, 360);

  // 2. Convert current encoder ticks directly into output shaft degrees
  currentAngle = (encoderTicks / TICKS_PER_REVOLUTION) * 360.0;

  // 3. Compute Delta Time (Seconds)
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  if (deltaTime <= 0) deltaTime = 0.001; 
  lastTime = currentTime;

  // 4. Run Core PID Angle Calculations
  float error = targetAngle - currentAngle;
  
  if (abs(error) < 0.8) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
    integral = 0; 
    lastError = error;
  } 
  else {
    integral += error * deltaTime;
    integral = constrain(integral, -220, 220); 
    
    float derivative = (error - lastError) / deltaTime;
    lastError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int motorSpeed = constrain(abs(output), 0, 255);

    // Adaptive Torque Floor
    if (motorSpeed > 0 && motorSpeed < 120) {
      motorSpeed = 95 + (int)(abs(error) * 0.8); 
      if (motorSpeed > 180) motorSpeed = 180; 
    }

    // Direction assignment logic
    if (output > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      ledcWrite(ENA, motorSpeed); 
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      ledcWrite(ENA, motorSpeed); 
    }
  }

  // 5. Non-blocking LCD Display Routine
  if (millis() - lastDisplayUpdateTime > 200) {
    lastDisplayUpdateTime = millis();
    
    if (abs(targetAngle - lastDisplayedTarget) > 0.5 || abs(currentAngle - lastDisplayedCurrent) > 0.5) {
      lcd.setCursor(0, 0);
      lcd.print("Target :        "); 
      lcd.setCursor(9, 0);
      lcd.print(targetAngle, 1);
      lcd.print((char)223); 
      
      lcd.setCursor(0, 1);
      lcd.print("Current:        "); 
      lcd.setCursor(9, 1);
      lcd.print(currentAngle, 1);
      lcd.print((char)223);

      lastDisplayedTarget = targetAngle;
      lastDisplayedCurrent = currentAngle;
    }
  }

  // 6. Structured Comma-Separated Stream for MATLAB Parsing
  Serial.print(targetAngle, 1);  Serial.print(",");
  Serial.print(currentAngle, 1); Serial.print(",");
  Serial.print(Kp, 2);           Serial.print(",");
  Serial.print(Ki, 2);           Serial.print(",");
  Serial.println(Kd, 2);         

  delay(10); 
}