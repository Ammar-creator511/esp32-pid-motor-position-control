#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- Pin Definitions ---
#define ENC_A 19      
#define ENC_B 4       
#define POT_PIN 34    

#define IN1 25        
#define IN2 26        
#define ENA 27        

const float TICKS_PER_REVOLUTION = 462.0; 
LiquidCrystal_I2C lcd(0x27, 16, 2); 

const int pwmFreq = 5000;
const int pwmResolution = 8; 

volatile long encoderTicks = 0;
float currentAngle = 0.0;
float targetAngle = 0.0;
float angleDifference = 0.0; // Variable to store the error gap

unsigned long lastDisplayUpdateTime = 0;

// --- Smooth PID Tuning Parameters (Dynamic) ---
float Kp = 1.5;  
float Ki = 0.15;     
float Kd = 0.2;

float integral = 0, lastError = 0;
unsigned long lastTime = 0;

void IRAM_ATTR readEncoder() {
  int aState = digitalRead(ENC_A);
  int bState = digitalRead(ENC_B);
  if (aState == bState) encoderTicks--; else encoderTicks++;
}

void setup() {
  Serial.begin(115200); 
  delay(500); 

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PID Dynamic Mode");
  delay(1000);
  lcd.clear();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  encoderTicks = 0; 
  currentAngle = 0.0;

  int initialPot = analogRead(POT_PIN);
  targetAngle = map(initialPot, 0, 4095, -360, 360);

  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, CHANGE);
  ledcAttach(ENA, pwmFreq, pwmResolution);

  lastTime = millis();
}

void loop() {
  // --- LISTEN FOR MATLAB TUNING COMMANDS ---
  if (Serial.available() > 0) {
    String incomingStr = Serial.readStringUntil('\n');
    int firstComma = incomingStr.indexOf(',');
    int secondComma = incomingStr.indexOf(',', firstComma + 1);
    
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
  angleDifference = error; // Save difference for display

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

    // Adaptive Torque Floor - Helps eliminate the steady-state gap
    if (motorSpeed > 0 && motorSpeed < 120) {
      motorSpeed = 95 + (int)(abs(error) * 0.8); 
      if (motorSpeed > 180) motorSpeed = 180; 
    }

    if (output > 0) {
      digitalWrite(IN1, HIGH);   digitalWrite(IN2, LOW);    ledcWrite(ENA, motorSpeed); 
    } else {
      digitalWrite(IN1, LOW);    digitalWrite(IN2, HIGH);   ledcWrite(ENA, motorSpeed); 
    }
  }

  // 5. High-Density 16x2 LCD Display Routine
  if (millis() - lastDisplayUpdateTime > 150) {
    lastDisplayUpdateTime = millis();
    
    // Row 1 Layout: Displays Target, Current, and the raw Difference cleanly
    lcd.setCursor(0, 0);
    lcd.print("T:");    lcd.print((int)targetAngle);
    lcd.print(" C:");   lcd.print((int)currentAngle);
    lcd.print(" D:");   lcd.print((int)angleDifference);
    lcd.print("    "); // Clears any residual character fragments

    // Row 2 Layout: "P1.5 I0.15 D0.20"
    lcd.setCursor(0, 1);
    lcd.print("P"); lcd.print(Kp, 1);
    lcd.print(" I"); lcd.print(Ki, 2);
    lcd.print(" D"); lcd.print(Kd, 2);
    lcd.print("  "); 
  }

  // 6. Structured Comma-Separated Stream for MATLAB Parsing
  Serial.print(targetAngle, 1);  Serial.print(",");
  Serial.print(currentAngle, 1); Serial.print(",");
  Serial.print(Kp, 2);           Serial.print(",");
  Serial.print(Ki, 2);           Serial.print(",");
  Serial.println(Kd, 2);         

  delay(10); 
}