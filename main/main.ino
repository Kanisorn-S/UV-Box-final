#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 170
#define SERVOMAX 650
#define SERVOPIN 0
#define OPEN_ANGLE 90
#define CLOSE_ANGLE 0
#define PUSHBUTTON_PIN 13
#define DELAY_TIME 5000

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

bool isRotating = false;
bool isClosed = true;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50); 
  closeLid(); // Start with the lid closed
  
  pinMode(PUSHBUTTON_PIN, INPUT_PULLUP);
  delay(1000);
}

void loop() {
  int buttonState = digitalRead(PUSHBUTTON_PIN);
  if (buttonState == LOW && !isRotating) {
    if (isClosed) {
      openLid();
      delay(DELAY_TIME); 
      closeLid();
    }
  }
}

void openLid() {
  isRotating = true;
  pwm.setPWM(SERVOPIN, 0, angleToPulse(OPEN_ANGLE));
  delay(15);
  Serial.println("Lid opened");
  isClosed = false;
  isRotating = false;
}

void closeLid() {
  isRotating = true;
  pwm.setPWM(SERVOPIN, 0, angleToPulse(CLOSE_ANGLE));
  delay(15);
  Serial.println("Lid closed");
  isClosed = true;
  isRotating = false;
}

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

