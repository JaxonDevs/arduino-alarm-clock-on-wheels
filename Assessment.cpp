#include <Wire.h>
#include "RTClib.h"
#include <Servo.h>

RTC_DS1307 rtc;
Servo myServoRight;
Servo myServoLeft;
Servo sensorServo;

const int servoRightPin = 12;
const int servoLeftPin = 13;
const int buttonPin = 5;
const int buzzerPin = 6;
const int trigPin = 3;
const int echoPin = 4;
const int sensorServoPin = 9;

const int targetHour = 6;
const int targetMinute = 0;

bool started = false;
bool stopped = false;

float obstacleDistance = 0;
unsigned long lastSensorMove = 0;
unsigned long sensorInterval = 200;
int sensorPos = 90; // center
int sensorDir = 1;

// state for non-blocking avoidance
bool avoiding = false;
unsigned long avoidStartTime = 0;
int avoidStage = 0;

void setup() {
  Serial.begin(9600);

  myServoRight.attach(servoRightPin);
  myServoLeft.attach(servoLeftPin);
  sensorServo.attach(sensorServoPin);

  Wire.begin();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if (!rtc.begin()) while (1);
  if (!rtc.isrunning()) rtc.adjust(DateTime(2025, 10, 30, 5, 59, 50));

  sensorServo.write(sensorPos);
}

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration_us = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  float distance_cm = 0.017 * duration_us;
  if (distance_cm == 0) distance_cm = 300;
  return distance_cm;
}

void setMotors(int right, int left) {
  myServoRight.write(right);
  myServoLeft.write(left);
}

void loop() {
  DateTime now = rtc.now();

  // Start alarm at target time
  if (!started && now.hour() == targetHour && now.minute() == targetMinute) {
    started = true;
    Serial.println("Alarm started!");
  }

  // Stop alarm on button press (immediate)
  if (started && !stopped && digitalRead(buttonPin) == LOW) {
    stopped = true;
    setMotors(90, 90);
    noTone(buzzerPin);
    Serial.println("Alarm stopped!");
  }

  if (started && !stopped) {
    tone(buzzerPin, 2000);
    obstacleDistance = readUltrasonic();
    Serial.print("Distance: ");
    Serial.println(obstacleDistance);

    if (!avoiding) {
      if (obstacleDistance < 10) {
        avoiding = true;
        avoidStage = 0;
        avoidStartTime = millis();
        Serial.println("Obstacle detected!");
      } else {
        // go forward normally
        setMotors(0, 170);
      }
    } else {
      //avoid objects
      unsigned long elapsed = millis() - avoidStartTime;

      if (avoidStage == 0) {
        // reverse for 400ms
        setMotors(170, 10);
        if (elapsed > 400) {
          avoidStage = 1;
          avoidStartTime = millis();
        }
      } else if (avoidStage == 1) {
        // turn right for 400ms
        setMotors(0, 90);
        if (elapsed > 400) {
          avoidStage = 2;
          avoidStartTime = millis();
        }
      } else if (avoidStage == 2) {
        // resume forward motion
        avoiding = false;
      }
    }

    // Sweep sensor servo
    if (millis() - lastSensorMove > sensorInterval) {
      sensorPos += sensorDir * 10;
      if (sensorPos >= 150) { sensorPos = 150; sensorDir = -1; }
      if (sensorPos <= 30)  { sensorPos = 30;  sensorDir = 1; }
      sensorServo.write(sensorPos);
      lastSensorMove = millis();
    }
  }
}
