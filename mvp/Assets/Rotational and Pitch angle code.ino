#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <cmath>

const int stepPin = D7;
const int dirPin  = D1;
float angle = 0.0;
int fangle = 0;
long homePosition = 0;

AccelStepper stepper(1, stepPin, dirPin);

const int servoPin = D6;
Servo pitchServo;
float pitchAngle = 0.0;

void setup() {
    Serial.begin(9600);
    delay(200);

    homePosition = stepper.currentPosition();
    stepper.setMaxSpeed(100);
    stepper.setAcceleration(50);

    pitchServo.attach(servoPin);

    // Wait for the rotation angle from the PC, sent as "ROTATION:270.0\n"
    while (Serial.available() == 0) {
        yield();
    }
    String rotLine = Serial.readStringUntil('\n');
    if (rotLine.startsWith("ROTATION:")) {
        angle = rotLine.substring(9).toFloat();
    }

    fangle = round((200.0 * angle / 360.0));
    stepper.moveTo(fangle);

    while (stepper.distanceToGo() != 0) {
        stepper.run();
        yield();
    }

    delay(5000);

    // Wait for the pitch angle from the PC, sent as "PITCH:37.5\n"
    while (Serial.available() == 0) {
        yield();
    }
    String pitchLine = Serial.readStringUntil('\n');
    if (pitchLine.startsWith("PITCH:")) {
        pitchAngle = pitchLine.substring(6).toFloat();
        pitchAngle = constrain(pitchAngle, 0, 180);
        pitchServo.write(pitchAngle);
    }
}

void loop() {
    stepper.moveTo(fangle);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        yield();
    }
    delay(1000);

    stepper.moveTo(homePosition);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        yield();
    }
    delay(1000);
}