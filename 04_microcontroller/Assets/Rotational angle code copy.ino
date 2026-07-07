#include <AccelStepper.h>
#include <Arduino.h>
#include <cmath>

const int stepPin = D0;
const int dirPin  = D1;
float angle = 0.0;
int fangle = 0;
long homePosition = 0;


AccelStepper stepper(1, stepPin, dirPin);

void setup() {
    Serial.begin(9600);

    homePosition = stepper.currentPosition();

    Serial.println("Angle of rotation: ");
    while (Serial.available() == 0) {
        // do nothing, just wait until you actually type something and hit enter
    }
    angle = Serial.parseFloat();
    Serial.print(angle);
    stepper.setMaxSpeed(100);
    stepper.setAcceleration(50); // <- add this

    fangle = round((200.0*angle/360.0));
    stepper.moveTo(fangle); // use floats to avoid integer division issues
}

void loop() {
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    delay(1000);
    stepper.moveTo(homePosition); // move back to home position after reaching the target ang
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}