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
       
    }
    angle = Serial.parseFloat();
    Serial.print(angle);
    stepper.setMaxSpeed(100);
    stepper.setAcceleration(50);

    fangle = round((200.0*angle/360.0));
    stepper.moveTo(fangle); 
}

void loop() {
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    delay(1000);
    stepper.moveTo(homePosition);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}