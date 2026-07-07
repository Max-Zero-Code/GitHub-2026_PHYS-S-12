#include <AccelStepper.h>

const int stepPin = D0;
const int dirPin  = D1;
unsigned long elapsedTime = 0;
int studyMinutes = 0;
unsigned long starttime = 0;

AccelStepper stepper(1, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  
  Serial.println("How long will you study today? (in minutes)");
   while (Serial.available() == 0) {
  }
  studyMinutes = Serial.parseInt();
  Serial.print(studyMinutes);

  stepper.setMaxSpeed(100);
  stepper.setSpeed(-200.0 / (studyMinutes * 60.0)); 
  
  starttime = millis(); 
}

void loop() {
  elapsedTime = (millis() - starttime) / 1000; 
  stepper.runSpeed();
  if (elapsedTime >= studyMinutes * 60) {
    stepper.setSpeed(0);
  }
}