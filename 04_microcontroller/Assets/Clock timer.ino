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
    // do nothing, just wait until you actually type something and hit enter
  }
  studyMinutes = Serial.parseInt();
  Serial.print(studyMinutes);

  stepper.setMaxSpeed(100);
  stepper.setSpeed(-200.0 / (studyMinutes * 60.0)); // use floats to avoid integer division issues
  
  starttime = millis(); // record the start time in milliseconds
}

void loop() {
  elapsedTime = (millis() - starttime) / 1000; // get the elapsed time in seconds
  stepper.runSpeed();
  if (elapsedTime >= studyMinutes * 60) { // check if the elapsed time is greater than or equal to the specified study time in seconds
    stepper.setSpeed(0); // stop the stepper
  }
}