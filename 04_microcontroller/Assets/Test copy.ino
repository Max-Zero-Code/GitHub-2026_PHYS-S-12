int buttonPin_1 = D6;
int buttonPin_2 = D5;
int buttonPin_3 = D4;
int buttonPin_4 = D3;
String Entered = "";
String Passcode = "";

void clear() {
  while (Serial.available() > 0) {
    Serial.read(); 
  }
}

void setup() {
Serial.begin(9600); 
pinMode(buttonPin_1, INPUT);
pinMode(buttonPin_2, INPUT);
pinMode(buttonPin_3, INPUT);
pinMode(buttonPin_4, INPUT);

attachInterrupt(digitalPinToInterrupt(buttonPin_1), buttonPressed_1, RISING);
attachInterrupt(digitalPinToInterrupt(buttonPin_2), buttonPressed_2, RISING);
attachInterrupt(digitalPinToInterrupt(buttonPin_3), buttonPressed_3, RISING);
attachInterrupt(digitalPinToInterrupt(buttonPin_4), buttonPressed_4, RISING);

Serial.println("Button passcode");
while (Serial.available() == 0) {
    
  }
Passcode = Serial.readStringUntil('\n');
Entered = ""; 
}

void loop() {
 if (Entered.length() == 4) {
    Serial.println(Entered == Passcode ? "Access Granted" : "Access Denied");
    Entered = ""; 
 } 
clear();

}

void buttonPressed_1() {
    delay(500);
    Serial.println("1");
    Entered += "1";
  
}   

void buttonPressed_2() {
    delay(500);
    Serial.println("2");
    Entered += "2";
}       

void buttonPressed_3() {
    delay(500);
    Serial.println("3");
    Entered += "3";
}

void buttonPressed_4() {
    delay(500);
    Serial.println("4");
    Entered += "4";
}
