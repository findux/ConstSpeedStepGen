#include "ConstStepGen.h"


String SeriBilgi;
char c;
float negTamSayi;

ConstStepGen stepper0(A0, A2);

void setup() {
  Serial.begin(115200);
  delay(1);

  pinMode(8, OUTPUT);

  stepper0.setMaxSpeed(200);
  stepper0.setSpeed(200);
  //stepper0.moveTo(-5);

}

void loop() {

//  if (digitalRead(8))
//  {
//    delay(300);
//    stepper0.run();
//    while (digitalRead(8)) {};
//  }
  stepper0.run();
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseSeriBilgi(SeriBilgi);
      SeriBilgi = "";
    }
    else {
      SeriBilgi += c;
    }
  }
}
void parseSeriBilgi(String com) {
  signed long pos, speeddd ;

  String Part0, Part1;
  Part0 = com.substring(0, com.indexOf(" "));

  com = com.substring(com.indexOf(" ") + 1);
  Part1 = com.substring(0, com.indexOf(" "));
  pos = Part0.toFloat();
  speeddd = Part1.toFloat();

  stepper0.setSpeed(speeddd);
  stepper0.moveTo(pos);

}

