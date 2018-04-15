#include<Encoder.h>
const int ERA = 6; // Encoder pins
const int ERB = 7;
const int ELA = 8;
const int ELB = 9;
const int RDIR = 11;
const int RPWM = 10;
const int LDIR = 18;
const int LPWM = 20;
const int DIST = 15;
  Encoder enc1(ERA, ERB);
  Encoder enc2(ELA, ELB);
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  pinMode(ERA, INPUT);
//  pinMode(ERB, INPUT);
//  pinMode(ELA, INPUT);
//  pinMode(ELB, INPUT);
  pinMode(RPWM, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // 3mm per count!
   long left = enc2.read();
   Serial.println(left);
   analogWrite(LPWM, 0.003);
   delay(50);
}
