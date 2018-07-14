#include <Servo.h> 
Servo myservo;
int pos = 0;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(5);
  Serial.begin(9600);
  myservo.write(0);
  //Serial.println("Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  char inByte = ' ';
  //Serial.println("hello");
  if(Serial.available()){ // only send data back if data has been sent
  byte inByte = Serial.read(); // read the incoming data
  myservo.write(int(inByte));              // tell servo to go to position in variable 'pos' 
  delay(10);
  Serial.println(inByte,DEC); // send the data back in a new line so that it is not all one long line
  }
  delay(100); // delay for 1/10 of a second
}
