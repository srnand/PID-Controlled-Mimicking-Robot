// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object 
  //myservo.write(0);
  myservo.write(30);
  delay(5000);
  myservo.write(90);
  delay(5000);
  myservo.write(120);
  delay(5000);
} 
 
 
void loop() 
{ 
  //for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    //myservo.write(30);              // tell servo to go to position in variable 'pos' 
    //delay(2000);                       // waits 15ms for the servo to reach the position 
    //myservo.write(90);              // tell servo to go to position in variable 'pos' 
    //delay(2000);
    //myservo.write(120);              // tell servo to go to position in variable 'pos' 
    //delay(2000);
  } 
  /*for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }*/ 
} 
