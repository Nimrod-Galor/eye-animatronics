// Locate the initial mid position 

#include <Servo.h>   

Servo srvRx; // pin 6 - right swing
Servo srvRy; // pin 5 - right tilt
Servo srvLx; // pin 8 - left swing
Servo srvLy; // pin 7 - left tilt
Servo srvLid; // pin 9 - eyelid tilt
Servo srvShut; // pin 4 - eyelid open/close

void setup()
{
  //initialize all servos

  srvRx.attach(6);
  srvRy.attach(5);
  srvLx.attach(8);
  srvLy.attach(7);
  srvLid.attach(9);
  srvShut.attach(4);

}

void loop(void)
{
  srvRx.write(90);
  delay(50);
  srvRy.write(90);
  delay(50);
  srvLx.write(90);
  delay(50);
  srvLy.write(90);
  delay(50);
  srvLid.write(90);
  delay(50);
  srvShut.write(90);
  delay(250);    
}
