// Locate the initial mid position 

#include <Servo.h>   

Servo srvRx; // pin 2 - right swing
Servo srvRy; // pin 3 - right tilt
Servo srvLx; // pin 7 - left swing
Servo srvLy; // pin 6 - left tilt
Servo srvLid; // pin 4 - eyelid tilt
Servo srvShut; // pin 5 - eyelid open/close
void setup()
{
  //initialize all servos

  srvRx.attach(2);
  srvRy.attach(3);
  srvLx.attach(7);
  srvLy.attach(6);
  srvLid.attach(4);
  srvShut.attach(5);

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
