/*
 *  Original ADNS3080 Code by: Simon Winder
 *      https://github.com/impressivemachines/Arduino
 *      https://www.youtube.com/user/robotbugs
 *  small changes by: Jan Neumann aka. Neumi
 *      https://github.com/Neumi
 *      https://www.youtube.com/user/NeumiElektronik
 * Motion Follow Eye animatronics Created By Galor Nimrod 11/7/2018
 *      https://github.com/Nimrod-Galor/eye-animatronics
 *
 * Arduino sketch for eye animatronics
 * Based on Optical Flow Sensor (ADNS3080)
 *      instructables: https://www.instructables.com/id/Eye-Animatronics/
 *      youtube: https://youtu.be/Ep9fWQiFmoM
*/

#include <SPI.h>
#include <Servo.h>

Servo srvRx; // pin 2 - right swing
Servo srvRy; // pin 3 - right tilt
Servo srvLx; // pin 7 - left swing
Servo srvLy; // pin 6 - left tilt
Servo srvLid; // pin 4 - eyelid tilt
Servo srvShut; // pin 5 - eyelid open/close

short loopDelay = 10;
int posX = 90; //swing
int posY = 90; // tilt
int posS = 90; // shut
short blinkMax = 55;
short blinkMin = 80;
short yMin = 70;
short yMax = 110;
short xMin = 60;
short xMax = 120;
short posYrev = 90;
unsigned long blinkTime = 2000;
unsigned long lastTrack = 0;
const int ledCount = 4;    // the number of LEDs in the bar graph
int ledPins[] = {A7, A6, A5, A4};   // an array of pin numbers to which LEDs are attached
bool rest = true;

// these pins may be different on different boards
// this is for the uno
//#define PIN_SS        8
#define PIN_MISO      12
#define PIN_MOSI      11
#define PIN_SCK       13

#define PIN_MOUSECAM_RESET     9
#define PIN_MOUSECAM_CS        10

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

void mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}

int mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  
  mousecam_reset();
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);

  return 0;
}

void mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(1);
  return ret;
}

struct MD
{
 byte motion;
 char dx, dy;
 byte squal;
 word shutter;
 byte max_pix;
};

void mousecam_read_motion(struct MD *p)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  p->motion =  SPI.transfer(0xff);
  p->dx =  SPI.transfer(0xff);
  p->dy =  SPI.transfer(0xff);
  p->squal =  SPI.transfer(0xff);
  p->shutter =  SPI.transfer(0xff)<<8;
  p->shutter |=  SPI.transfer(0xff);
  p->max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(5);
}

void setup() 
{
  Serial.begin(38400);
  Serial.println("Init");
  
  //pinMode(PIN_SS,OUTPUT);
  pinMode(PIN_MISO,INPUT);
  pinMode(PIN_MOSI,OUTPUT);
  pinMode(PIN_SCK,OUTPUT);
  
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  if(mousecam_init()==-1)
  {
    Serial.println("Mouse cam failed to init");
    while(1);
  }

  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  srvRx.attach(2);
  srvRy.attach(3);
  srvLx.attach(7);
  srvLy.attach(6);
  srvLid.attach(4);
  srvShut.attach(5);

  srvRx.write(posX);
  srvLx.write(posX);
  srvRy.write(posY);
  srvLy.write(posY);
  srvLid.write(posS);
  srvShut.write(blinkMax);
}

char asciiart(int k)
{
  static char foo[] = "WX86*3I>!;~:,`. ";
  return foo[k>>4];
}

byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];

void loop() 
{

  delay(loopDelay);

  //Blink
  if(!rest){
    blinkTime = blinkTime - loopDelay;
    Serial.println(blinkTime);
    if(blinkTime <= 100){// open
      
      srvShut.write(blinkMin);
      blinkTime = random(2, 12) * 500;
    }else if(blinkTime <= 200){// close
      srvShut.write(blinkMax);
    }
  }
  //End Blink

  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  MD md;
  mousecam_read_motion(&md);

  // LED Bar
  // this section produces a bar graph of the surface quality that can be used to focus the camera
  int ledLevel = map(md.squal/4, 0, 12, 0, ledCount);
  //Serial.print("led level: ");
  //Serial.println(ledLevel);
  // loop over the LED array:
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    if (thisLed <= ledLevel) { // if the array element's index is less than ledLevel turn the pin for this element on:
      digitalWrite(ledPins[thisLed], HIGH);
    }else { // turn off all pins higher than the ledLevel:
      digitalWrite(ledPins[thisLed], LOW);
    }
  }
  // End LED Bar

  if((int)md.dx != 0){
      if(rest){
        srvShut.write(blinkMin);
        rest = false;
      }
      posX -= (int)md.dx;
      if(posX > xMax){
        posX = xMax;
      }else if(posX < xMin){
        posX = xMin;
      }
      
      srvRx.write(posX);
      srvLx.write(posX);

      lastTrack = millis();

      Serial.print("X: ");
      Serial.println(posX);
  }

  if((int)md.dy != 0){
    if(rest){
        srvShut.write(blinkMin);
        rest = false;
    }
    
    posY -= (int)md.dy;
    
    if(posY > yMax){
      posY = yMax;
    }else if(posY < yMin){
      posY = yMin;
    }

    srvRy.write(posY);
    posYrev = map(posY, yMin, yMax, yMax, yMin);
    srvLy.write(posYrev);

    srvLid.write(posYrev);

    lastTrack = millis();

    Serial.print("Y: ");
    Serial.println(posY);
  }

  if( millis() - lastTrack >= 3000){
    Serial.println("reset");
    rest = true;
    posX = 90; //swing
    posY = 90; // tilt
    posS = 90; // shut

    srvRx.write(posX);
    srvRy.write(posY);
    srvLx.write(posX);
    srvLy.write(posY);
    srvLid.write(posS);
    srvShut.write(blinkMax);
  }
 
  
}
