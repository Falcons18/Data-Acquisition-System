#include <NeoSWSerial.h>
#include <Wire.h>
#include <MS5611.h>
#include<Servo.h>
#include<Kalman.h>
#include<TinyGPS++.h>

#define GLAT 12.972138
#define GLON 79.155670

TinyGPSPlus gps;
NeoSWSerial ss(4,3);
Kalman kalman(0.6, 0.6, 1, 0);
MS5611 ms5611;
Servo servo1, servo2;
float referencePressure=0, temp=0, h=0, Lat=0, Lon=0, Vel=0;
char t;
int flag = -1, dropFlag = 0;

void setup() 
{
  Serial.begin(57600);
  ss.begin(9600);

  servo1.attach(5);
  servo2.attach(9);
  servo1.write(115);
  servo2.write(115);
  
  ms5611.begin();
  delay(1000);

  // Get reference pressure for relative altitude
   for(int i=0; i<4; i++)  
   { 
     referencePressure += ms5611.readPressure();
     delay(10);
   } 
  referencePressure = referencePressure/4;

   for(int i=0; i<4; i++)
   {
     temp += ms5611.readTemperature();
     delay(10);
   }
   temp = temp/4;

   digitalWrite(6, HIGH);
   pinMode(6, OUTPUT);
}
int c=0;

void loop()
{ 
  float p = 0, c;
  unsigned long afterdrop, start;
  
  if(flag == -1)
  {
    flag=0;
    delay(1000);
  }

  start = millis();
  while((millis() - start) <= 100)
   {
    if(Serial.available())
      {
        t = Serial.read();
        if(t=='d' || t=='D')
        { 
          Serial.print("t\n");
          hatch_open();
          afterdrop = millis();
          flag = 1;
        }
        else if(t=='s' || t=='S')
        {
          dropFlag = 1;
        }
        else if(t=='o' || t=='O')
        {
          hatch_open();
        }
        else if (t=='c' || t=='C')
        {
          hatch_1_close();
          hatch_2_close();
        }
        else if(t=='r'||t=='R')
        {
          digitalWrite(6, LOW);
        }
      } 
      
    while(ss.available())
      gps.encode(ss.read());    
}
  
   p = calcP();
   c = calcHeight(p, temp);
   p = calcP();
   c += calcHeight(p, temp);
   h = kalman.getFilteredValue(c/2);
 
 Serial.print("h"); 
 Serial.println(h, 2);

 if(gps.location.isUpdated())
 {
    Lat = gps.location.lat();
    Lon = gps.location.lng();
    Vel = gps.speed.kmph();
    Vel = Vel * (5/18) * 3.28084;  
 }  

 Serial.println(Lat, 6);
 Serial.println(Lon, 6);
 Serial.println(Vel, 2); 

 if(dropFlag == 1)
 {
    float dropRange = calcRange(h, Vel);
    float havRange = haversineDistance(GLAT, GLON, Lat, Lon);

    if(havRange <= dropRange + 40)
      {
        Serial.print("t\n");
        hatch_open();
        afterdrop = millis();
        flag = 1;
      }
 }     

//IF HATCH IS OPEN
 if(flag == 1 && millis()-afterdrop >= 1000)
 {
    hatch_1_close();
 }

if(flag == 1 && millis()-afterdrop >=1700)
 {
  hatch_2_close();
  flag = 0;
 }
}

void hatch_open()
{
  servo1.write(0);
  servo2.write(0);
}

void hatch_1_close()
{
  servo1.write(115);
}  

void hatch_2_close()
{
  servo2.write(115);
}

float calcHeight(float p, float t)
{
  float a = pow((referencePressure/p), 0.19022) - 1;
  float b = a*(t + 273.15);
  float c2 = b/0.0065; 
  return c2*3.28084;
}

float calcP()
{
  float p = 0; 
  for(int i=0; i<4; i++)  
   { 
     p += ms5611.readPressure();
     delay(10);
   } 
      
  return p = p/4;
}

float calcRange(float h, float vel)
{
  return sqrt(2*h/9.8)*vel;
}

float haversineDistance(float glat, float glon, float flat, float flon)
{
  glat *= 0.017453;
  glon *= 0.017453;
  flat *= 0.017453;
  flon *= 0.017453;

  float dlon = flon - glon; 
  float dlat = flat - glat; 
  float a = pow(sin(dlat/2),2) + cos(glat) * cos(flat) * pow(sin(dlon/2),2);
  float c = 2*atan2(sqrt(a), sqrt(1-a));
  return fabs(6371000*c*3.28084);
}

