#include <Arduino.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "compass.h"

#define GPSBaud 9600
#define ConsoleBaud 9600

// The serial connection to the GPS device
// The TinyGPS++ object
TinyGPSPlus gps1, gps2, gps3 ;

float location_lat;
float location_lng;
float degree;

//Define variables for the SD card storage
File file;
unsigned int filenumber = 1;
String fileName = String();

unsigned long lastUpdateTime = 0;

//Define varibles for bluetooth communication and get lattitute and longitude destination
String inData;
boolean stopped = true;
boolean moving = false;
boolean arrived = false;
float LAT =0;
float LNG =1;

// Define pinout for the L298N board
int PWMA = 7;
int IN1 = 6;
int IN2 = 5;
int IN3 = 4;
int IN4 = 3;
int PWMB = 2; 
  
void setup()
{
  // Define pinout for the L298N board
  pinMode(PWMA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // Begin serial communication
  Serial.begin(ConsoleBaud);
  Serial1.begin(GPSBaud);
  Serial2.begin(GPSBaud);
  Serial3.begin(GPSBaud);

  // Begin I2C communication 
  Wire.begin();

  //Compass pre calibration, offset, error and gain configuration
  compass_x_offset = -17.16;
  compass_y_offset = -102.02;
  compass_z_offset = -161.82;
  compass_x_gainError = 1.00;
  compass_y_gainError = 1.03;
  compass_z_gainError = 0.97;
  compass_init(2);
  compass_debug = 1;
  compass_offset_calibration(3);


  //Verifies if the card is connected and create a file
  if (!SD.begin(53)) {
  Serial.println("initialization failed!");
  return;
  }
  else{
  Serial.println("initialization done.");
  }

  while(filenumber != 0){
    fileName = "data";
    fileName += filenumber;
    fileName += ".txt";
    
    char charFileName[fileName.length()+1];
    fileName.toCharArray(charFileName, fileName.length()+1);
    
    if(SD.exists(charFileName)){
      filenumber++;
    }
    else{
      file = SD.open(charFileName, FILE_WRITE);  
      filenumber=0;
    }
  }
}

void loop()
{
 //Setting up LAT and LNG through Bluetooth communication
 while (Serial.available()>0 && LAT == 0) 
  {
    inData = Serial.readString();
    LAT = inData.toFloat();
    LNG = 0;
  }

  while (Serial.available()>0 && LNG == 0) 
  {
    inData = Serial.readString();
    LNG = inData.toFloat();
  }

  while (LAT != 0 && LNG != 0){

    moving = true;
    //reset command
    
    if (Serial.available()>0 && Serial.readString() == "r"){
      LAT = 0;
      file.close();
      run(0, HIGH, LOW, HIGH, LOW, 0);
      return;
    }


  // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (Serial1.available() > 0 && Serial2.available() > 0 && Serial3.available() >0){
     

    gps1.encode(Serial1.read());
    gps2.encode(Serial2.read());
    gps3.encode(Serial3.read());
}

  // Every 1 seconds, do an update.
  if (millis() - lastUpdateTime >= 1000)
  {
    
      lastUpdateTime = millis();
      compass_heading();
      Serial.println();

      // average of the three gps modules readings
      location_lat = (gps1.satellites.value()*gps1.location.lat() +gps2.satellites.value()*gps2.location.lat()+gps3.satellites.value()*gps3.location.lat())/(gps1.satellites.value()+gps2.satellites.value()+gps3.satellites.value());
      location_lng = (gps1.satellites.value()*gps1.location.lng() +gps2.satellites.value()*gps2.location.lng()+gps3.satellites.value()*gps3.location.lng())/(gps1.satellites.value()+gps2.satellites.value()+gps3.satellites.value());
    
     
      // Establish current status
      double distanceToDestination = TinyGPSPlus::distanceBetween(location_lat, location_lng,LAT, LNG);
      double courseToDestination = TinyGPSPlus::courseTo( location_lat, location_lng, LAT, LNG);
      int courseChangeNeeded = (int)(360 + courseToDestination - bearing ) % 360; 

      file.print(courseToDestination);
      file.print(",");
      file.print(bearing);
      file.print(",");
      file.print(courseChangeNeeded);
      file.print(",");
      file.print(location_lat, 6);
      file.print(",");
      file.print(location_lng, 6);
      file.print(LAT, 6);
      file.print(",");
      file.print(LNG, 6);
      file.print(",");
      file.println(distanceToDestination);
      
      Serial.println(bearing);
      Serial.println(courseChangeNeeded);
      Serial.println(distanceToDestination);
      
      // Within 3.0m arrived at destination
      if (distanceToDestination <= 3.0)
      {
        Serial.println("ARRIVED");
        delay(3000);
        arrived = true;
        Serial.print(LAT);
        Serial.print(" , ");
        Serial.print(LNG);
        file.close();
        run(0, HIGH, LOW, HIGH, LOW, 0); 
        LAT = 0;
        return;
      }
 
      if (courseChangeNeeded >= 345 || courseChangeNeeded < 15){      
        run(200, HIGH, LOW, HIGH, LOW, 200);
        return; 
      }
      else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345){
        run(100, HIGH, LOW, HIGH, LOW, 180);
        return; 
      }
      else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45){
        run(180, HIGH, LOW, HIGH, LOW, 100);
        return; 
      }
      else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315){
        run(50, HIGH, LOW, HIGH, LOW, 180); 
        return;
      }
      else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105){
        run(180, HIGH, LOW, HIGH, LOW, 50);
        return;
      }
      else{
        run(200, LOW, HIGH, LOW, HIGH, 50);
        //To Do: Implement code for turning 180 degrees
        return;
      }
    }
  }
}

  //Run motors 
  void run(int L, int LF, bool LR, bool RF, bool RR, int R){
      analogWrite(PWMA, L); 
      digitalWrite(IN1, LF);
      digitalWrite(IN2, LR);
      digitalWrite(IN3, RF);
      digitalWrite(IN4, RR);
      analogWrite(PWMB, R);
  }
