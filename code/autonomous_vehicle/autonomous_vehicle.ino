#include <TinyGPS++.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>

#define GPSBaud 9600
#define ConsoleBaud 9600

// The serial connection to the GPS device
// The TinyGPS++ object
TinyGPSPlus gps1, gps2, gps3;

float location_lat;
float location_lng;

float speed_kmph;

unsigned long lastUpdateTime = 0;

// Create a compass
HMC5883L_Simple compass;

// varibles for bluetooth communications and get lattitute and longitude destination
String inData;
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
  
  // Begin I2C communication 
  Wire.begin();

  // Setting the comppas declination
  compass.SetDeclination(-21, 7, 'W'); 

  // Configuring the HMC5883L_Simple library
  compass.SetSamplingMode(COMPASS_SINGLE);
  compass.SetScale(COMPASS_SCALE_130);
  compass.SetOrientation(COMPASS_VERTICAL_Y_WEST);
   
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
    
    //reset command
    
    if (Serial.available()>0 && Serial.readString() == "r"){
      LAT = 0;
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

  // Every 2 seconds, do an update.
  if (millis() - lastUpdateTime >= 1000)
  {
    
      lastUpdateTime = millis();
      Serial.println();

      // average of the three gps modules readings
      location_lat = (gps1.location.lat() +gps2.location.lat()+gps3.location.lat())/3;
      location_lng = (gps1.location.lng() +gps2.location.lng()+gps3.location.lng())/3;
      location_lng = (gps1.location.lng() +gps2.location.lng()+gps3.location.lng())/3;
      speed_kmph = (gps1.speed.kmph() +gps2.speed.kmph()+gps3.speed.kmph())/3;
      
      // Establish current status
      double distanceToDestination = TinyGPSPlus::distanceBetween(
        location_lat, location_lng,LAT, LNG);
      double courseToDestination = TinyGPSPlus::courseTo(
        location_lat, location_lng, LAT, LNG);
      const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
      int courseChangeNeeded = (int)(360 + courseToDestination + compass.GetHeadingDegrees()) % 360; 
  
      Serial.print("  Course to Destination: ");
      Serial.print(courseToDestination);
      Serial.print("  Current Course: ");
      Serial.print(compass.GetHeadingDegrees());
      Serial.print("  Directions to Destination: ");
      Serial.print(directionToDestination);
      Serial.print("  Relative Course: ");
      Serial.print(courseChangeNeeded);
      Serial.print("  Speed: ");
      Serial.println(speed_kmph);
      Serial.print("  lattitude: ");
      Serial.print(location_lat, 6);
      Serial.print("  longitude: ");
      Serial.println(location_lng, 6);
      
  
      // Within 7.0m arrived at destination
      if (distanceToDestination <= 7.0)
      {
        Serial.println("CONGRATULATIONS: You've arrived!");
        exit(1);
      }
  
      Serial.print("DISTANCE: ");
      Serial.print(distanceToDestination);
      Serial.println(" meters to go.");
      Serial.print("INSTRUCTION: ");
  
      Serial.println(courseChangeNeeded);  
      if (courseChangeNeeded >= 345 || courseChangeNeeded < 15){      
        run(255, HIGH, LOW, HIGH, LOW, 255);
        return; 
      }
      else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345){
        run(255, HIGH, LOW, HIGH, LOW, 125);
        return; 
      }
      else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45){
        run(125, HIGH, LOW, HIGH, LOW, 255);
        return; 
      }
      else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315){
        run(255, HIGH, LOW, HIGH, LOW, 90); 
        return;
      }
      else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105){
        run(90, HIGH, LOW, HIGH, LOW, 255);
        return;
      }
      else{
        run(0, HIGH, LOW, HIGH,  LOW, 0);
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
