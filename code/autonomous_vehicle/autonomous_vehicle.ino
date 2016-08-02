#include <TinyGPS++.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>

#define GPSBaud 9600
#define ConsoleBaud 9600

// The serial connection to the GPS device
// The TinyGPS++ object
TinyGPSPlus gps;
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
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());

  // Every 2 seconds, do an update.
  if (millis() - lastUpdateTime >= 1000)
  {
      Serial.println(LAT,6);
      Serial.println(LNG,6);
      
      lastUpdateTime = millis();
      Serial.println();

  
      // Establish current status
      double distanceToDestination = TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(),LAT, LNG);
      double courseToDestination = TinyGPSPlus::courseTo(
        gps.location.lat(), gps.location.lng(), LAT, LNG);
      const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
      int courseChangeNeeded = (int)(360 + courseToDestination + compass.GetHeadingDegrees()) % 360; 
  
      Serial.print("DEBUG: Course2Dest: ");
      Serial.print(courseToDestination);
      Serial.print("  CurCourse: ");
      Serial.print(gps.course.deg());
      Serial.print("  Dir2Dest: ");
      Serial.print(directionToDestination);
      Serial.print("  RelCourse: ");
      Serial.print(courseChangeNeeded);
      Serial.print("  CurSpd: ");
      Serial.println(gps.speed.kmph());
      Serial.print("  lattitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print("  longitude: ");
      Serial.println(gps.location.lng(), 6);
      
  
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
