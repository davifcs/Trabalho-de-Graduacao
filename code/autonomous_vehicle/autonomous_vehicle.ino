#include <TinyGPS++.h>
#define GPSBaud 9600
#define ConsoleBaud 115200

// The serial connection to the GPS device
// The TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;


#define LAT -23.664674
#define LNG -46.686737
/* This example shows a basic framework for how you might
   use course and distance to guide a person (or a drone)
   to a destination.  This destination is the Eiffel Tower.
   Change it as required.  

   The easiest way to get the lat/long coordinate is to 
   right-click the destination in Google Maps (maps.google.com),
   and choose "What's here?".  This puts the exact values in the 
   search box.
*/
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
  
  //
  Serial.begin(ConsoleBaud);
  Serial1.begin(GPSBaud);
}

void loop()
{
  // If any characters have arrived from the GPS,
  // send them to the TinyGPS++ object
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());

  // Every 5 seconds, do an update.
  if (millis() - lastUpdateTime >= 5000)
  {
    lastUpdateTime = millis();
    Serial.println();

    // Establish our current status
    double distanceToDestination = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(),LAT, LNG);
    double courseToDestination = TinyGPSPlus::courseTo(
      gps.location.lat(), gps.location.lng(), LAT, LNG);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360; 
    Serial.println(gps.course.deg());   
    // debug
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

    // Within 20 meters of destination?  We're here!
    if (distanceToDestination <= 7.0)
    {
      Serial.println("CONGRATULATIONS: You've arrived!");
      exit(1);
    }

    Serial.print("DISTANCE: ");
    Serial.print(distanceToDestination);
    Serial.println(" meters to go.");
    Serial.print("INSTRUCTION: ");

    // Standing still? Just indicate which direction to go.
//    if (gps.speed.kmph() < 2.0)     
//    {       
//      Serial.print("Head ");         
//      Serial.print(directionToDestination);       
//      Serial.println(".");       
//      return;     
//    }           
    
    Serial.println(courseChangeNeeded);  
    if (courseChangeNeeded >= 345 || courseChangeNeeded < 15){      
      run(200, HIGH, LOW, HIGH, LOW, 200);
      return; 
    }
    else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345){
      run(100, HIGH, LOW, HIGH, LOW, 200);
      return; 
    }
    else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45){
      run(200, HIGH, LOW, HIGH, LOW, 100);
      return; 
    }
    else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315){
      run(70, HIGH, LOW, HIGH, LOW, 200); 
      return;
    }
    else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105){
      run(200, HIGH, LOW, HIGH, LOW, 70);
      return;
    }
    else{
      run(0, HIGH, LOW, HIGH,  LOW, 0);
      return;
    }
  }
}


  void run(int L, int LF, bool LR, bool RF, bool RR, int R){
      analogWrite(PWMA, L); 
      digitalWrite(IN1, LF);
      digitalWrite(IN2, LR);
      digitalWrite(IN3, RF);
      digitalWrite(IN4, RR);
      analogWrite(PWMB, R);
  }
