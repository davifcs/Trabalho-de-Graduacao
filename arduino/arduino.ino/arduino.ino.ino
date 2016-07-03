#include <TinyGPS++.h>


int PWMA = 7;
int IN1 = 6;
int IN2 = 5;
int IN3 = 4;
int IN4 = 3;
int PWMB = 2; 

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;




void setup()
{
 //Define Baud para Serial monitor e GPS
 Serial.begin(115200);
 Serial1.begin(GPSBaud);
 
  //Define os pinos como saida
 pinMode(PWMA, OUTPUT);
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(PWMB, OUTPUT);

}
  
void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo();
    
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  displayInfo();
  distance();
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void distance()
{


const double Destiny_Lat = -23.664691;
const double Destiny_Log = -46.686514;
double distance_m =
  TinyGPSPlus().distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    Destiny_Lat ,
    Destiny_Log);
double courseTo =
  TinyGPSPlus().courseTo(
    gps.location.lat(),
    gps.location.lng(),
    Destiny_Lat ,
    Destiny_Log);



if(distance_m > 7.0){
 //Serial.println(distance_m);
 //Gira o Motor A no sentido horario
 analogWrite(PWMA, 140); 
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 analogWrite(PWMB, 140);
 Serial.println(distance_m);
}
else{
 digitalWrite(PWMA, LOW); 
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, LOW);
 digitalWrite(PWMB, LOW);
}


    
Serial.print("Distance (km) to Destiny ");
Serial.println(distance_m);
Serial.print("Course to Destiny: ");
Serial.println(courseTo);
Serial.print("Human directions: ");
Serial.println(TinyGPSPlus().cardinal(courseTo));



}







 //Gira o Motor A no sentido horario


