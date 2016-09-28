
#include <SPI.h>
#include <SD.h>

uint8_t gps_config_change[129]={
//Remove Message
//GLL
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B,
//GSA
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32,
//GSV
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39,
//RMC
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40,
//VTG
0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47,

//Baud rate (115200)

0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E,

//Save
0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB }; 


String inputString = "";         // a string to hold incoming data
String inputString1 = "";         // a string to hold incoming data
String inputString2 = "";         // a string to hold incoming data
String inputString3 = "";         // a string to hold incoming data
int len1 = 0;
int len2 = 0;
int len3 = 0;

boolean stringComplete= false;  // whether the string is complete
boolean stringComplete1= false;  // whether the string is complete
boolean stringComplete2 = false;  // whether the string is complete
boolean stringComplete3 = false;  // whether the string is complete

File file;
unsigned int filenumber = 1;
String fileName = String();
boolean start = false;
boolean over = false;
String message = String();
boolean gotogo = false;
boolean wait = false;

// Define pinout for the L298N board
int PWMA = 7;
int IN1 = 6;
int IN2 = 5;
int IN3 = 4;
int IN4 = 3;
int PWMB = 2; 

void setup() {

  pinMode(8, INPUT);
  digitalWrite(8, HIGH); // pull-up
  
  Serial.begin(9600);
 
  Serial1.begin(9600);
  Serial1.write(gps_config_change,sizeof(gps_config_change));
  Serial1.end();
  Serial1.begin(115200);

  Serial2.begin(9600);
  Serial2.write(gps_config_change,sizeof(gps_config_change));
  Serial2.end();
  Serial2.begin(115200);

  Serial3.begin(9600);
  Serial3.write(gps_config_change,sizeof(gps_config_change));
  Serial3.end();
  Serial3.begin(115200);

   inputString.reserve(200);
   inputString2.reserve(200);
   inputString3.reserve(200);


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

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void loop() {
  // print the string when a newline arrives:

  len1 = inputString1.length() - '0';
  len2 = inputString2.length() - '0';
  len3 = inputString3.length() - '0';

  if (len1 > 0 && len2 > 0 && len3 > 0 && start == false && gotogo == false){
    len1 = 0;
    len2 = 0;
    len3 = 0;
    delay(40000);
    gotogo = true;
    Serial.println("Ready");
  }

  if(Serial.available()>0 && Serial.read()=='s'){
     start = true;
     over = false;
     wait = false;
  inputString1 = "";
  inputString2 = "";
  inputString3 = "";
  }

  if (Serial.available()>0 && Serial.read()=='o'  ||  !digitalRead(8)){
    over = true;
    run(0, HIGH, LOW, HIGH, LOW, 0);
    file.close();
    Serial.println("File closed");
    }  
  
  if(stringComplete1 && stringComplete2 && stringComplete3 && file && over == false && start == true ) {

    Serial.println(inputString1.length());
    Serial.println(inputString2.length());
    Serial.println(inputString3.length());
    file.print("1-");
    file.print(inputString1);
    file.flush();
    // clear the string:
    inputString1 = "";
    stringComplete = false;
    len1 = 0;
    
    file.print("2-");
    file.print(inputString2);
    file.flush();
    // clear the string:
    inputString2 = "";
    stringComplete2 = false;
    len2 = 0;
    
    file.print("3-");
    file.print(inputString3);
    file.flush();
    // clear the string:
    inputString3 = "";
    stringComplete3 = false;
    len3 = 0;

    run(200, HIGH, LOW, HIGH, LOW, 201);

  }
}
 
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar1 = (char)Serial1.read();
    // add it to the inputString:
    inputString1 += inChar1;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar1 == '\n') {
      stringComplete1 = true;
      
    }
  }
}

void serialEvent2() {
  while (Serial2.available()) {
    // get the new byte:
    char inChar2 = (char)Serial2.read();
    // add it to the inputString:
    inputString2 += inChar2;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar2 == '\n') {
      stringComplete2 = true;
    }
  }
}


void serialEvent3() {
  while (Serial3.available()) {
    // get the new byte:
    char inChar3 = (char)Serial3.read();
    // add it to the inputString:
    inputString3 += inChar3;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar3 == '\n') {
      stringComplete3 = true;
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
