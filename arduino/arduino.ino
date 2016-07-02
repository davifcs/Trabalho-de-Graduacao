int PWMA = 7;
int IN1 = 6;
int IN2 = 5;
int IN3 = 4;
int IN4 = 3;
int PWMB = 2; 
void setup()
{
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
 //Gira o Motor A no sentido horario
 digitalWrite(PWMA, HIGH); 
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 digitalWrite(PWMB, HIGH);
}

