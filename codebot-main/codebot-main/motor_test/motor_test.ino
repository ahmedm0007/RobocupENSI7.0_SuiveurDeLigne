//macros 
/*# define M A2 //midsensor
# define L A3 //left sensor
# define R A1 //right sensor
# define LL A4 //extreme left sensor
# define RR A0 // extreme right sensor
# define ER 5 //echo right ultrason
# define EF 3 //echo front ultrason
# define TR 4  //trigger right ultrason
# define TF 2  //trigger front ultrason
# define LED_1 A5
# define LED_2 13
# define LED_3 0*/
#include <Servo.h>
#define MR1 5 //right motor forward 
#define MR2 4  //right motor backward 
#define MRS 3  //right motor speed
#define ML1 6 //left motor forward 
#define ML2 2 //left motor backward 
#define MLS 11 //left motor speed
Servo myservo;
int a=0;
int i=0;
// variables 
unsigned long currentTime;
unsigned long previousTime;
void setup() 
{ Serial.begin(9600);
  myservo.attach(9);
  myservo.write(45);
  pinMode (MR1 ,OUTPUT);
  pinMode (MR2 ,OUTPUT);
  pinMode (MRS,OUTPUT);
  pinMode (ML1 ,OUTPUT);
  pinMode (ML2,OUTPUT);
  pinMode (MLS,OUTPUT);
    /*pinMode (M ,INPUT);
  pinMode (L,INPUT);
  pinMode (R,INPUT);
  pinMode (LL,INPUT);
  pinMode (RR,INPUT);
  pinMode (ER,INPUT);
  pinMode (EF,INPUT);
  pinMode (TR,OUTPUT);
  pinMode (TF,OUTPUT);
  pinMode (LED_1,OUTPUT);
  pinMode (LED_2,OUTPUT);
  pinMode (LED_3,OUTPUT);*/
 
}
//fuctions 
void rotate_motor_backward(int pin1,int pin2,int pin_PWM,int U)
{
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,HIGH);
  analogWrite(pin_PWM,255*0.01*U);//pourcentage
}
void rotate_motor_forward(int pin1,int pin2,int pin_PWM,int U)
{
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  analogWrite(pin_PWM,255*0.01*U);//pourcentage
}
void loop() 
{
  /*currentTime = millis();
  if (currentTime - previousTime >= 5) 
  { 
    previousTime = currentTime;
  }*/
  
  //rotate_motor_forward(MR1,MR2,MRS,30);
  rotate_motor_forward(MR1,MR2,MRS,20);
  rotate_motor_forward(ML1,ML2,MLS,20);
 myservo.write(45);
 delay(2000);
myservo.write(0);
Serial.println("0");
 delay(2000);
 myservo.write(90);
 Serial.println("90");
 delay(2000);
  /*
  while (a<1){
  myservo.write(90);
  rotate_motor_forward(MR1,MR2,MRS,50);
  rotate_motor_forward(ML1,ML2,MLS,50);
  delay(2000);
  a++;}
  a=0;
  while (a<1)
  { myservo.write(0);
  rotate_motor_forward(MR1,MR2,MRS,50);
  rotate_motor_forward(ML1,ML2,MLS,50);
  delay(2000);
  a++;}*/
}