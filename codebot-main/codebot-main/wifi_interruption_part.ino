#include <GOTStateMachine.h>
//macros 
# define M A2 //midsensor
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
# define LED_3 0
#define MR1 8  //right motor forward 
#define MR2 7  //right motor backward 
#define MRS 6  //right motor speed
#define ML1 9 //left motor forward 
#define ML2 10 //right motor backward 
#define MLS 11 //left motor speed
 
GOTStateMachine stateMachine(50); // execute every 50 milliseconds

//states 
void bot_idle1();
void bfollower1();
void uturn();
void bfollower2();
void tunnel();
void bot_idle2();
void infinity();
void wifi();
void bfollower3();
void pitstop();
void bfollower4();
void bot_idle3();
void wfollower();
void y();
void interruption1();
void bfollower5();
void interruption2();
void bot_idle4();
void stop();

// variables 
unsigned long currentTime;
unsigned long previousTime;
u_int B=0;
 u_int etatprecedent=0;
 u_int i=0;
 
void setup() 
{
  pinMode (MR1 ,OUTPUT);
  pinMode (MR2 ,OUTPUT);
  pinMode (MRS,OUTPUT);
  pinMode (ML1 ,OUTPUT);
  pinMode (ML2,OUTPUT);
  pinMode (MLS,OUTPUT);
  pinMode (M ,INPUT);
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
  pinMode (LED_3,OUTPUT);
 Serial.begin(1200);

  stateMachine.setStartState(bot_idle1); // Initialise state machine
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
 // rotate_motor(MR1,MR2,MRS,100);
  //rotate_motor(ML1,ML2,MLS,100);
  stateMachine.execute();             // process the states as required

}
 
void wifi()
{
  rotate_motor_forward(MR1,MR2,MRS,200);
  rotate_motor_forward(ML1,ML2,MLS,200);
 while(i<5)
 {etatprecedent=B;
  B= digitalRead(M);
  if ( B==1 && B != etatprecedent ) 
  {
    i++ ; 

  }
   if (i==5) {

    stateMachine.changeState(bfollower3);
    return;}
 }
  
}

void interruption1()
{ i=0;
  rotate_motor_forward(MR1,MR2,MRS,200);
  rotate_motor_forward(ML1,ML2,MLS,200);
  while(i<2)
 {etatprecedent=B;
  B= digitalRead(M);
  if ( B==1 && B != etatprecedent ) 
  {
    i++ ; 

  }
   if (i==2) {

    stateMachine.changeState(bfollower5);
    return;}
 }
}

void interruption2()
{ i=0;
  rotate_motor_forward(MR1,MR2,MRS,200);
  rotate_motor_forward(ML1,ML2,MLS,200);
  while(i<2)
 {etatprecedent=B;
  B= digitalRead(M);
  if ( B==1 && B != etatprecedent ) 
  {
    i++ ; 

  }
   if (i==2 && stateMachine.isDelayComplete(1000) )
   {
    stateMachine.changeState(bot_idle4);
    return;}
 }
}
