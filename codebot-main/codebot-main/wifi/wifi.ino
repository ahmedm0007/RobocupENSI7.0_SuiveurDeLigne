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
# define LED_3 1
#define MR1 8  //right motor forward 
#define MR2 7  //right motor backward 
#define MRS 6  //right motor speed
#define ML1 9 //left motor forward 
#define ML2 10 //right motor backward 
#define MLS 11 //left motor speed
unsigned B=LOW;
unsigned etatprecedent=LOW;
unsigned i=0;

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

void setup() {
   // process the states as required
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
}

void loop() {
    Serial.println(i);

    while(i<5){
          Serial.println(i);

    rotate_motor_forward(MR1,MR2,MRS,30);
    rotate_motor_forward(ML1,ML2,MLS,30);
    etatprecedent=B;
    B= digitalRead(M);
    if ( B==HIGH && B != etatprecedent ) 
      {
        i++ ; 
      }  
    }
         Serial.println(i);

    if (i==5){
      digitalWrite(LED_1,HIGH);
      rotate_motor_forward(MR1,MR2,MRS,0);
    rotate_motor_forward(ML1,ML2,MLS,0); 
    }   

}