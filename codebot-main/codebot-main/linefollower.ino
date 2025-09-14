
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
// Variables
unsigned long currentTime;
unsigned long previousTime;
int L_Value, M_Value, R_Value,LL_Value,RR_Value;//for sensors values 
//for PID
int error, lastError, totalError;
double correction;
int leftSpeed, rightSpeed;
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
void readIR()
{
  L_Value = digitalRead(L);  
  M_Value = digitalRead(M);
  R_Value = digitalRead(R);  
  LL_Value = digitalRead(L); 
  RR_Value = digitalRead(R);
}
void PID(double KP,double KI,double KD) 
{
  error = (L_Value * 1) + (M_Value * 0) - (M_Value * 1); 
  totalError += error; // Compute PID correction
  correction= KP * error+KD * (error - lastError)+KI * totalError;//calculate correction
  lastError = error;// Save current error for next iteration
  // Calculate motor speeds
  leftSpeed = 255 - correction;
  rightSpeed = 255 + correction;
  // Constrain motor speeds to 0-255 range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  rotate_motor_forward(MR1,MR2,MRS,rightSpeed);
  rotate_motor_forward(ML1,ML2,MLS,leftSpeed);
}
void loop() 
{
  
}

