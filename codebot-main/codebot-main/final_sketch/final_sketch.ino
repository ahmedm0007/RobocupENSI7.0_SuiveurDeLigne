# define M A2 //midsensor
# define L A3 //left sensor
# define R A1 //right sensor
# define LL A4 //extreme left sensor
# define RR A0 // extreme right sensor
# define EL 5 //echo right ultrason
# define TL 4  //trigger right ultrason
# define LED_1 12
# define LED_2 2
# define LED_3 3
#define MR1 8  //right motor forward 
#define MR2 7  //right motor backward 
#define MRS 6  //right motor speed
#define ML1 9 //left motor forward 
#define ML2 10 //right motor backward 
#define MLS 11 //left motor speed
 typedef enum
 {   bot_idle1,
 bfollower1,
 uturn,
 bfollower2,
 tunnel,
 bot_idle2,
 infinity,
 wifi,
 bfollower3,
 pitstop,
 bfollower4,
 bot_idle3,
 wfollower1,
 y,
 wfollower2,
 interruption1,
 bfollower5,
 interruption2,
 bot_idle4,
 stop,
 }Robot_States;
Robot_States ROBOT_CurrentState_en=bot_idle1;


// Variables
unsigned long currentTime;
unsigned long previousTime;
int L_Value, M_Value, R_Value,LL_Value,RR_Value;//for sensors values 
//for PID
int error=0, lastError=0, totalError=0;
double correction=0;
int leftSpeed, rightSpeed;
//wifi variables
u_int etatactuel_0=0;
 u_int etatprecedent_0=0;
 u_int cpt_0=0;
// interruption1 variables
u_int etatactuel_1=0;
 u_int etatprecedent_1=0;
 u_int cpt_1=0;
// interruption2 variables
u_int etatactuel_2=0;
 u_int etatprecedent_2=0;
 u_int cpt_2=0;



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
  pinMode (EL,INPUT);
  pinMode (TL,OUTPUT);
  pinMode (LED_1,OUTPUT);
  pinMode (LED_2,OUTPUT);
  pinMode (LED_3,OUTPUT);
  Serial.begin(1200);
}

void rotate_motor_backward(int pin1,int pin2,int pin_PWM,int U)
{
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,HIGH);
  analogWrite(pin_PWM,U);//pourcentage
}
void rotate_motor_forward(int pin1,int pin2,int pin_PWM,int U)
{
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  analogWrite(pin_PWM,U);//pourcentage
}
void readIR()
{
  L_Value = digitalRead(L);  
  M_Value = digitalRead(M);
  R_Value = digitalRead(R);  
  LL_Value = digitalRead(LL); 
  RR_Value = digitalRead(RR);
}
void bot_idle1fct()
{
  readIR();
       if(L_Value && M_Value && R_Value && LL_Value && RR_Value)
      {
      delay(500);
      rotate_motor_forward(MR1,MR2,MRS,50);
      rotate_motor_forward(ML1,ML2,MLS,50);
      }
}

void bot_idle3fct()
{
  rotate_motor_forward(MR1,MR2,MRS,50);
  rotate_motor_forward(ML1,ML2,MLS,50);
}//condition de passage to the other state in loop ()

int ir3()
{
  return(100*L_Value + 10*M_Value+R_Value);
}
void PIDwhite(double KP,double KI,double KD) 
{ 
  switch (ir3())
  {
    case 111 :
         error=lastError;
         break;
    case 110:
         error=-2;
         break;
    case 101:
         error=0;
         break;
    case 100:
         error=-1;
         break;
    case 11:
         error=2;
         break;
    case 1:
         error=1;
         break;
    } 
  totalError += error; // Compute PID correction
  correction= KP * error + KD * (error - lastError) + KI * totalError;//calculate correction
  lastError = error;// Save current error for next iteration
  // Calculate motor speeds
  leftSpeed = 100 - correction;
  rightSpeed = 100 + correction;
  // Constrain motor speeds to 0-255 range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  rotate_motor_forward(MR1,MR2,MRS,rightSpeed);
  rotate_motor_forward(ML1,ML2,MLS,leftSpeed);
}
void WhiteFollower1()
{
  readIR();
  PIDwhite(50,0,0);
}
//codition passage wf1 to Y : RR=0
void Y_fct()
{
  rotate_motor_forward(MR1,MR2,MRS,0);
  rotate_motor_forward(ML1,ML2,MLS,50);
  readIR();
} 
//condition Y to 2() : RR=1 & LL=1
void WhiteFollower2()
{
  readIR();
  PIDwhite(50,0,0);
}
// condition out  RR=0 & LL=0 
void pit_stopfct()
{
            delay(250);
           rotate_motor_forward(MR1,MR2,MRS,0);
            rotate_motor_forward(ML1,ML2,MLS,0);
            digitalWrite(LED_1, HIGH);
            delay(2000);
            digitalWrite(LED_2, HIGH);
            delay(2000);
            digitalWrite(LED_3, HIGH);
            delay(1000);
            digitalWrite(LED_1, LOW);
            digitalWrite(LED_2, LOW);
            digitalWrite(LED_3, LOW);
            rotate_motor_forward(MR1,MR2,MRS,100);
            rotate_motor_forward(ML1,ML2,MLS,100);
            delay(250);
}


  


 void loop()
 { currentTime = millis();
  if (currentTime - previousTime >= 5) 
  { 
    switch (ROBOT_CurrentState_en)
    {
      case bot_idle1:
        
        {
          bot_idle1fct();
          readIR();
          if(L_Value && M_Value && R_Value && LL_Value && RR_Value)
                ROBOT_CurrentState_en =bfollower1;
        } break                                                                                                                                                                
      case bfollower1:
      case uturn:
      case bfollower2:
      case tunnel:
      case bot_idle2:
      case infinity:


      case wifi:
        { cpt_0=0
            while(cpt_0<5)
        {
           rotate_motor_forward(MR1,MR2,MRS,50);
           rotate_motor_forward(ML1,ML2,MLS,50);
           etatprecedent_0=etatactuel_0;
           etatactuel_0 = digitalRead(M);
          if ( etatactuel_0==HIGH && etatactuel_0!= etatprecedent_0 ) 
          {
            cpt_0++ ; 
          }  
        }
      
           if (cpt_0==5)
             ROBOT_CurrentState_en=bfollower3;
        }  

        case bfollower3:
        
  case pitstop:
        {
        pit_stopfct();
          readIR();
          if  ((LL_Value==LOW) || (RR_Value==LOW))
                ROBOT_CurrentState_en =bfollower4;
          }break;

          case bfollower4:
        {
          //bfollower4 fct
          ROBOT_CurrentState_en =bot_idle3;
        }break;

        case bot_idle3:
        {
          bot_idle3fct();
          readIR();
          if(M_Value==LOW)
          ROBOT_CurrentState_en =wfollower1;
        } break;

        case wfollower1:
        {
          WhiteFollower1();
          readIR();
          if(RR_Value==LOW)
          {
             ROBOT_CurrentState_en =y;
          } 
        }break;

        case y:
        {
          Y_fct();
          if(RR_Value==HIGH && LL_Value==LOW)
          {
            ROBOT_CurrentState_en =wfollower2;
          }
        } break;
         
         case wfollower2:
        {
          WhiteFollower2();
          readIR();
          if(RR_Value==LOW && LL_Value==LOW)
          {
             ROBOT_CurrentState_en =interruption1;
          } 
        }break;
        
      case interruption1:
      { 
             cpt_1=0;
               while(cpt_1<2)
               {rotate_motor_forward(MR1,MR2,MRS,50);
                rotate_motor_forward(ML1,ML2,MLS,50);
                   etatprecedent_1=etatactuel_1;
                    etatactuel_1= digitalRead(M);
                     if ( etatactuel_1==1 && etatactuel_1 != etatprecedent_1 ) 
                        cpt_1++ ; 
                         
                         }
        if (cpt_1==2) {
          ROBOT_CurrentState_en=bfollower5;
          }
      }

      case bfollower5:

      case interruption2:
      {
        cpt_2=0;
               while(cpt_2<2)
               {rotate_motor_forward(MR1,MR2,MRS,50);
                rotate_motor_forward(ML1,ML2,MLS,50);
                   etatprecedent_2=etatactuel_2;
                    etatactuel_2= digitalRead(M);
                     if ( etatactuel_2==1 && etatactuel_2 != etatprecedent_2 ) 
                        cpt_2++ ; 
                         
                         }
        if (cpt_2==2) {
          ROBOT_CurrentState_en=bot_idle4;
          }
      }
      case bot_idle4:
      {
                rotate_motor_forward(MR1,MR2,MRS,50);
                rotate_motor_forward(ML1,ML2,MLS,50);
                readIR();
                if(L_Value && M_Value && R_Value && LL_Value && RR_Value)
                ROBOT_CurrentState_en=stop;
      }
      case stop:
      {
        delay(250);
        rotate_motor_forward(MR1,MR2,MRS,0);
         rotate_motor_forward(ML1,ML2,MLS,0);

      }

       

      


      


      } Default:{}
       previousTime = currentTime;
  }
}