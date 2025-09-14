# define M A2 //midsensor
# define L A3 //left sensor
# define R A1 //right sensor
# define LL A4 //extreme left sensor
# define RR A0 // extreme right sensor
# define EL 5 //echo right ultrason
# define TL 4  //trigger right ultrason
# define LED_1 12
# define LED_2 2
# define LED_3 0
#define MR1 8  //right motor forward 
#define MR2 7  //right motor backward 
#define MRS 11  //right motor speed
#define ML1 9 //left motor forward 
#define ML2 10 //right motor backward 
#define MLS 6 //left motor speed
 typedef enum
 {  
 bot_idle1,
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
Robot_States ROBOT_CurrentState_en=tunnel;


// Variables
unsigned long currentTime;
unsigned long previousTime;
int L_Value, M_Value, R_Value,LL_Value,RR_Value;//for sensors values 
//for PIDwhite
int error=0, lastError=0, totalError=0;
double correction=0;
int leftSpeed, rightSpeed;
//for pid black
int Berror=0, BlastError=0, BtotalError=0;
double Bcorrection=0;
int BleftSpeed, BrightSpeed;
//wifi variables
int etatactuel_0=0;
int etatprecedent_0=0;
int cpt_0=0;
// interruption1 variables
int etatactuel_1=0;
int etatprecedent_1=0;
int cpt_1=0;
// interruption2 variables
int etatactuel_2=0;
int etatprecedent_2=0;
int cpt_2=0;
//tunnel var
float left_dist;
float front_dist;
float I,P,D,PID_value,previous_I,previous_error,terror;
float right_motor_speed,left_motor_speed;                   

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
 // non PID functions 
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
void motor_control()
{
  //right_motor_speed=100+PID_value;
  //left_motor_speed=100-PID_value;
  right_motor_speed=60+PID_value;//100
  left_motor_speed=60-PID_value;//100  
  rotate_motor_forward(MR1,MR2,MRS,right_motor_speed*1.42);//1.42
  rotate_motor_forward(ML1,ML2,MLS,left_motor_speed);
  
}
void left_ult_dist()
{
  float duration;
  digitalWrite(TL, LOW);
  delayMicroseconds(2);
  digitalWrite(TL, HIGH);
  delayMicroseconds(3);
  digitalWrite(TL, LOW);

  duration = pulseIn(EL, HIGH);
  left_dist = 10*(duration*.0343)/2;
}

void erreur_function(){
  left_ult_dist();
  terror =left_dist+58-150;//150
}

// PID functions
void PID(double KP,double KI,double KD) 
{ 
  switch(ir3())
  {
    case 0 :
         Berror=BlastError;
         break;
    case 1:
         Berror=-2;//negative-->right turn
         break;
    case 10:
         Berror=0;
         break;
    case 11:
         Berror=-1;
         break;
    case 100:
         Berror=2;//positive-->left turn
         break;
    case 110:
         Berror=1;
         break;
    } 
  BtotalError += Berror; // Compute PID correction
  Bcorrection= KP * Berror + KD * (Berror - BlastError) + KI * BtotalError;//calculate correction
  BlastError = Berror;// Save current error for next iteration
  // Calculate motor speeds
  BleftSpeed = 45- Bcorrection; //100
  BrightSpeed = 45+ Bcorrection;
  // Constrain motor speeds to 0-255 range
  BleftSpeed = constrain(BleftSpeed, 0, 255);
  BrightSpeed = constrain(BrightSpeed, 0, 255);
rotate_motor_forward(MR1,MR2,MRS,BrightSpeed*1.42);
rotate_motor_forward(ML1,ML2,MLS,BleftSpeed);
readIR();
} 

void PIDbf2(double KP,double KI,double KD) 
{ 
  switch(ir3())
  {
    case 0 :
         Berror=BlastError;
         break;
    case 1:
         Berror=-2;//negative-->right turn
         break;
    case 10:
         Berror=0;
         break;
    case 11:
         Berror=-1;
         break;
    case 100:
         Berror=2;//positive-->left turn
         break;
    case 110:
         Berror=1;
         break;
    } 
  BtotalError += Berror; // Compute PID correction
  Bcorrection= KP * Berror + KD * (Berror - BlastError) + KI * BtotalError;//calculate correction
  BlastError = Berror;// Save current error for next iteration
  // Calculate motor speeds
  BleftSpeed = 45- Bcorrection; //60 
  BrightSpeed = 45+ Bcorrection;
  // Constrain motor speeds to 0-255 range
  BleftSpeed = constrain(BleftSpeed, 0, 255);
  BrightSpeed = constrain(BrightSpeed, 0, 255);
rotate_motor_forward(MR1,MR2,MRS,BrightSpeed*1.42);
rotate_motor_forward(ML1,ML2,MLS,BleftSpeed);
readIR();
} 

void PIDU(double KP,double KI,double KD) 
{ 
  switch(ir3())
  {
    case 0 :
         Berror=BlastError;
         break;
    case 1:
         Berror=-2;//negative-->right turn
         break;
    case 10:
         Berror=0;
         break;
    case 11:
         Berror=-1;
         break;
    case 100:
         Berror=2;//positive-->left turn
         break;
    case 110:
         Berror=1;
         break;
    } 
  BtotalError += Berror; // Compute PID correction
  Bcorrection= KP * Berror + KD * (Berror - BlastError) + KI * BtotalError;//calculate correction
  BlastError = Berror;// Save current error for next iteration
  // Calculate motor speeds
  BleftSpeed = 35- Bcorrection;//40 yemchi wykoss //80 for the infinity state 100 for the rest of states //45 y3addi eddoura
  BrightSpeed = 35+ Bcorrection;
  // Constrain motor speeds to 0-255 range
  //BleftSpeed = constrain(BleftSpeed, 0, 255);
  //BrightSpeed = constrain(BrightSpeed, 0, 255);
  if (Berror==1 && currentTime>=9000 ){
  rotate_motor_forward(MR1,MR2,MRS,65);
  rotate_motor_backward(ML1,ML2,MLS,45);
  //delay(200);
  }
  else if(BleftSpeed>=0 && BrightSpeed>=0)
  {
    rotate_motor_forward(MR1,MR2,MRS,BrightSpeed*1.42);
    rotate_motor_forward(ML1,ML2,MLS,BleftSpeed);
  }
  else if (BleftSpeed<0 && BrightSpeed>=0)
  {
    
    rotate_motor_backward(MR1,MR2,MRS,abs(BrightSpeed*1.42));
    rotate_motor_forward(ML1,ML2,MLS,BleftSpeed);
  }
  else if (BleftSpeed>=0 && BrightSpeed<0)
  {
  rotate_motor_forward(MR1,MR2,MRS,70);
  rotate_motor_backward(ML1,ML2,MLS,40);
  }


readIR();
}
void pid_value(float kp,float ki,float kd){
  erreur_function();
  if (terror>=300)
  {
    terror=200;
  }
  if(terror>-10 && terror<10)
  {
    terror=0;
  }
  I = I + previous_I;
  D = terror-previous_error;
  P=terror;
  PID_value = (kp*P) + (ki*I) + (kd*D);
    
  previous_I=I;
  previous_error=terror;
}

void PIDinfinity(double KP,double KI,double KD) 
{ 
  switch(ir3())
  {
    case 0 :
         Berror=BlastError;
         break;
    case 1:
         Berror=-2;//negative-->right turn
         break;
    case 10:
         Berror=0;
         break;
    case 11:
         Berror=-1;
         break;
    case 100:
         Berror=2;//positive-->left turn
         break;
    case 110:
         Berror=1;
         break;
    } 
  BtotalError += Berror; // Compute PID correction
  Bcorrection= KP * Berror + KD * (Berror - BlastError) + KI * BtotalError;//calculate correction
  BlastError = Berror;// Save current error for next iteration
  // Calculate motor speeds
  BleftSpeed = 80- Bcorrection; //80 for the infinity state 100 for the rest of states 
  BrightSpeed = 80+ Bcorrection;
  // Constrain motor speeds to 0-255 range
  BleftSpeed = constrain(BleftSpeed, 0, 255);
  BrightSpeed = constrain(BrightSpeed, 0, 255);
rotate_motor_forward(MR1,MR2,MRS,BrightSpeed*1.065);
rotate_motor_forward(ML1,ML2,MLS,BleftSpeed);
} 

void PIDwhite(double KP,double KI,double KD) 
{ 
  switch (ir3())
  {
    case 111 :
         error=lastError;
         break;
    case 110:
         error=-2;//2
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
  leftSpeed = 100 - correction;//100
  rightSpeed = 100 + correction;//100
  // Constrain motor speeds to 0-255 range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  rotate_motor_forward(MR1,MR2,MRS,rightSpeed);
  rotate_motor_forward(ML1,ML2,MLS,leftSpeed);
}


// states fuctions 
void bot_idle1fct()
{
  readIR();
       //if(L_Value && M_Value && R_Value && LL_Value && RR_Value)
      
      rotate_motor_forward(MR1,MR2,MRS,60);
      rotate_motor_forward(ML1,ML2,MLS,60);
      
}

void Blackfollower1()
{
  readIR();
  PID(20,0,0); //20
  readIR();

}

void U()
{
  readIR();
  PIDU(12,0,0); //12
}

void tunnelfct()
{
  pid_value(0.4,0,0.15);
  motor_control();
  Serial.println(left_dist);
  Serial.print("r");
  Serial.println(right_motor_speed);
  Serial.print("l");
  Serial.println(left_motor_speed);

}
void infinityturn()
{
  readIR();
  PIDinfinity(50,0,0);
}

void Blackfollower3()
{
  readIR();
  PID(60,0,0); 
}

void Blackfollower4()
{
  readIR();
  PID(60,0,0); 
}
void Blackfollower5()
{
  readIR();
  PID(60,0,0); 
}

void bot_idle3fct()
{
  rotate_motor_forward(MR1,MR2,MRS,85);
  rotate_motor_forward(ML1,ML2,MLS,50);
  delay(25);
}//condition de passage to the other state in loop ()

int ir3()
{
  return(100*L_Value + 10*M_Value+R_Value);
}

void WhiteFollower1()
{ 
  readIR();
  if (RR_Value==0)
   {Y_fct();}
  PIDwhite(60,0,0);
}
//codition passage wf1 to Y : RR=0
void Y_fct()
{ /*readIR();
  while(LL_Value&&)//compteur 
 { rotate_motor_forward(MR1,MR2,MRS,0);
  rotate_motor_forward(ML1,ML2,MLS,50);
  readIR();}*/
  cpt_2=0;
               while(cpt_2<1)
               {rotate_motor_forward(MR1,MR2,MRS,0);
                 rotate_motor_forward(ML1,ML2,MLS,50);
                   etatprecedent_2=etatactuel_2;
                    etatactuel_2= digitalRead(LL);
                     if ( etatactuel_2==1 && etatactuel_2 != etatprecedent_2 ) 
                        cpt_2++ ; 
                         
                         }
} 
void WhiteFollower2()
{
  readIR();
  PIDwhite(60,0,0);
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

// main code 
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
          if((LL_Value==0) && (RR_Value==0))
            ROBOT_CurrentState_en =bfollower1;
        } break;
      
      case bfollower1:
      { 
        Blackfollower1();
        if (currentTime>=7500)
            ROBOT_CurrentState_en =uturn;
      }break;
    
      case uturn:
      {
        digitalWrite(LED_2, HIGH);
        readIR();
        U();
        //left_ult_dist();
        //if (left_dist<150 && LL_Value==0 && L_Value==0 && M_Value==0 && R_Value==0 && RR_Value==0 )
        if (left_dist<150 && LL_Value==0 && L_Value==0 && M_Value==0 && R_Value==0 && RR_Value==0)
            ROBOT_CurrentState_en =tunnel;//uturn to turn toll
      }break;
      case tunnel:
      { 
        digitalWrite(LED_1,HIGH);
        tunnelfct();
        readIR();
        if (LL_Value || L_Value || M_Value || R_Value || RR_Value)
          ROBOT_CurrentState_en =tunnel;
      }break;
      case bot_idle2:
      {

      }break;
      case infinity:
      {
          infinityturn();
          if (LL_Value==0 && L_Value ==0 && M_Value==0 && R_Value ==0 && RR_Value==0)
            ROBOT_CurrentState_en =wifi;
      }break;
       
      case wifi:
        { cpt_0=0;
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
      {
        Blackfollower3();
        if (LL_Value && L_Value && M_Value && R_Value && RR_Value)
            ROBOT_CurrentState_en =pitstop;
      }
        
      case pitstop:
        {
        pit_stopfct();
          readIR();
          if  ((LL_Value==LOW) || (RR_Value==LOW))
                ROBOT_CurrentState_en =bfollower4;
          }break;

      case bfollower4:
        {  Blackfollower4();
          if (RR_Value)
            { 
              ROBOT_CurrentState_en =wfollower1;}
        }break;

        case bot_idle3:
        {
          bot_idle3fct();
          readIR();
          if(LL_Value && RR_Value==1)
          { delay(100);
          ROBOT_CurrentState_en =wfollower1;}
        } break;

        case wfollower1:
        { 
          digitalWrite(LED_2,HIGH);
          WhiteFollower1();
          readIR();
          if (LL_Value==0 && RR_Value==0&&(L_Value==0||R_Value==0))
          {ROBOT_CurrentState_en =stop ;}

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
      {
        Blackfollower5();
        if (LL_Value && L_Value && M_Value && R_Value && RR_Value)
            ROBOT_CurrentState_en =stop; 
      }

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
      } 
      
      Default:{}
       previousTime = currentTime;
  }
}