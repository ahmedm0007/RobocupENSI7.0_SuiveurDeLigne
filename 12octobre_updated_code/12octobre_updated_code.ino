# define M 4 //midsensor
# define L 3 //left sensor
# define R 5 //right sensor
# define LL 7 //extreme left sensor
# define RR 2 // extreme right sensor
#define LED 8 //yellow led

#define RB 10  //right motor forward 
#define RF 9  //right motor backward 
#define RP 6  //right motor speed
#define LB 12 //left motor forward 
#define LF 13 //right motor backward 
#define LP 11 //left motor speed
float a=1.1;
int m;
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

void motor_forward()
{
rotate_motor_forward(RF,RB,RP,60);
rotate_motor_forward(LF,LB,LP,60);
}

void motor_stop()
{
rotate_motor_forward(RF,RB,RP,0);
rotate_motor_forward(LF,LB,LP,0);
}

void motor_right()
{
  rotate_motor_backward(RF, RB, RP, 60);
  rotate_motor_forward(LF, LB, LP, 60);
}

void motor_left()
{
  rotate_motor_forward(RF, RB, RP, 60);
  rotate_motor_backward(LF, LB, LP, 60);
}

void motor_backward()
{
rotate_motor_backward(RF,RB,RP,60);
rotate_motor_backward(LF,LB,LP,60);
}
typedef enum
{
  botIdle1,
  bfollower1,
  fHexagone,
  bfollower2,
  discontinued,
  bfollower3,
  corner,
  bfollower4,
  corner2,
  //stripe,
  bfollower5,
  bHexagone,
  interruption1,
  botIdle2,
  bfollower6,
  circle,
  bfollower7,
  stairs,
  bfollower8,
  wfollower,
  bfollower9,
  triangle,
  botIdle3,
  stop,
}Robot_States;
Robot_States ROBOT_CurrentState_en=botIdle1;
int compteur;
unsigned long currentTime;
unsigned long previousTime;
int L_Value, M_Value, R_Value,LL_Value,RR_Value;//for sensors values 
int Pre_LL_Value = LL_Value;
//for PIDwhite
int error=0, lastError=0, totalError=0;
double correction=0;
int leftSpeed, rightSpeed;
//for pid black
int Berror=0, BlastError=0, BtotalError=0;
double Bcorrection=0;
int BleftSpeed, BrightSpeed;
//discontinued variables
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
unsigned long cornerStartTime;

void readIR()//sensor readings
{
  L_Value = digitalRead(L);  
  M_Value = digitalRead(M);
  R_Value = digitalRead(R);  
  LL_Value = digitalRead(LL); 
  RR_Value = digitalRead(RR);
}

void bot_idle1fct()//first starting function
{
       //if(L_Value && M_Value && R_Value && LL_Value && RR_Value) 
      rotate_motor_forward(RF,RB,RP,50);
      rotate_motor_forward(LF,LB,LP,50);
      
}

int ir3()//only read the 3 middle ones
{
   L_Value = digitalRead(L);  
  M_Value = digitalRead(M);
  R_Value = digitalRead(R);  
  return(R_Value+10*M_Value+100*L_Value);
}

// PID functions
void PID(double KP,long KI,double KD) 
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
  BleftSpeed = 100 - Bcorrection; //45
  BrightSpeed = 100 + Bcorrection;//45
  // Constrain motor speeds to 0-255 range
  BleftSpeed = constrain(BleftSpeed, 0, 255);
  BrightSpeed = constrain(BrightSpeed, 0, 255);
rotate_motor_forward(RF,RB,RP,BrightSpeed);//rotate_motor_forward(MR1,MR2,MRS,BrightSpeed*1.42);
rotate_motor_forward(LF,LB,LP,BleftSpeed);

} 

void PIDhexagone(double KP,long KI,double KD) 
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
  BleftSpeed = 80 - Bcorrection; //45
  BrightSpeed = 80 + Bcorrection;//45
  // Constrain motor speeds to 0-255 range
  BleftSpeed = constrain(BleftSpeed, 0, 255);
  BrightSpeed = constrain(BrightSpeed, 0, 255);
rotate_motor_forward(RF,RB,RP,BrightSpeed);//rotate_motor_forward(MR1,MR2,MRS,BrightSpeed*1.42);
rotate_motor_forward(LF,LB,LP,BleftSpeed);
} 

void Blackfollower2()
{
  PIDhexagone(70,0.8,20);
}

void PIDwhite(double KP,double KI,double KD) 
{ 
  switch (static_cast<int>(ir3()))
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
  rotate_motor_forward(RF,RB,RP,rightSpeed);
  rotate_motor_forward(LF,LB,LP,leftSpeed);
}

void Blackfollower1()
{
  PID(70,0.8,20); //order(kp,ki,kd)
}

void WhiteFollower1()
{
  readIR();
  PIDwhite(30,0,0);//(kp,ki,kd)
} 

int calcul_compteur()
{  readIR();
   if (Pre_LL_Value!=LL_Value)
   {
    compteur++;
   }
   return (compteur);
}


void setup() {
  pinMode (M ,INPUT);
  pinMode (L,INPUT);
  pinMode (R,INPUT);
  pinMode (LL,INPUT);
  pinMode (RR,INPUT);
  pinMode(LED,OUTPUT);

  pinMode (RF ,OUTPUT);
  pinMode (RB ,OUTPUT);
  pinMode (RP,OUTPUT);
  pinMode (LF ,OUTPUT);
  pinMode (LB,OUTPUT);
  pinMode (LP,OUTPUT);
  Serial.begin(9600);
}

//the loop
void loop() {
digitalWrite(LED,LOW);

 { currentTime = millis();
  if (currentTime - previousTime >= 5) 
  { 
    switch (ROBOT_CurrentState_en)
    {
      case botIdle1:
        
        {// readIR();
          bot_idle1fct();
          readIR();
                  
       
          if((LL_Value==0) && (RR_Value==0))
            ROBOT_CurrentState_en =bfollower1;
        } break;


      case bfollower1:
      { digitalWrite(LED,LOW);
          Blackfollower1();
        if(M_Value==1 && RR_Value==1 )
           ROBOT_CurrentState_en = fHexagone;
      }break;


      case fHexagone:
      { compteur = 0;
       digitalWrite(LED,HIGH);
        delay(1000);
        Pre_LL_Value==LL_Value;
      motor_stop();
       while(calcul_compteur()<4)
       { 
        rotate_motor_forward(RF, RB, RP, 0);
        rotate_motor_forward(LF, LB, LP, 50);
       }
      motor_stop();
      compteur=0;
      while(LL_Value!=1)
      {
      Blackfollower2();
      }
      motor_stop();
      while(calcul_compteur()<3)
       { 
        motor_right();
       }
      motor_stop();
      ROBOT_CurrentState_en = stop;
      }break;

      
      
      case bfollower2:
      { digitalWrite(LED,LOW);
        Blackfollower1();
        if (LL_Value==0 && L_Value ==0 && M_Value==0 && R_Value ==0 && RR_Value==0)//(currentTime>=10000)
            ROBOT_CurrentState_en = discontinued;
      }break;

      
       case discontinued:
        { cpt_0=0;
            while(cpt_0<5)
        {
           motor_forward();
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
        Blackfollower1();
        if(RR_Value==1)// (currentTime>=14000)
            ROBOT_CurrentState_en = corner;
      }break;


      case corner:
      { 
      if (cornerStartTime == 0) {
        cornerStartTime = millis();}  // Start the timer when entering the corner state}
    rotate_motor_backward(RF, RB, RP, 60);
    rotate_motor_forward(LF, LB, LP, 60);

    if (((R_Value == 0) && (RR_Value == 0) && (M_Value == 1)) || 
        (millis() - cornerStartTime > 720))  // Example timeout after 2 seconds
    {
        ROBOT_CurrentState_en = bfollower4;
        cornerStartTime = 0;  // Reset the timer for the next corner
    }
      
      }break;
 
      
     
      case bfollower4://tbalbiz
      { 
        Blackfollower1();
        if(RR_Value==1)// (currentTime>=14000)
            ROBOT_CurrentState_en = corner2;
      }break; 
      
      case corner2:
      { 
      if (cornerStartTime == 0) {
        cornerStartTime = millis();}  // Start the timer when entering the corner state}
    rotate_motor_backward(RF, RB, RP, 60);
    rotate_motor_forward(LF, LB, LP, 60);

    if (((R_Value == 0) && (RR_Value == 0) && (M_Value == 1)) || 
        (millis() - cornerStartTime > 720))  // Example timeout after 2 seconds
    {
        ROBOT_CurrentState_en = bfollower5;
        cornerStartTime = 0;  // Reset the timer for the next corner
    }
      
      }break;
      

     case bfollower5:
     {
       Blackfollower1();
       if (currentTime>=18000)
            ROBOT_CurrentState_en = stop;
     }break;
      
      case bfollower8:
      {
        Blackfollower1();
        if(LL_Value==1 && RR_Value==1 && M_Value==0)
           ROBOT_CurrentState_en = wfollower;
      }
      
      case wfollower:
        { 
          WhiteFollower1();
          readIR();
          if (LL_Value==0 && RR_Value==0 && M_Value==1)
          {ROBOT_CurrentState_en =stop ;}

        }break;
      
      case bfollower9:
      {
        Blackfollower1();
        if (currentTime>= 20000)
          ROBOT_CurrentState_en = triangle;
      }

      case stop:
      {
        delay(250);
        rotate_motor_forward(RF,RB,RP,0);
         rotate_motor_forward(LF,LB,LP,0);
      }
    }
    previousTime = currentTime;
  }
}
}