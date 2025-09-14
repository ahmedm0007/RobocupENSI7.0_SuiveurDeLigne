//macros 
# define SENSOR_mid A2
# define SENSOR_left A3
# define SENSOR_right A1
# define SENSOR_left_front  A4
# define SENSOR_right_front A0
# define ECHO_PIN_right A6
# define ECHO_PIN_front A7
# define TRIG_PIN_right A5
# define TRIG_PIN_front 2
# define LED_1 3
# define LED_2 4
# define LED_3 5
#define MOTOR_right_pin_sens_1 8
#define MOTOR_right_pin_sens_2 7
#define MOTOR_right_pin_PWM 6
#define MOTOR_left_pin_sens_1 9 
#define MOTOR_left_pin_sens_2 10
#define MOTOR_left_pin_PWM 11

// variables 
unsigned long currentTime;
unsigned long previousTime;
void setup() 
{
  pinMode (MOTOR_right_pin_sens_1 ,OUTPUT);
  pinMode (MOTOR_right_pin_sens_2 ,OUTPUT);
  pinMode (MOTOR_right_pin_PWM,OUTPUT);
  pinMode (MOTOR_left_pin_sens_1 ,OUTPUT);
  pinMode (MOTOR_left_pin_sens_2,OUTPUT);
  pinMode (MOTOR_left_pin_PWM ,OUTPUT);
  pinMode (SENSOR_mid ,INPUT);
  pinMode (SENSOR_left,INPUT);
  pinMode (SENSOR_right,INPUT);
  pinMode (SENSOR_left_front,INPUT);
  pinMode (SENSOR_right_front,INPUT);
  pinMode (ECHO_PIN_right,INPUT);
  pinMode (ECHO_PIN_front,INPUT);
  pinMode (TRIG_PIN_right,OUTPUT);
  pinMode (TRIG_PIN_front,OUTPUT);
  pinMode (LED_1,OUTPUT);
  pinMode (LED_2,OUTPUT);
  pinMode (LED_3,OUTPUT);
 Serial.begin(1200);
}
//fuctions 
void rotate_motor(int pin1,int pin2,int pin_PWM,int U)
{
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
  analogWrite(pin_PWM,U);
}
void loop() 
{
  /*currentTime = millis();
  if (currentTime - previousTime >= 5) 
  { 
    previousTime = currentTime;
  }*/
  rotate_motor(MOTOR_right_pin_sens_1,MOTOR_right_pin_sens_2,MOTOR_right_pin_PWM,200);
  rotate_motor(MOTOR_left_pin_sens_1,MOTOR_left_pin_sens_2,MOTOR_left_pin_PWM,200);

}
