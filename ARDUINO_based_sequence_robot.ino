#include <PIDController.h>

//pins declarations
#define DMOTOR1 2
#define DMOTOR1D 3
#define DIRECTION1M1 39
#define DIRECTION2M1 43
#define PWM1 9
#define DMOTOR2 18
#define DMOTOR2D 19
#define DIRECTION1M2 51
#define DIRECTION2M2 53
#define PWM2 10
#define TRIGPIN1  11
#define ECHOPIN1 12

//variables declarations
PIDController pos_pid1;
PIDController pos_pid2;
PIDController error;
volatile long int encoder_pos1cm = 50; //the desired distance of the first motor
volatile long int encoder_pos2cm = 50; //the desired distance of the second motor
volatile long int encoder_pos1 =  encoder_pos1cm*(480/(3.14*6)) ; //converting the distance into number of ticks
volatile long int encoder_pos2 = encoder_pos2cm*(480/(3.14*6));
long int last_encoder_pos1 = 0; // the previous value of the encoder's position
long int last_millis1 = 0; // the previous instant 

int motor_value1;
int motor_value2;
int motor_mode = 1; 
int v_limit1 = 250;
int v_limit2 = 250;
long duration1; // duration variable acquired from the ultrasonic sensor used to calulate the distance to navigate in a 2d space
int distance1; // distance variable 
int state = 0; // states 
  long Cmd;

//FUNCTIONS DECLARATIONS
 void encoder1(); //the function encoder is used to convert the binary sequence recieved from the encoder into a useful information expressed in integer number
// using a counter every time an event occures , this information is used after that to regulate either the position the the speed of the motor.
 void encoder2();
 void Motor1Clockwise(int); // rotation in the one direction for the first motor
 void Motor2Clockwise(int); // rotation in the one direction for the second motor
 void Motor1CounterClockwise(int); // rotation in the other direction for the first motor
 void Motor2CounterClockwise(int); // rotation in the other direction for the second motor


 void Move(void); // after executing this function the two motors will rotate in a specific direction based on the sign of encoder_pos1 and 2
 int STOP_Moving(void); // this function is used to stop the two motors in case of encountring an unexpected obstacle
 void stay_on_hold(void); // this function gives a 50 PWM to both motor in order to stop them from moving, the 50 value is choosen according to 
//the minimum required power for motors to start rotating.

// the following four functions give orders to both motors through the variables ecoder_pos1 and ecoder_pos2 
 void Rotate(void);
 void State1(void);
 void State3(void);
 void State5(void);
 
void setup()
{
  //defining pin types
  pinMode(DMOTOR1,INPUT);
  pinMode(DMOTOR1D,INPUT);
  pinMode(DIRECTION1M1, OUTPUT);
  pinMode(DIRECTION2M1, OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(DMOTOR2,INPUT);
  pinMode(DMOTOR2D,INPUT);
  pinMode(DIRECTION1M2, OUTPUT);
  pinMode(DIRECTION2M2, OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(ECHOPIN1, INPUT);
  Serial.begin(115200);
  
  // choosing the gains of the PID controller
  attachInterrupt(digitalPinToInterrupt(DMOTOR1D),encoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(DMOTOR2D),encoder2,RISING);
  pos_pid1.begin();    
  pos_pid1.tune(50, 0 ,5000);    
  pos_pid1.limit(-120, 120);
  pos_pid1.setpoint(0);
  pos_pid2.begin();    
  pos_pid2.tune(50, 0,5200);    
  pos_pid2.limit(-120, 120);
  pos_pid2.setpoint(0);
  delay(100);
}

void loop()
{
  Move();
  Rotate();
  State1();
  delay(20);
}

// t
void encoder1()
  {
    
    if(digitalRead(DMOTOR1) == HIGH)
    {
      encoder_pos1++;
    }
    else
    {
      encoder_pos1--;
    }
  }
  void encoder2()
  {
    
    if(digitalRead(DMOTOR2) == HIGH)
    {
      encoder_pos2++;
    }
    else
    {
      encoder_pos2--;
    }
  }
  
  void Motor1Clockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M1, HIGH);
  digitalWrite(DIRECTION2M1, LOW);
  analogWrite(PWM1, power);
  }else{
    digitalWrite(DIRECTION1M1, LOW);
    digitalWrite(DIRECTION2M1, LOW);
  }
}

void Motor1CounterClockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M1, LOW);
  digitalWrite(DIRECTION2M1, HIGH);
  analogWrite(PWM1, power);
  
  }else{
    digitalWrite(DIRECTION1M1, LOW);
    digitalWrite(DIRECTION2M1, LOW);
  }
}
void Motor2Clockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M2, HIGH);
  digitalWrite(DIRECTION2M2, LOW);
  analogWrite(PWM2, power);
  }else{
    digitalWrite(DIRECTION1M2, LOW);
    digitalWrite(DIRECTION2M2, LOW);
  }
}

void Motor2CounterClockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M2, LOW);
  digitalWrite(DIRECTION2M2, HIGH);
  analogWrite(PWM2, power);
  
  }else{
    digitalWrite(DIRECTION1M2, LOW);
    digitalWrite(DIRECTION2M2, LOW);
  }
}

void stay_on_hold(void)
{
  int T = millis();
  if(abs(T-millis())<2000)
  {
  analogWrite(PWM2, 50);
  analogWrite(PWM1, -50);
  }
}
//completing the state

void Move( void )
{

   motor_value1 =  pos_pid1.compute(encoder_pos1);
   motor_value2 =  pos_pid2.compute(encoder_pos2);
   if(motor_value2 > 0){if(motor_value2>v_limit2) motor_value2=v_limit2;
    Motor2CounterClockwise(motor_value2);
   }else{if(motor_value2<-v_limit2) motor_value2=-v_limit2;
    Motor2Clockwise(abs(motor_value2));
   }
   if(motor_value1 > 0){if(motor_value1>v_limit1) motor_value1=v_limit1;
    Motor1CounterClockwise(motor_value1);
   }else{if(motor_value1<-v_limit1) motor_value1=-v_limit1;
    Motor1Clockwise(abs(motor_value1));
   }
   Cmd += 0.00001*(abs(encoder_pos1)-abs(encoder_pos2));
}

//special moves
void Rotate(void)
{
  if(abs(encoder_pos1)<8 && abs(encoder_pos2)<8 && (state == 0 || state == 2))
  {
    delay(200);
    encoder_pos1cm = 3.14*17.5/2;
    encoder_pos2cm = -3.14*17.5/2;
    encoder_pos1 =  encoder_pos1cm*(480/(3.14*6)) ;
    encoder_pos2 = encoder_pos2cm*(480/(3.14*6));
    state++;
  }
}
void State1(void)
{
  if(abs(encoder_pos1)<8 && abs(encoder_pos2)<8 && state == 1)
  {
    delay(200);
    encoder_pos1cm = 50;
    encoder_pos2cm = 50;
    encoder_pos1 =  encoder_pos1cm*(480/(3.14*6)) ;
    encoder_pos2 = encoder_pos2cm*(480/(3.14*6));
    state++;
  }
}


void State3(void)
{
  if(abs(encoder_pos1)<10 && abs(encoder_pos2)<10 && state == 3)
  {
    encoder_pos1 = 1920;
    encoder_pos2 = -1920;
    state++;
  }
}
void State5(void)
{
  if(abs(encoder_pos1)<15 && abs(encoder_pos2)<15 && state == 5)
  {
    encoder_pos1 = 1920;
    encoder_pos2 = -1920;
    state++;
  }
}
//STOP

int STOP_Moving(void)
{
  int Stop_M = 0;
  digitalWrite(TRIGPIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN1, LOW);
  duration1 = pulseIn(ECHOPIN1, HIGH);
  distance1 = duration1*0.034/2;
  if(distance1 < 5) Stop_M = 1;
  else Stop_M = 0;

  return Stop_M;
  
}

