
#define MOTOR_DIR 10                           // Non PWM pin for direction control
#define MOTOR_PWM 9                           // PWM controlled pin.  Pin 11 must be used since we are using Timer2 for pwm control

float Kp;
float Kd;
float Ki;
float target;
float motorSpeed;
float error;

int inByte = 0;  

int pos;

float lastError;
float sumError=0;
float Deriv;
float Fu=1/0.03;       //Sampling frequency to use on Ziegler-Nichols method

const int wiper        = 0;   //Position of fader relative to GND (Analog 0)

//Variables Calibration
double faderMax        = 0;   //Value read by fader's maximum position (0-1023)
double faderMin        = 0;   //Value read by fader's minimum position (0-1023)
int cal =1152;

//Var get Setpoint
char a[2];
int starttime=0;
int n;

//Matlab var
float x;
float y;
float timer;
int datap= 0.0;
float start_time;

float last_time=0;
float dt;

float P;
float I;
float D;

//End var

void setup() {

    Serial.begin(115200);

      
  pinMode(MOTOR_DIR, OUTPUT);            // initialize MOTOR_DIR pin as an OUTPUT pin; used to control the motor direction
  pinMode(MOTOR_PWM, OUTPUT);            // initialize MOTOR_PWM pin as an OUTPUT pin; used to control the motor pwm

  // initialize the OUTPUT's to a LOW value
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_PWM, LOW);


      Serial.println("Press 't' to choose target, 'h' to home, 's' to stop motor, 'c' to calibrate fader, 'p' to get position" ); //Serial print commands on initialization

}

void loop() {

int val = analogRead(A0); // read the potentiometer value (0 - 1023)
int pos = map(val, 0, 1023, 0, cal);// set the seek to target by mapping the potentiometer range to the encoder max count


Kp = 0.3;                         // set position gain; my values - yours will very likely be different

Kd = 0;                        // set derivative gain; my values - yours will very likely be different

if(abs(error)>5){
  Ki=2;
} else if (abs(error<=5)){     //To avoid too low values on low error
  Ki=0.04;
}

y=(x)*(12.0/1023.0);                       //Voltage of DC motor 12V
timer=((millis()-start_time)/1000.0000);   //Calculate time after 't' command is given in seconds


  dt=timer-last_time;  //Sampling time
  last_time=timer;

 if (Serial.available() > 0) 
  {  
  // then read the first available byte
  inByte = Serial.read();
  
  switch (inByte) 
  {
  case't':                             //User pressed 't', choose target (setpoint) Value 0-999
  getSerial();
datap=1;                             //Start printing data
start_time=millis();
sumError=0;                          //set sumError to '0' for the new target
last_time=0;    //Avoid negative dt
  break;
  
  case 'h':                             // User pressed 'h', home
    motor_home();                      // home the motor
    break;  

  case 's':                            //'s'
  motor_stop();                       //Stop motor
  target=0;  
  datap=0;                      //Stops printing data
  motorSpeed=0;
  break;

  case 'c':                            //'c'
  calibrateFader();                   //Calibrates fader and potentiometter mapping
  break;

  case 'p':                            //'p'
  readpot();                          //Print the output o analog A0 pin and the mapped position value
  break;

  }
  }
//Timer to change target on exact time
if (timer>2&&timer<2.005&&datap==1){
  sumError=0;
  target=600;
} else if (timer>4&&timer<4.005&&datap==1){
  sumError=0;
  target=400;
}else if (timer>6&&timer<6.005&&datap==1){
  sumError=0;
  target=200;
}else if(timer>=8&&datap==1){
motor_stop();
  target=0;  
  datap=0;                      //Stops printing data
  timer=0;                     //Zero timer to remame tests
}

  // PID

  
  error = target - pos;
  Deriv=(error - lastError)/dt;
  sumError = sumError + (error*dt);

// generalized PID formula
//correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)


  
  P = Kp * error;             //Proportional
  D = (Kd * (Deriv));         //Derivative
  I = Ki * sumError;          //Integral

  if(I>255){        //To avoid windup
    I=255;
  } else if(I<-255){
    I=-255;
  } 

  motorSpeed = P+I+D; 

    if (abs(motorSpeed)>255) {                     //To operate on Arduino analogWrite 0-255
    motorSpeed=255;
  } 
  
    if (error<0){                         //motor control change direction
    analogWrite(MOTOR_PWM, abs(motorSpeed));
    analogWrite(MOTOR_DIR, 0);
    x=analogRead(MOTOR_PWM);
  } else if(error>0) {
    analogWrite(MOTOR_PWM, 0);
    analogWrite(MOTOR_DIR, abs(motorSpeed));
    x=analogRead(MOTOR_DIR);
  } else {
    motor_stop();
  }

  lastError=error;



//end PID controller



if (datap==1){  //print data if datap=1
  //Print data to plot
  Serial.print(timer,4);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(motorSpeed,4);
  Serial.print("\t");
  Serial.print(pos);
  Serial.print("\t");
    Serial.print(dt,4);
  Serial.print("\t");
  Serial.print(error);
      Serial.print("\t");
  Serial.print(P);
    Serial.print("\t");
  Serial.print(I,4);
     Serial.print("\t");
  Serial.println(D);
}
}


void motor_stop() 
{

  // Put the motor control outputs to a safe 'LOW'
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_PWM, LOW);

}

void motor_home() 
{
  
    digitalWrite(MOTOR_PWM, HIGH);
        delay(250);
    digitalWrite(MOTOR_PWM, LOW);

}


void calibrateFader() {
    //Send fader to the top and read max position
    motor_home();
    digitalWrite(MOTOR_DIR, HIGH);
    delay(250);
    digitalWrite(MOTOR_DIR, LOW);    
    faderMax = analogRead(wiper);
        Serial.print("Max: "); 
        Serial.println(faderMax); 
    //Send fader to the bottom and read max position
    digitalWrite(MOTOR_PWM, HIGH);
    delay(250);
    digitalWrite(MOTOR_PWM, LOW);
    faderMin = analogRead(wiper);
        Serial.print(" Min: "); 
        Serial.println(faderMin); 

    cal = (1024/faderMax)*1000;
            Serial.print(" cal: "); 
        Serial.println(cal); 
}

  void readpot() {
int pot = analogRead(A0); // read the potentiometer value (0 - 1023)
int pos = map(pot, 0, 1023, 0, cal);// set the seek to target by mapping the potentiometer range to the encoder max count
    
            Serial.print(" Valor do A0 Pin: "); 
        Serial.println(pot); 
            Serial.print(" Valor da posição: "); 
        Serial.println(pos);   
  }

int getSerial() //Get a 3 characters number on Serial data after some character random, e.g.:*050, 0100, /999
{
 while(Serial.available() == 0); //Waiting serial data
 
 starttime = millis();
 
 while(Serial.available() < 3 && (millis()-starttime < 800));  //Only 3 characters is read
 n = Serial.available();
 if(Serial.available() > 3)
   {
   n=2;
   }
   
       for(int i = 0; i < n ; i++)  //Reading serial data to char
       {
       a[i] = Serial.read();
       }
       
     while(Serial.available() > 0) //Clearing serial buffer
     {
     Serial.read();

     }

 char *number_string = a;
 unsigned int value = atoi (number_string);  // convert char* to unsigned int
  target=value;
        Serial.print("Target: "); 
        Serial.println(target); 
 char a[2];  //Clearing char
}




