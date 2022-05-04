
#define MOTOR_DIR 10                           // Non PWM pin for direction control
#define MOTOR_PWM 9                           // PWM controlled pin.  Pin 11 must be used since we are using Timer2 for pwm control

float Kp;
float Kd;
float Ki;
int target;
int motorSpeed;
int error;

int inByte = 0;  
int correction;
int pos;

int lastError;
int sumError;
int scalefactor= 0.001;      //To obtain Values between 0-255, when the fader is calibrated to 0-1000

const int wiper        = 0;   //Position of fader relative to GND (Analog 0)

//Variables Calibration
double faderMax        = 0;   //Value read by fader's maximum position (0-1023)
double faderMin        = 0;   //Value read by fader's minimum position (0-1023)
int cal =1152;

//Var get Setpoint
char a[2];
int starttime=0;
int n;

void setup() {

    Serial.begin(115200);

      
  pinMode(MOTOR_DIR, OUTPUT);            // initialize MOTOR_DIR pin as an OUTPUT pin; used to control the motor direction
  pinMode(MOTOR_PWM, OUTPUT);            // initialize MOTOR_PWM pin as an OUTPUT pin; used to control the motor pwm

  // initialize the OUTPUT's to a LOW value
  digitalWrite(MOTOR_DIR, LOW);
  digitalWrite(MOTOR_PWM, LOW);



      Serial.println("Press '6' to go forwards, '4' to go backwards, 'h' to home, 's' to stop motor, 'c' to calibrate fader, 'p' to get position" );

}

void loop() {

int val = analogRead(A0); // read the potentiometer value (0 - 1023)
int pos = map(val, 0, 1023, 0, cal);// set the seek to target by mapping the potentiometer range to the encoder max count


Kp = 1;                         // set position gain; my values - yours will very likely be different
Ki = 0;
Kd = 1;                        // set derivative gain; my values - yours will very likely be different



 if (Serial.available() > 0) 
  {  
  // then read the first available byte
  inByte = Serial.read();
  
  switch (inByte) 
  {
  case't':                             //User pressed 't', choose target (setpoint) Value 0-999
  getSerial();
  break;
  
  case 'd':                             // User pressed 'd', move forward
//    motor_forward();                   // turn the motor forward 
  

  case 'e':                             // User pressed 'e', move backward
//    motor_backward();                  // turn the motor backward
    break;  

  case 'h':                             // User pressed 'h', home
    motor_home();                      // home the motor
    break;  

  case 's':                            //'s'
  motor_stop();                       //Stop motor
  break;

  case 'c':                            //'c'
  calibrateFader();                   //Calibrates fader and potentiometter mapping
  break;

  case 'p':                            //'p'
  readpot();                          //Print the output o analog A0 pin and the mapped position value
  break;

  case 'z':                          //'z'
  target=0;                          //zera o target
  break; 
//  default:  
//  
//    Serial.write(inByte);
//    Serial.println(" is not a valid key");

  }
  }


  // PID

  
  error = target - pos;
// generalized PID formula
//correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)

  motorSpeed = Kp * error;
  motorSpeed += Kd * (error - lastError);
  motorSpeed += Ki * (sumError);

    if (motorSpeed<0){                         //motor control change direction
    analogWrite(MOTOR_PWM, motorSpeed);
digitalWrite(MOTOR_DIR, LOW);
  } else {
    analogWrite(MOTOR_PWM, LOW);
    digitalWrite(MOTOR_DIR, motorSpeed);
  }

  
  if (abs(motorSpeed)>255) {                     //To avoid wind up
    motorSpeed=255;
  } 
  
//  else if (abs(motorSpeed)<0) {                //For unidirectional
//    motorSpeed=0;
//  } 

  lastError=error;
  sumError += error;
  
  motorSpeed=motorSpeed*scalefactor;

  
//print_pos_t();


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

void print_pos_t(){
    Serial.print("A posição alvo é: ");
  Serial.println(target - error); 

    Serial.print("O motorSpeed é: ");
  Serial.println(motorSpeed); 

  
    Serial.print("O target é: ");
  Serial.println(target); 
delay (2000);
  
}



