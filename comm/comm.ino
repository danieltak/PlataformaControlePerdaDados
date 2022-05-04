/*
  Firstbot PID code:  Implements a PID controller using
  analog inputs for actual and desired positions.

 The circuit: 
 * RX is digital pin 2 (connect to TX of other device)
 * TX is digital pin 3 (connect to RX of other device)

 */
#include <SoftwareSerial.h>

// define some constants
int ActPos = A0;    // select the input pin for feedback signal
int DesPos = A1;    // select the input pin for control signal

byte PWMOutput;
long Error[10];
long Accumulator;
long PID;
int PTerm;
int ITerm;
int DTerm;
byte Divider;

/* 
The FIRSTBOT has a PIC16F1829 controller that controls the 
two MC33926 H-bridges on the board.  A oftware serial interface
is used to control that part.
*/
SoftwareSerial mySerial(2, 3); // Receive data on 2, send data on 3
byte SerialTXBuffer[5];
byte SerialRXBuffer[5];

void setup()  
{

 // Open serial communications and wait for port to open:
  Serial.begin(9600);
  mySerial.begin(9600);
}

/* GetError():
Read the analog values, shift the Error array down 
one spot, and load the new error value into the
top of array.
*/
void GetError(void)
{
  byte i = 0;
  // read analogs
  word ActualPosition = analogRead(ActPos);  
// comment out to speed up PID loop
//  Serial.print("ActPos= ");
//  Serial.println(ActualPosition,DEC);

  word DesiredPosition = analogRead(DesPos);
// comment out to speed up PID loop
//  Serial.print("DesPos= ");
//  Serial.println(DesiredPosition,DEC);

  // shift error values
  for(i=0;i<10;i++)
    Error[i+1] = Error[i];
  // load new error into top array spot  
  Error[0] = (long)DesiredPosition-(long)ActualPosition;
// comment out to speed up PID loop
//  Serial.print("Error= ");
//  Serial.println(Error[0],DEC);

}

/* CalculatePID():
Error[0] is used for latest error, Error[9] with the DTERM
*/
void CalculatePID(void)
{
// Set constants here
  PTerm = 2000;
  ITerm = 25;
  DTerm = 0;
  Divider = 10;

// Calculate the PID  
  PID = Error[0]*PTerm;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  PID += ITerm*Accumulator; // add integral gain and error accumulation
  PID += DTerm*(Error[0]-Error[9]); // differential gain comes next
  PID = PID>>Divider; // scale PID down with divider

// comment out to speed up PID loop  
//Serial.print("PID= ");
//  Serial.println(PID,DEC);

// limit the PID to the resolution we have for the PWM variable

  if(PID>=127)
    PID = 127;
  if(PID<=-126)
    PID = -126;

//PWM output should be between 1 and 254 so we add to the PID    
  PWMOutput = PID + 127;

// comment out to speed up PID loop
//  Serial.print("PWMOutput= ");
//  Serial.println(PWMOutput,DEC);

}

/* WriteRegister():
Writes a single byte to the PIC16F1829, 
"Value" to the register pointed at by "Index".  
Returns the response 
*/
byte WriteRegister(byte Index, byte Value)
{
byte i = 0;
byte checksum = 0;
byte ack = 0;

SerialTXBuffer[0] = 210;
SerialTXBuffer[1] = 1;
SerialTXBuffer[2] = 3;
SerialTXBuffer[3] = Index;
SerialTXBuffer[4] = Value;

for (i=0;i<6;i++)
  {
  if (i!=5)
    {
    mySerial.write(SerialTXBuffer[i]);
    checksum += SerialTXBuffer[i];    
    }
  else
    mySerial.write(checksum);     
  }
  delay(5);

  if (mySerial.available())
    ack = mySerial.read();

  return ack;
} 

void loop() // run over and over
{
     GetError();       // Get position error
     CalculatePID();   // Calculate the PID output from the error
     WriteRegister(9,PWMOutput);  // Set motor speed
}
