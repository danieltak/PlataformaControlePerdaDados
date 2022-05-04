
int motor1Pin = 9;
int motor2Pin = 10;
//int enablePin = 9;
int entrada = 0;
int inByte=0;
int val;

const int wiper        = 0;   //Position of fader relative to GND (Analog 0)

//Variables
double faderMax        = 0;   //Value read by fader's maximum position (0-1023)
double faderMin        = 0;   //Value read by fader's minimum position (0-1023)

void setup()
{
   Serial.begin(115200);
pinMode(motor1Pin, OUTPUT);
pinMode(motor2Pin, OUTPUT);
//pinMode(enablePin, OUTPUT);

}
void loop()

{


//  analogWrite(enablePin, 255);
//    analogWrite(motor1Pin, 255);
//      analogWrite(motor2Pin, 255);
 // testa se a porta serial está disponível
 if (Serial.available() > 0) {
 // le os dados da porta serial armazena na variavel 'entrada'
 entrada = Serial.read();
 if (entrada == 'e'){
 esquerda();
 Serial.print("esquerda ");
 
 } else if (entrada == 'd'){
 direita();
    Serial.print("direita ");
 
 } else if (entrada =='c'){
  calibrateFader();
 }
 else {
 parar();
    Serial.print("parar "); 
 }
 }
//
//  inByte = Serial.read();
//if(abs(inByte)>255) {
//  val=255;
//} else if (abs(inByte)<0){
//  val=0;
//} else {
//  val=abs(inByte);
//}
//  
//  }

}
 
void esquerda(){
//analogWrite(enablePin, 255);
digitalWrite(motor1Pin, LOW);
analogWrite(motor2Pin, 40);
}
 
void direita(){
//analogWrite(enablePin, 255);
analogWrite(motor1Pin, 50);
digitalWrite(motor2Pin, LOW);
}
 
void parar(){
 digitalWrite(motor1Pin, 0);   // para o motor
 digitalWrite(motor2Pin, 0);   //
}

//Calibrates the min and max position of the fader
void calibrateFader() {
    //Send fader to the top and read max position
    digitalWrite(motor1Pin, HIGH);
    delay(250);
    digitalWrite(motor1Pin, LOW);    
    faderMax = analogRead(wiper);
        Serial.print("Max: "); 
        Serial.print(faderMax); 
    //Send fader to the bottom and read max position
    digitalWrite(motor2Pin, HIGH);
    delay(250);
    digitalWrite(motor2Pin, LOW);
    faderMin = analogRead(wiper);
        Serial.print(" Min: "); 
        Serial.print(faderMin); 
}

