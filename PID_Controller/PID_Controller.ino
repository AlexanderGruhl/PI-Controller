#include <math.h>

int trig = 12; 
int echo = 11; 
long lecture_echo; 
long cm;

int INA = 10; //motor output A
int INB = 9; //motor output B
int chooseIN; //choose output

int motorValue = 0; //PWM value sent to motor

//PID constants
double kp = 0.1; //0.06
double ki = 0.005; //0.01 0.005
double kd = 1;
 
//PID controller variables
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double error = 0;
double lastError = 0;
double output = 0, setPoint;
double cumError, rateError;

double prevOut = 0;

int posSaturation = 95, negSaturation = -95; //clamping saturation value, max value sent to motor (avoids sending max value)
int posMax = 100, negMax = -100; //max value that can be sent to the motor for mapping, used to avoid running motor at max voltage

void setup() 
{ 
  //ultrasonic sensor setup
  pinMode(trig, OUTPUT); 
  digitalWrite(trig, LOW); 
  pinMode(echo, INPUT);
   
  Serial.begin(9600); //sets serial transmission data baud rate to 9600
  
  setPoint = 30; //choose setPoint

  //motor output
  pinMode(INA,OUTPUT); 
  pinMode(INB,OUTPUT); 
}

void loop() 
{  
  do{
    return_cm();
  } while(cm > 75);
  
  //Serial.print("Distance in cm : "); 
  //Serial.println(cm);
  
  output = computePID(cm);
  
  //determines whether the motor will spin CW or CCW based on whether an object is closer or farther from
  //the ultrasonic sensor than setPoint
  if(output > 0){
    chooseIN = INA;
    motorValue = map(output, 0, posMax, 0, 255); //converts range of output to range of analogWrite for the motor
  } else if(output < 0){
    chooseIN = INB;
    motorValue = map(output, 0, negMax, 0, 255);
  }

  analogWrite(chooseIN, motorValue); 
}

//function to return the read distance (in cm) by the ultrasonic sensor
void return_cm(){
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW); 
  lecture_echo = pulseIn(echo, HIGH); 
  cm = lecture_echo / 58; 
  
  //ultrasonic sensor can be finicky in detected range so allow a 1cm leeway for the sensor to be at setPoint
  if((long)abs(setPoint - cm) <= 1){
    cm = setPoint;
  }
}

//function which calculates the PID controller output
double computePID(double inp){     
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
  error = setPoint - inp;                                // determine error
      
  //integral clamping, prevents integral windup
  if(output >= posSaturation && error > 0){
    cumError += 0;
  } else if(output <= negSaturation && error < 0){
    cumError += 0;
   } else {
    cumError += error * elapsedTime;                // compute integral
   }
        
   rateError = (error - lastError)/elapsedTime;   // compute derivative
        
   double out = kp*error + ki*cumError + kd*rateError;                //PID output
   Serial.println(out);
   
   if(out > posSaturation){
    out = posSaturation;               
   } else if(out < negSaturation){
    out = negSaturation;
   }
 
   lastError = error;                                //remember current error
   previousTime = currentTime;                        //remember current time
 
   return out;                                        //have function return the PID output
}

