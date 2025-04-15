#define PI 3.14159265358979323846

//ESP32 pin Layout
int sensors[8] = {35, 27, 26, 25, 14, 13, 12, 15}; // sensor's pins from left to right
int pushButton = 34; // pushbutton pin
int motorRA = 21; // right motor +
int motorRB = 19; // right motor -
int motorLA = 22; // left motor +
int motorLB = 23; // left motor -
int encoderRA = 33; // right encoder A phase
int encoderRB = 18; // right encoder B phase
int encoderLA = 5; // left encoder A phase
int encoderLB = 4; // left encpder B phase

volatile int encoderRCount = 0; // tick count of right encoder
volatile int previousEncoderRCount = 0; // previous tick count of right encoder
volatile double speedR = 0; // current speed of right motor
int targetR; // target encoder count of right motor

volatile int encoderLCount = 0;
volatile int previousEncoderLCount = 0;
volatile double speedL = 0;
int targetL;

float kpP = 2; // proportional weight of position control PID
float kiP = 0.43; // integral weight of position control PID
float kdP = 0.54; // derivative weight of position control PID
float ksP = 1; // all in one weight of position control PID (s stands for speed)
int previousErrorR = 0; // previous position error, needed for position control PID
unsigned long previousTimeR = 0;
unsigned long previousTimeL = 0;
int previousErrorL = 0;
int integralTermR = 0;
int integralTermL = 0;

const int PPR = 408;

void IRAM_ATTR encoderRISRA() { // increments or dectrements encoder count based on the state of A and B phases
  if(digitalRead(encoderRB) == digitalRead(encoderRA)){ // plot A and B to further see why it works
    encoderRCount--;
  }else{
    encoderRCount++;
  }
}

void IRAM_ATTR encoderRISRB() { // increments or dectrements encoder count based on the state of A and B phases
  if(digitalRead(encoderRB) == digitalRead(encoderRA)){ // plot A and B to further see why it works
    encoderRCount++;
  }else{
    encoderRCount--;
  }
}

void IRAM_ATTR encoderLISRA() { // same for its right equivilant
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){
    encoderLCount++;
  }else{
    encoderLCount--;
  }
}

void IRAM_ATTR encoderLISRB() { // increments or dectrements encoder count based on the state of A and B phases
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){ // plot A and B to further see why it works
    encoderLCount--;
  }else{
    encoderLCount++;
  }
}

int getPositionCorrectionR(){
  int error = targetR - encoderRCount; // distance which the wheel still needs to go, measured in ticks
  integralTermR = constrain( integralTermR + error, -100 ,100 );

  double derivative = (error - previousErrorR) / ( (micros() - previousTimeR) / 1000000.0) ;

  int value = (int)( ksP*( kpP*error + kdP*derivative + kiP*integralTermR) ); // PID formula
  
  previousErrorR = error;
  previousTimeR = micros();
  return( constrain(value, -255, 255) );
}

int getPositionCorrectionL(){
  int error = targetL - encoderLCount; // distance which the wheel still needs to go, measured in ticks
  integralTermL = constrain( integralTermL + error, -100 ,100 );

  double derivative = (error - previousErrorL) / ( (micros() - previousTimeL) / 1000000.0) ;

  int value = (int)( ksP*( kpP*error + kdP*derivative + kiP*integralTermL) ); // PID formula
  
  previousErrorL = error;
  previousTimeL = micros();
  return( constrain(value, -255, 255) );
}

void speedRight(int speed){
  if (speed >=0){
    analogWrite(motorRA, 0);
    analogWrite(motorRB, speed);
  }else{
    analogWrite(motorRB, 0);
    analogWrite(motorRA, (-1)*speed);
  }
}

void speedLeft(int speed){
  if(speed >=0){
    analogWrite(motorLA, 0);
    analogWrite(motorLB, speed);
  }else{
    analogWrite(motorLB, 0);
    analogWrite(motorLA, (-1)*speed);
  }
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);

  targetL = (158.0/4)/(66.0)*PPR;
  targetR = -(158.0/4)/(66.0)*PPR;
}

void loop() {
  speedLeft(getPositionCorrectionL());
  speedRight(getPositionCorrectionR());
  delay(10);
}
