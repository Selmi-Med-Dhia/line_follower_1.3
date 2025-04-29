#define PI 3.14159265358979323846

//ESP32 pin Layout

int sensors[10] = {2, 34, 35, 27, 26, 25, 14, 13, 12, 15}; // sensor's pins from left to right
int motorRA = 23; // right motor +
int motorRB = 22; // right motor -
int motorLA = 19; // left motor +
int motorLB = 21; // left motor -
int encoderRA = 4; // right encoder A phase
int encoderRB = 5; // right encoder B phase
int encoderLA = 33; // left encoder A phase
int encoderLB = 18; // left encoder B phase
int startButton = 32;
float threashold[10] = {0,0,0,0,0,0,0,0,0,0};

volatile long encoderRCount = 0; // tick count of right encoder
volatile long previousEncoderRCount = 0; // previous tick count of right encoder
volatile float speedR = 0; // current speed of right motor
int targetR; // target encoder count of right motor
long previousMeasureTimeR = 0;
float targetSpeedR = 0;
float previousSpeedErrorR = 0;
int currentPWMR = 0;

volatile long encoderLCount = 0;
volatile long previousEncoderLCount = 0;
volatile float speedL = 0;
int targetL;
unsigned long previousMeasureTimeL = 0;
float targetSpeedL = 0;
float previousSpeedErrorL = 0;
int currentPWML = 0;

unsigned long tmp;

float kpP = 1; // proportional weight of position control PID
float kiP = 0.8; // integral weight of position control PID
float kdP = 50; // derivative weight of position control PID
float ksP = 1; // all in one weight of position control PID (s stands for speed)
int previousErrorR = 0; // previous position error, needed for position control PID
unsigned long previousTimeR = 0;
unsigned long previousTimeL = 0;
int previousErrorL = 0;
int integralTermR = 0;
int integralTermL = 0;

float kpS = 0.8; // proportional weight of position control PID
float kiS = 0.005; // integral weight of position control PID
float kdS = 0; // derivative weight of position control PID
float ksS = 0.2; // all in one weight of position control PID (s stands for speed)
float speedIntegralTermR = 0;
float speedIntegralTermL = 0;
unsigned long previousSpeedTimeR = 0;
unsigned long previousSpeedTimeL = 0;

const int PPR = 408;
float wheelDiamemter = 67.5;
float wheelSpacing = 154;

bool blackOnWhite = true;

float weights100[10] = {-150,-100,-40,-35,-5,5,35,40,100,150};
float weights200[10] = {-170,-100,-40,-25,-10,10,25,40,100,170};
float weights70[10] = {-150,-100,-40,-35,-5,5,35,40,100,150};
float weightsOffCenter[10] = {-100,-100,-90,-70,-40,-35,-5,5,35,40};

float weights[10] ={-150,-100,-40,-35,-5,5,35,40,100,150};

float kpL = 0.5; // proportional weight of line control PID 0.5 for 100
float kiL = 0.00; // integral weight of line control PID 
float kdL = 10; // derivative weight of line control PID 40 for 300
float ksL = 1;
float lineIntegralTerm = 0;
float previousLineError = 0;
long previousLineTime = 0;
float previousCorrection = 0;
bool lastMoveEnabled = true;

bool flags[20] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
long triggerPointsOfFlags[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float baseRPM = 95;

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
  speed = constrain(speed, -255, 255);
  if (speed >=0){
    analogWrite(motorRA, 0);
    analogWrite(motorRB, speed);
  }else{
    analogWrite(motorRB, 0);
    analogWrite(motorRA, (-1)*speed);
  }
}

void speedLeft(int speed){
  speed = constrain(speed, -255, 255);
  if(speed >=0){
    analogWrite(motorLA, 0);
    analogWrite(motorLB, speed);
  }else{
    analogWrite(motorLB, 0);
    analogWrite(motorLA, (-1)*speed);
  }
}

int getSpeedCorrectionR(){
  float error = targetSpeedR - speedR;
  
  speedIntegralTermR = constrain( speedIntegralTermR + error, -80 ,80 );

  double derivative = (error - previousSpeedErrorR) / ( (micros() - previousSpeedTimeR) / 1000000.0) ;

  int value = (int)( ksS*( kpS*error + kdS*derivative + kiS*speedIntegralTermR) ); // PID formula
  
  previousSpeedErrorR = error;
  previousSpeedTimeR = micros();
  return( constrain(value, -255, 255) );
}

int getSpeedCorrectionL(){
  float error = targetSpeedL - speedL;
  //Serial.println(error);
  delay(1);
  
  speedIntegralTermL = constrain( speedIntegralTermL + error, -80 ,80 );

  double derivative = (error - previousSpeedErrorL) / ( (micros() - previousSpeedTimeL) / 1000000.0) ;

  int value = (int)( ksS*( kpS*error + kdS*derivative + kiS*speedIntegralTermL) ); // PID formula

  previousSpeedErrorL = error;
  previousSpeedTimeL = micros();
  return( constrain(value, -255, 255) );
}

int getValue(int index){
  if(blackOnWhite){
    return (analogRead(sensors[index]) > threashold[index]);
  }else{
    return (analogRead(sensors[index]) < threashold[index]);
  }
}

double getLineCorrection(){ // returns correction needed to go back on line if off it
  float error = 0;
  for(int j=0; j<10; j++){
    for(int i=0; i<10; i++){
      error += getValue(i) * weights[i]; // summing current readings of all sensors multiplied by its corresponding weight, more positive means we need to go right and on like that
    }
    delayMicroseconds(10); // small delay tuned for stability
  }
  error /= 10;

  int sum = 0;
  for(int i=0;i<10;i++){
    sum += getValue(i);
  }
  if(sum == 0 && lastMoveEnabled){
    error = previousLineError;
  }

  lineIntegralTerm = constrain( lineIntegralTerm + error, -1000 ,1000 );

  double derivative = (error - previousLineError) / ( (micros() - previousLineTime) / 1000000.0) ;

  double value =    ksL*(kpL*error  +    kdL*derivative   +  kiL*lineIntegralTerm); // applying the PID formula
  
  // pushing back history of sum
  previousLineError = error;

  return value;
}

void calibrate(){
  for(int i=0; i<100; i++){
    for(int j=0; j<10; j++){
      threashold[j] += analogRead(sensors[j]);
    }
    delay(10);
  }

  targetR = encoderRCount + distanceToTicks(60);
  targetL = encoderLCount + distanceToTicks(60);
  while(!targetsReached(10)){
    speedRight(getPositionCorrectionR());
    speedLeft(getPositionCorrectionL());
  }
  stop();

  for(int i=0; i<100; i++){
    for(int j=0; j<10; j++){
      threashold[j] += analogRead(sensors[j]);
    }
    delay(10);
  }

  targetR = encoderRCount - distanceToTicks(60);
  targetL = encoderLCount - distanceToTicks(60);
  while(!targetsReached(10)){
    speedRight(getPositionCorrectionR());
    speedLeft(getPositionCorrectionL());
  }
  stop();
  for(int i=0;i<10 ;i++){
    threashold[i] /= 200;
  }
}

void stop(){
  for(int i=0; i<10; i++){
    speedRight(0);
    speedLeft(0);
    delay(1);
  }
}

void turn(float degree){
  targetR = encoderRCount + ( (wheelSpacing*degree/360.0)/(wheelDiamemter) ) * PPR;
  targetL = encoderLCount - ( (wheelSpacing*degree/360.0)/(wheelDiamemter) ) * PPR;
  while(!targetsReached(20)){
    speedRight(getPositionCorrectionR());
    speedLeft(getPositionCorrectionL());
  }
  stop();
}

int sumSensors(int start, int end){
  int sum = 0;
  for(int i=start; i<end; i++){
    sum += getValue(i);
  }
  return sum;
}

void setup() {
  Serial.begin(115200);

  pinMode(startButton,INPUT_PULLUP);
  triggerPointsOfFlags[9] = - distanceToTicks(1000);
  flags[9] = true;

  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);

  calibrate();

  while(!digitalRead(startButton)){
    delay(1);
  }
  delay(100);
}

// sensor [4] ynoooo555555 w [5],[7],[8] yno55

int distanceToTicks(float distance){
  return ( distance/(wheelDiamemter*PI) )*PPR ;
}

bool targetsReached(int accuracy){
  return abs(targetL - encoderLCount) < accuracy && abs(targetR - encoderRCount) < accuracy ;
}

void loop() {
  // reading speed

  tmp = micros();
  speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
  previousEncoderRCount = encoderRCount;
  previousMeasureTimeR = tmp;

  speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
  previousEncoderLCount = encoderLCount;
  previousMeasureTimeL = tmp;
  delay(8);
  
  ////////////////////////////

  //sensor values visualization
  /*
  for(int i=0; i<9; i++){
    Serial.print((getValue(i)?"|":"."));
    Serial.print(":");
  }
  Serial.println(getValue(9)?"|":".");
  */

  //speed control
  
  currentPWMR += getSpeedCorrectionR();
  currentPWML += getSpeedCorrectionL();

  speedRight(currentPWMR);
  speedLeft(currentPWML);
  

  /////////////////////////  

  float correction = (float)getLineCorrection();

  targetSpeedR = baseRPM - correction;
  targetSpeedL = baseRPM + correction;

  // jumping to speed (approximately)
  if(previousCorrection != correction){
    if (speedR > 5){
      speedRight(currentPWMR*(targetSpeedR/speedR));
    };
    if (speedL > 5){
      speedLeft(currentPWML*(targetSpeedL/speedL));
    };
  }
  previousCorrection = correction;
  
  //flags
  /*
  // slows down to turn the 0.75*circle
  if (!flags[0] && encoderRCount > distanceToTicks(120)){
    kdL = 25;
    for(int i=0; i<10;i++){
      weights[i] = weights100[i];
    }
    baseRPM = 100;
    flags[0] = true;
  }
  // brings the speed back up after the circle
  if (flags[0] && !flags[1] && encoderRCount > distanceToTicks(950)){
    kdL = 40;
    for(int i=0; i<10;i++){
      weights[i] = weights200[i];
    }
    baseRPM = 290;
    flags[1] = true;
  }
  // detects the base of the tree and turns 90° to face it
  if (flags[1] && !flags[2] && encoderRCount > distanceToTicks(850) && sumSensors(4, 10)>3 ){
    stop();
    delay(300);
    turn(-140);
    for(int i=0; i<10;i++){
      weights[i] = weights70[i];
    }
    baseRPM = 70;
    kdL = 30;
    flags[2] = true;
    triggerPointsOfFlags[2] = encoderRCount;
  }
  // brings the speed back up after the first sharp tun in the tree
  if (flags[2] && !flags[3] && encoderRCount - triggerPointsOfFlags[2] > distanceToTicks(500) ){
    baseRPM = 140;
    for(int i=0; i<10;i++){
      weights[i] = weights100[i];
    }
    kdL = 30;
    flags[3] = true;
    triggerPointsOfFlags[3] = encoderRCount;
  }
  // detects the top of the tree and turns the sharp turn
  if (flags[3] && !flags[4] && encoderRCount - triggerPointsOfFlags[3] > distanceToTicks(600) && sumSensors(0,10) > 3){
    delay(50);
    stop();
    delay(300);
    turn(90);
    baseRPM = 100;
    for(int i=0; i<10;i++){
      weights[i] = weights100[i];
    }
    kdL = 20;
    flags[4] = true;
    triggerPointsOfFlags[4] = encoderRCount;
  }
  // turns at the end of the tree
  if (flags[4] && !flags[5] && encoderRCount - triggerPointsOfFlags[4] > distanceToTicks(900) && sumSensors(0,10) > 3){
    stop();
    delay(300);
    targetR = encoderRCount + distanceToTicks(270);
    targetL = encoderLCount ;
    while(!targetsReached(15)){
      speedRight(getPositionCorrectionR());
      speedLeft(getPositionCorrectionL());
    }
    stop();

    baseRPM = 55;
    for(int i=0; i<10;i++){
      weights[i] = weights100[i];
    }
    kdL = 30;

    weights[0] = 0;
    weights[1] = 0;
    weights[2] = 0;

    weights[7] =60;
    weights[8] =200;
    weights[9] = 250;
    flags[5] = true;
    triggerPointsOfFlags[5] = encoderRCount;
  }
  // brings the speed back up after exiting the tree
  if (flags[5] && !flags[6] && encoderRCount - triggerPointsOfFlags[5] > distanceToTicks(280) ){
    baseRPM = 230;
    for(int i=0; i<10;i++){
      weights[i] = weights200[i];
    }
    kdL = 50;
    flags[6] = true;
    triggerPointsOfFlags[6] = encoderRCount;
  }
  // slows down for the squiguelly line
  if (flags[6] && !flags[7] && encoderRCount - triggerPointsOfFlags[6] > distanceToTicks(270) ){
    baseRPM = 95;
    for(int i=0; i<10;i++){
      weights[i] = weights100[i];
    }
    kdL = 30;
    flags[7] = true;
    triggerPointsOfFlags[7] = encoderRCount;
  }
  // reduces the Kd for the second part of the squiguelly line
  if (flags[7] && !flags[8] && encoderRCount - triggerPointsOfFlags[7] > distanceToTicks(500) ){
    kdL = 5;
    weights[0] = -120;
    weights[1] = -80;
    weights[8] = 80;
    weights[9] = 120;
    flags[8] = true;
    triggerPointsOfFlags[8] = encoderRCount;
  }
  // prefers right for the frogs eyes
  if (flags[8] && !flags[9] && encoderRCount - triggerPointsOfFlags[8] > distanceToTicks(1880) ){
    kdL = 10;
    weights[8] = 0;
    weights[9] = 0;
    weights[3] = -50;
    flags[9] = true;
    triggerPointsOfFlags[9] = encoderRCount;
  }
  */
  // detects the frogs leaf and turns towards it
  if(flags[9] && !flags[10] && encoderRCount-triggerPointsOfFlags[9]>distanceToTicks(1400) && sumSensors(0,6)>3 ){
    stop();
    delay(200);
    targetR = encoderRCount + distanceToTicks(230);
    targetL = encoderLCount;
    while(!targetsReached(15)){
      speedRight(getPositionCorrectionR());
      speedLeft(getPositionCorrectionL());
    }
    stop();
    for(int i=0 ;i<10;i++){
      weights[i] = weights100[i];
    }
    flags[10] = true;
    triggerPointsOfFlags[10] = encoderRCount;
  }
  // zeroes the middle sensors after exiting the leaf
  if(flags[10] && !flags[11] && encoderRCount - triggerPointsOfFlags[10] > distanceToTicks(200) && sumSensors(0,10)>3 ){ 
    for(int i=1;i<9;i++){
      weights[i] = 0 ;
    }
    weights[0] = -200;
    weights[9] = 200;
    baseRPM = 50;
    flags[11] = true;
  }
  // detects the entrance to the after leaf thingy and sprints forward
  if(flags[11] && !flags[12] && getValue(0) && getValue(9)){
    targetR = encoderRCount;
    targetL = encoderLCount;
    stop();
    delay(200);
    while(!targetsReached(15)){
      speedRight(getPositionCorrectionR());
      speedLeft(getPositionCorrectionL());
    }
    stop();
    delay(200);
    targetR = encoderRCount+distanceToTicks(100);
    targetL = encoderLCount+distanceToTicks(95);
    targetSpeedR = 100;
    targetSpeedL = 95;
    while(true){
      tmp = micros();
      speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
      previousEncoderRCount = encoderRCount;
      previousMeasureTimeR = tmp;

      speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
      previousEncoderLCount = encoderLCount;
      previousMeasureTimeL = tmp;
      delay(8);

      currentPWMR += getSpeedCorrectionR();
      currentPWML += getSpeedCorrectionL();

      speedRight(currentPWMR);
      speedLeft(currentPWML);
      if(encoderRCount > targetR - 10 && encoderLCount > targetL - 10){
        break;
      }
    }
    stop();
    baseRPM=250;
    for(int i=0; i<10;i++){
      weights[i] = weights200[i];
    }
    kdL = 50;
    weights[0] = 0;
    weights[9] = 0;
    lastMoveEnabled = false;
    previousLineError = 0;
    flags[12] = true;
    triggerPointsOfFlags[12] = encoderRCount;
  }
  // slows down for the 90° turn near the base
  if( flags[12] && !flags[13] && encoderRCount - triggerPointsOfFlags[12] > distanceToTicks(1400)){
    baseRPM=120;
    for(int i=0; i<10;i++){
      weights[i] = weights100[i];
    }
    kdL = 0;
    lastMoveEnabled = true;
    flags[13]=true;
  }
  // stops at the base
  if(flags[13] && !flags[14] && sumSensors(0,10) == 8){
    targetR = encoderRCount + distanceToTicks(155);
    targetL = encoderLCount + distanceToTicks(155);
    stop();
    targetSpeedR = 100;
    targetSpeedL = 100;
    while(true){
      tmp = micros();
      speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
      previousEncoderRCount = encoderRCount;
      previousMeasureTimeR = tmp;

      speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
      previousEncoderLCount = encoderLCount;
      previousMeasureTimeL = tmp;
      delay(8);

      currentPWMR += getSpeedCorrectionR();
      currentPWML += getSpeedCorrectionL();

      speedRight(currentPWMR);
      speedLeft(currentPWML);
      if(encoderRCount > targetR - 10 && encoderLCount > targetL - 10){
        break;
      }
    }
    stop();
    delay(2000);
    targetR = encoderRCount + 530;
    targetL = encoderLCount;
    targetSpeedR = 100;
    targetSpeedL = 0;
    while(true){
      tmp = micros();
      speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
      previousEncoderRCount = encoderRCount;
      previousMeasureTimeR = tmp;

      speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
      previousEncoderLCount = encoderLCount;
      previousMeasureTimeL = tmp;
      delay(8);

      currentPWMR += getSpeedCorrectionR();
      currentPWML += getSpeedCorrectionL();

      speedRight(currentPWMR);
      speedLeft(currentPWML);
      if(encoderRCount > targetR - 10 && encoderLCount > targetL - 5){
        break;
      }
    }
    stop();
    delay(300);

    targetR = encoderRCount - distanceToTicks(75);
    targetL = encoderLCount - distanceToTicks(75);
    targetSpeedR = -100;
    targetSpeedL = -100;
    currentPWMR = 0;
    currentPWML = 0;

    while(true){
      speedRight(currentPWMR);
      speedLeft(currentPWML);
      
      delay(8);
      tmp = micros();
      speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
      previousEncoderRCount = encoderRCount;
      previousMeasureTimeR = tmp;

      speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
      previousEncoderLCount = encoderLCount;
      previousMeasureTimeL = tmp;

      currentPWMR += getSpeedCorrectionR();
      currentPWML += getSpeedCorrectionL();

      if(encoderRCount < targetR + 10 && encoderLCount < targetL + 10){
        break;
      }
    }
    stop();
    baseRPM = 0;
    kdL = 30;
    flags[14]=true;
    triggerPointsOfFlags[14] = millis();
  }
  // centers itself on the line preparing for the ramp
  if( flags[14] && !flags[15] && millis() - triggerPointsOfFlags[14] > 1500){
    baseRPM = 100;
    for(int i=0;i<10;i++){
      weights[i] = weights100[i];
    }
    flags[15] = true;
    triggerPointsOfFlags[15] = encoderRCount;
  }
  if( flags[15] && !flags[16] && encoderRCount - triggerPointsOfFlags[15] > distanceToTicks(80) && sumSensors(0,6) > 3){
    targetR = encoderRCount + distanceToTicks(70);
    targetL = encoderLCount + distanceToTicks(70);
    targetSpeedR = 100;
    targetSpeedL = 100;
    while(true){
      tmp = micros();
      speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
      previousEncoderRCount = encoderRCount;
      previousMeasureTimeR = tmp;

      speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
      previousEncoderLCount = encoderLCount;
      previousMeasureTimeL = tmp;
      delay(8);

      currentPWMR += getSpeedCorrectionR();
      currentPWML += getSpeedCorrectionL();

      speedRight(currentPWMR);
      speedLeft(currentPWML);
      if(encoderRCount > targetR - 10 && encoderLCount > targetL - 10){
        break;
      }
    }
    flags[16] = true;
    triggerPointsOfFlags[16] = encoderRCount;
  }
  if( flags[16] && !flags[17] && encoderRCount - triggerPointsOfFlags[16] > distanceToTicks(120) ){
    targetR = encoderRCount + distanceToTicks(1000);
    targetL = encoderLCount + distanceToTicks(1000);
    targetSpeedR = 300;
    targetSpeedL = 300;
    while(true){
      tmp = micros();
      speedR = ( (encoderRCount - previousEncoderRCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeR);
      previousEncoderRCount = encoderRCount;
      previousMeasureTimeR = tmp;

      speedL = ( (encoderLCount - previousEncoderLCount)*(60000000.0/PPR) ) / (tmp - previousMeasureTimeL);
      previousEncoderLCount = encoderLCount;
      previousMeasureTimeL = tmp;
      delay(8);

      currentPWMR += getSpeedCorrectionR();
      currentPWML += getSpeedCorrectionL();

      speedRight(currentPWMR);
      speedLeft(currentPWML);
      if(encoderRCount > targetR - 10 && encoderLCount > targetL - 10){
        break;
      }
    }
    stop();
    flags[17] = true;
    triggerPointsOfFlags[17] = encoderRCount;
  }
  if( flags[17] && !flags[18] && sumSensors(0, 10)>6 && encoderRCount - triggerPointsOfFlags[17] > distanceToTicks(1200)){
    blackOnWhite = false;
    flags[18] = true;
    triggerPointsOfFlags[18] = encoderRCount;
  }

}