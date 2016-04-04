//#include<Servo.h>


//==Encoder==
const int8_t fsmEncoder[4][4]={{0,1,-1,0},{-1,0,0,1},{1,0,0,-1},{0,-1,1,0}};
const uint8_t pinEncoderA=2;
const uint8_t pinEncoderB=3;
unsigned long lastEncoderTime = 0;
unsigned long lastEncoderTimeDiff;
////==Servo==
//const uint8_t pinServoV=8;
//const uint8_t pinServoH=9;
//==Stepper==
const int stepperPinCnt = 4;
const int stepperStateCnt = 8;
const int stepperPins[stepperPinCnt]={30,32,34,36};
const uint8_t fsmStepper[stepperStateCnt][stepperPinCnt]={
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};
//==Magnet==
const uint8_t pinMagnet[2] = {46,48};

//==Encoder==
int encoderPos=0;
////==Servo==
//Servo servoH;
//Servo servoV;
//==Stepper==
int stepperPos=0;
  
void setup() {
    //==Magnet==
    pinMode(pinMagnet[0], OUTPUT);
    pinMode(pinMagnet[1], OUTPUT);
    pinMode(pinMagnetB[0], OUTPUT);
    pinMode(pinMagnetB[1], OUTPUT);
    digitalWrite(pinMagnetB[0],LOW);
    digitalWrite(pinMagnetB[0],LOW);

  while(1);
  /*
  //==Example Stepper Code==
  for(int i = 0; i < stepperPinCnt; i++){
    pinMode(stepperPins[i], OUTPUT);
  }
  while(1){
    stepperPos++;
    updateStepper();
    delayMicroseconds(1500);
  }
  */
  
  Serial.begin(9600);
  //==Encoder==
  pinMode(pinEncoderA, INPUT);
  pinMode(pinEncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA), incrementEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderB), incrementEncoder, CHANGE);
  ////==Servo==
  //servoH.attach(pinServoV);
  //servoV.attach(pinServoH);
  //==Stepper==
  for(int i = 0; i < stepperPinCnt; i++){
    pinMode(stepperPins[i], OUTPUT);
  }
}

//==Magnet==
void setMagnet(bool on, bool direction){
    digitalWrite(pinMagnet[0],on && direction);
    digitalWrite(pinMagnet[1],on && !direction);
}

//==Stepper==
void updateStepper(){
  int stepperState = stepperPos % stepperStateCnt;
  if(stepperState < 0){
    stepperState += stepperStateCnt;
  }
  for(int i = 0; i < stepperPinCnt; i++){
    digitalWrite(stepperPins[i],fsmStepper[stepperState][i]);
  }
}

void loop(){
  /*
  //Serial.println(position);
  //Length Pendulum:175mm
  //Length Encoder 75mm
  //Distanze Pendulum 235mm
  double speedAngle = 0 * min(max(lastEncoderTimeDiff/1000,30),-30);
  double angle = 45.0/180.0*(encoderPos+speedAngle)*PI/180.0;
  servoH.write(90+180.0/PI*atan(175.0*sin(-angle)/235.0));
  servoV.write(67-35+180.0/PI*atan(175.0*cos(angle)/235.0));
  */

  updateStepper();
}

void incrementEncoder(){
  static uint8_t lasta = 0;
  static uint8_t lastb = 0;
  uint8_t ina = digitalRead(pinEncoderA)==HIGH;
  uint8_t inb = digitalRead(pinEncoderB)==HIGH;
  //if(ina != lasta || inb != lastb)
  encoderPos+=fsmEncoder[(ina<<1)+inb][(lasta<<1)+lastb];
  lasta = ina;
  lastb = inb;
  unsigned long newTime = micros();
  lastEncoderTimeDiff = newTime - lastEncoderTime;
  lastEncoderTime = newTime;
  stepperPos=-(int)(encoderPos*1.5);
}
