//#include<Servo.h>

//==LEDs==
const uint8_t ledCnt = 9;
const uint8_t ledPins[ledCnt]={43,45,47,49,51,53,48,50,52};

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
const int stepperPins[2][stepperPinCnt]={{30,32,34,36},{31,33,35,37}};
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
const uint8_t pinMagnet[2] = {44,42};//{48,46}{44,42};
const uint8_t range = 20;

//==Encoder==
int encoderPos=0;
////==Servo==
//Servo servoH;
//Servo servoV;

//==Stepper==
int stepperPos[2]={0,0};
  
void setup() {
  //==LEDs==
  for(uint8_t i = 0; i < ledCnt;i++){
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  /*
  while(1){
    static bool on = false;
    for(uint8_t i = 0; i < ledCnt;i++){
      digitalWrite(ledPins[i],on);
      delay(100);
    }
    on = !on;
  }
  */
  
  //==Magnet==
  pinMode(pinMagnet[0], OUTPUT);
  pinMode(pinMagnet[1], OUTPUT);
  //setMagnet(true,false); //Push
  //delay(500);

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
    pinMode(stepperPins[0][i], OUTPUT);
    pinMode(stepperPins[1][i], OUTPUT);
  }
}

//==Magnet==
void setMagnet(bool on, bool pull){
    digitalWrite(pinMagnet[0],on && !pull);
    digitalWrite(pinMagnet[1],on && pull);
}

//==Stepper==
void updateStepper(uint8_t stepper){
  int stepperState = stepperPos[stepper] % stepperStateCnt;
  if(stepperState < 0){
    stepperState += stepperStateCnt;
  }
  for(int i = 0; i < stepperPinCnt; i++){
    digitalWrite(stepperPins[stepper][i],fsmStepper[stepperState][i]);
    digitalWrite(stepperPins[stepper][i],fsmStepper[stepperState][i]);
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

  updateStepper(0);
  updateStepper(1);
  //Serial.println(encoderPos);
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
  stepperPos[0]=-(int)(encoderPos*1.5);
  stepperPos[1]= -abs((int)(encoderPos/2));

  for(uint8_t i = 0; i < ledCnt;i++){
    digitalWrite(ledPins[i],(ledCnt-i)*5 > abs(encoderPos));
  }  

  static bool magnetDir = false;
  static uint8_t magnetOn = 1;
  static int maximum = 0;
  if(abs(encoderPos) > range){
    if(magnetOn){
      magnetOn = 0;
      setMagnet(false,false); //off
      maximum = 0;
      Serial.println("a");
    } else {
      if(abs(maximum) < abs(encoderPos)){
        maximum = encoderPos;
        Serial.println("a2");
        Serial.println(maximum);
      }
    }
  } else {
    if(!magnetOn){
      magnetOn = 1;
      setMagnet(1,true);
      magnetDir = true;
      Serial.println("b");
    } else if(((encoderPos < 0) != (maximum < 0)) && magnetDir){
      magnetDir = false;
      setMagnet(0,false);
      Serial.println("c");
    }
  }
}
