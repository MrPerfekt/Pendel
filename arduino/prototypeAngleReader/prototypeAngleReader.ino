const int fsm[4][4]={{0,1,-1,0},{-1,0,0,1},{1,0,0,-1},{0,-1,1,0}};
const uint8_t pinA=10;
const uint8_t pinB=9;

int position=0; // zaehlen wir mal die absolute Position
  
void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
}

void loop() {
  uint8_t ina = digitalRead(pinA)==HIGH;
  uint8_t inb = digitalRead(pinB)==HIGH;
  static uint8_t lasta;
  static uint8_t lastb;
  position+=fsm[(ina<<1)+inb][(lasta<<1)+lastb];
  lasta = ina;
  lastb = inb;
}
