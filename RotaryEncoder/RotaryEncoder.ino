const int channelA = 3; // interrupt 0. green
const int channelB = 4; // interrupt 1. grey
volatile int stateB;
volatile long newPos;
long oldPos;
long posDiff;

void setup() {
  // Serial.begin(9600);
  pinMode(channelA, INPUT);
  pinMode(channelB, INPUT);
  stateB = digitalRead(channelB);             // initial state of B
  attachInterrupt(channelA, detectA, RISING); // detect rising edge channel a
  attachInterrupt(channelB, detectB, CHANGE); // keep track of state of channel b
  analogWrite(DAC0, 0);
}

void loop() {
  posDiff  = newPos - oldPos;
  posDiff = map(posDiff, 0, 100, 0, 4095);

  if (posDiff < 0) {
    posDiff = 0;
  }
  if (posDiff > 4095) {
    posDiff = 4095;
  }

  // write position difference out as analog voltage
  analogWriteResolution(12);
  analogWrite(DAC0, posDiff);

  // Serial.print("encoder: ");
  // Serial.print(posDiff);
  // Serial.print("; analog: ");
  // Serial.println(analogRead(A0));

  oldPos = newPos;
  delay(10);
}

void detectA() {
  !stateB ? newPos++ : newPos--;
  newPos > 1023 ? newPos == 0 : newPos;
}

void detectB() {
  stateB = !stateB;
}

