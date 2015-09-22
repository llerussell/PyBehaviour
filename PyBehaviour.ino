#include <elapsedMillis.h>
#include <TaskScheduler.h>

//---------------------------------------------------------
// INITIALISE
//---------------------------------------------------------

Task tTransmit;
Task tStimStart;
Task tStimStop;
Task tRespWinOpen;
Task tRespWinClose;
Task tAutoReward;
Task tRewardOn;
Task tRewardOff;
Task tRewardRemovalOn;
Task tRewardRemovalOff;
Task tEndTrial;

Scheduler taskManager;

elapsedMillis witholdTimer;

// pin numbers
const int responsePin[] = {1, 2}; // interrupt numbers 0:2 1:3 2:21
const int stimPin[] = {22, 23};
const int rewardPin[] = {4, 5};
const int rewardRemovalPin[] = {8, 9};
const int punishPin[] = {6, 7};

// serial communication
char incomingByte;
char varBuffer[20];
int varIndex = 0;
int varBufferIndex = 0;
boolean configStringStarted = false;
boolean configStringEnded = false;
boolean trialStringStarted = false;
boolean trialStringEnded = false;

// control
char INCOMING_COMMAND = '@';
char TEST_STIM = 'S';
char TEST_REWARD = 'R';
char TEST_PING = 'P'; // test if ready
char test_command;
char test_chan_num;
int test_chan_int;
String ConfigString;

// training parameters
int stimChan;
int respReq;
int rewardChan;
boolean trialCue;
boolean stimCue;
boolean respCue;
int respCueStart;
boolean withold;
int witholdReq;
int stimStart;
int stimStop;
int respStart;
int respStop;
int trialDuration;
boolean autoReward;
int autoRewardStart;
boolean punish;
int punishLength;
boolean punishTimeout;
boolean rewardRemoval;
int rewardRemovalDelay;

// session/trial states
boolean configReceived = false;
boolean trialConfigured = false;
boolean trialRunning;
boolean inRespWin;
boolean inWithold;

// response states
boolean responded = false;
boolean respondedInResp = false;
boolean rewarded = false;
boolean punished = false;

// record times
int preTrialDelayStart;
long startTime;
long now;
int timePressed;
String button1Times = "<";
String button2Times = "<";
int timeReward;
int timePunish;
String dataString;
int lengthDataString;
String newDataString;
String txString;
String comString;
int responseNum;
volatile long timeResponded;
// volatile long witholdTimer;
long currentTime;
int respWinResponses[200];
int respWinRespIdx;

// results
boolean correct;
boolean incorrect;
boolean hit;
boolean miss;
boolean fa;
boolean cr;


//---------------------------------------------------------
// RUNS ONCE AT STARTUP
//---------------------------------------------------------

void setup() {
  Serial.begin(19200);
  Serial.println("{READY}");
  delay(10);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  for ( int i = 0; i < sizeof(responsePin); ++i ) {
    pinMode(responsePin[i], INPUT);
  }
  for ( int i = 0; i < sizeof(rewardPin); ++i ) {
    pinMode(rewardPin[i], OUTPUT);
    digitalWrite(rewardPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(stimPin); ++i ) {
    pinMode(stimPin[i], OUTPUT);
    digitalWrite(stimPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(punishPin); ++i ) {
    pinMode(punishPin[i], OUTPUT);
    digitalWrite(punishPin[i], LOW);
  }
  for( int i = 0; i < sizeof(rewardRemovalPin);  ++i ) {
   pinMode(rewardRemovalPin[i], OUTPUT);
   digitalWrite(rewardRemovalPin[i], LOW);
  }
}


//---------------------------------------------------------
// MAIN LOOP
//---------------------------------------------------------

void loop() {
  // wait for configuration instructions
  while (!configReceived) {
    rxConfig();
  }

  // configuration has been received, set up trial and run it
  if (configReceived) {
    configTrial();
  }
  if (trialConfigured) {
    runTrial();
  }
}


//---------------------------------------------------------
// FUNCTIONS
//---------------------------------------------------------

void rxConfig() {
  while (Serial.available()) {
    incomingByte = Serial.read();

    // TEST STIM AND OR REWARD
    if (incomingByte == INCOMING_COMMAND) {
      delay(1);
      test_command = Serial.read();
      delay(1);
      test_chan_num = Serial.read();
      test_chan_int = test_chan_num - '0' - 1;

      if (test_command == TEST_REWARD) {
        testReward(rewardPin[test_chan_int]);
      }
      else if (test_command == TEST_STIM) {
        testStim(stimPin[test_chan_int]);
      }
      else if (test_command == TEST_PING) {
        Serial.println("{!}");
      }
    }

    // TRIAL CONFIGURATION
    else if (incomingByte == '<') {
      configStringStarted = true;
      varBufferIndex = 0;
      varBuffer[varBufferIndex] = '\0';
      varIndex = 0;
      //String ConfigString;
      //ConfigString = "{";
    }
    else if (incomingByte == '>') {
      // when reach > stop and clear buffer (variables already saved)
      configStringEnded = true;
      varBufferIndex = 0;
      varBuffer[varBufferIndex] = '\0';
      varIndex = 0;
      //ConfigString = ConfigString + "}";
      break;
    }
    else if (incomingByte == ':') {
      // : signals start of variable, the preceding identfier string is discarded
      varBufferIndex = 0;
      varBuffer[varBufferIndex] = '\0';
    }
    else if (incomingByte == ';') {
      // when reach seperator ; save the variable
      varIndex++;

      switch (varIndex) {
        case 1:
          stimChan = atoi(varBuffer) - 1;
          //ConfigString = ConfigString + "stim_chan:" + stimChan + ";";
          break;
        case 2:
          respReq = atoi(varBuffer);
          //ConfigString = ConfigString + "resp_req:" + respReq + ";";
          break;
        case 3:
          rewardChan = atoi(varBuffer) - 1;
          //ConfigString = ConfigString + "reward_chan:" + rewardChan + ";";
          break;
        case 4:
          trialCue = atoi(varBuffer);
          //ConfigString = ConfigString + "trial_cue:" + trialCue + ";";
          break;
        case 5:
          stimCue = atoi(varBuffer);
          //ConfigString = ConfigString + "stim_cue:" + stimCue + ";" ;
          break;
        case 6:
          respCue = atoi(varBuffer);
          //ConfigString = ConfigString + "resp_cue:" + respCue + ";";
          break;
        case 7:
          respCueStart = atoi(varBuffer);
          //ConfigString = ConfigString + "resp_cue_start:" + respCueStart + ";";
          break;
        case 8:
          withold = atoi(varBuffer);
          //ConfigString = ConfigString + "withold:" + withold + ";";
          break;
        case 9:
          witholdReq = atoi(varBuffer);
          //ConfigString = ConfigString + "withold_req:" + witholdReq + ";";
          break;
        case 10:
          stimStart = atoi(varBuffer);
          //ConfigString = ConfigString + "stim_start:" + stimStart + ";";
          break;
        case 11:
          stimStop = atoi(varBuffer);
          //ConfigString = ConfigString + "stim_stop:" + stimStop + ";";
          break;
        case 12:
          respStart = atoi(varBuffer);
          //ConfigString = ConfigString + "resp_start:" + respStart + ";";
          break;
        case 13:
          respStop = atoi(varBuffer);
          //ConfigString = ConfigString + "resp_stop:" + respStop + ";";
          break;
        case 14:
          trialDuration = atoi(varBuffer);
          //ConfigString = ConfigString + "trial_duration:" + trialDuration + ";";
          break;
        case 15:
          autoReward = atoi(varBuffer);
          //ConfigString = ConfigString + "auto_reward:" + autoReward + ";";
          break;
        case 16:
          autoRewardStart = atoi(varBuffer);
          //ConfigString = ConfigString + "auto_reward_start:" + autoRewardStart + ";";
          break;
        case 17:
          punish = atoi(varBuffer);
          //ConfigString = ConfigString + "punish:" + punish + ";";
          break;
        case 18:
          punishLength = atoi(varBuffer);
          //ConfigString = ConfigString + "punish_length:" + punishLength + ";";
          break;
        case 19:
          rewardRemoval = atoi(varBuffer);
          //ConfigString = ConfigString + "reward_removal:" + rewardRemoval + ";";
          break;
        case 20:
          rewardRemovalDelay = atoi(varBuffer);
          //ConfigString = ConfigString + "reward_removal_delay:" + rewardRemovalDelay + ";";
          break;
      }
      varBufferIndex = 0;
      varBuffer[varBufferIndex] = '\0';
    }
    else {
      // save the read character to the incoming variable buffer
      varBuffer[varBufferIndex] = incomingByte;
      varBufferIndex++;
      varBuffer[varBufferIndex] = '\0';
    }
  }
  if (configStringStarted && configStringEnded) {
    // received a whole < > packet
    configReceived = true;
    
    String ConfigString = "{";
    ConfigString = ConfigString + "withold_req:" + witholdReq + "}";
    Serial.println(ConfigString);

  }
}

void configTrial() {
  // set(delay, repeat, function)
  taskManager.init();

  tTransmit.set(100, -1, &txData);
  tStimStart.set(stimStart, 1, &stimOn);
  tStimStop.set(stimStop, 1, &stimOff);
  tRespWinOpen.set(respStart, 1, &respWinOpen);
  tRespWinClose.set(respStop, 1, &respWinClose);
  tRewardOn.set(0, 1, &rewardOn);
  tRewardOff.set(100, 1, &rewardOff);
  tAutoReward.set(autoRewardStart, 1, &autoRewardOn);
  tRewardRemovalOn.set(rewardRemovalDelay, 1, &rewardRemovalOn);
  tRewardRemovalOff.set(100, 1, &rewardRemovalOff);
  tEndTrial.set(trialDuration, 1, &stopTasks);

  tTransmit.disableOnLastIteration(true);
  tStimStart.disableOnLastIteration(true);
  tStimStop.disableOnLastIteration(true);
  tRespWinOpen.disableOnLastIteration(true);
  tRespWinClose.disableOnLastIteration(true);
  tRewardOn.disableOnLastIteration(true);
  tRewardOff.disableOnLastIteration(true);
  tAutoReward.disableOnLastIteration(true);
  tRewardRemovalOn.disableOnLastIteration(true);
  tRewardRemovalOff.disableOnLastIteration(true);
  tEndTrial.disableOnLastIteration(true);

  taskManager.addTask(tRewardOn); // pseudo priority
  taskManager.addTask(tTransmit);
  taskManager.addTask(tStimStart);
  taskManager.addTask(tStimStop);
  taskManager.addTask(tRespWinOpen);
  taskManager.addTask(tRespWinClose);
  taskManager.addTask(tRewardOff);
  taskManager.addTask(tAutoReward);
  taskManager.addTask(tRewardRemovalOn);
  taskManager.addTask(tRewardRemovalOff);
  taskManager.addTask(tEndTrial);
  taskManager.disableAll();

  dataString = "";

  trialConfigured = true;
  //Serial.println("{CONFIGURED}");
}

void runTrial() {
  startTime = millis();
  attachInterrupt(responsePin[0], response1, RISING);
  attachInterrupt(responsePin[1], response2, RISING);
  //attachInterrupt(responsePin[2], response3, RISING);
  trialRunning = true;
  digitalWrite(13, HIGH);

  // deliver trial cue
  if (trialCue) {
    Serial.println("trial cue");
  }

  tTransmit.enableDelayed();

  // pretrial withold requirement
  if (withold) {
    Serial.println("waiting for withold");
    witholdTimer = 0;
    inWithold = true;
    while (inWithold) {
      taskManager.execute();

      uint8_t SaveSREG = SREG; // save interrupt flag
      cli(); // disable interrupts
      if (witholdTimer >= witholdReq) {
        inWithold = false;
      }
      SREG = SaveSREG; // restore the interrupt flag
    }
  }


  // start trial
  now = millis() - startTime;
  startTime = millis();
  comString = "<*:";
  comString = comString + now + ">";
  Serial.println(comString);
  enableTasks();
  while (trialRunning) {
    taskManager.execute();
  }

  // end
  endTrial();
}

void enableTasks() {
  tStimStart.enableDelayed();
  tStimStop.enableDelayed();
  tRespWinOpen.enableDelayed();
  tRespWinClose.enableDelayed();
  tEndTrial.enableDelayed(trialDuration);
  if (autoReward) {
    tAutoReward.enableDelayed();
  }
}

void response1() {
  processResponse(1);
}

void response2() {
  processResponse(2);
}

void response3() {
  processResponse(3);
}

void processResponse(int responseNum) {
  uint8_t SaveSREG = SREG; // save interrupt flag
  cli(); // disable interrupts

  timeResponded = millis() - startTime;
  newDataString = "";
  newDataString = newDataString + responseNum + ":" + timeResponded + "|";
  dataString = dataString + newDataString;

  if (inWithold) { // or initate trial if response is required
    witholdTimer = 0;
  }
  else if ((inRespWin) && (respWinRespIdx == 0)) {
    if (responseNum == respReq) {
      tRewardOn.enable();
    }
    else {
      if (punish) {
      currentTime = millis() - startTime;
      tEndTrial.setInterval(trialDuration - currentTime + punishLength);
      }
    }
    respWinResponses[respWinRespIdx] = responseNum;
    respWinRespIdx = respWinRespIdx + 1;
  }

  // if post stim delay, cancel trial?

  SREG = SaveSREG; // restore the interrupt flag
}

void txData() {
  if (dataString != "") {
    //get length of current data string
    lengthDataString = dataString.length();

    //copy the data into new string to transmit
    txString = "<";
    txString = txString + dataString.substring(0, lengthDataString) + ">";

    //remove the tranmitted substring, whilst preserving any new data
    dataString.remove(0, lengthDataString);

    //print to serial
    Serial.println(txString);
  }
}

void stimOn() {
  Serial.println("stim on");
  digitalWrite(stimPin[stimChan], HIGH);
}

void stimOff() {
  Serial.println("stim off");
  digitalWrite(stimPin[stimChan], LOW);
}

void rewardOn() {
  Serial.print("reward on ");
  Serial.println(rewardPin[rewardChan]);
  digitalWrite(rewardPin[rewardChan], HIGH);
  if (rewardRemoval) {
    tRewardRemovalOn.enableDelayed();
  }
  tRewardOff.enableDelayed();
}

void autoRewardOn() {
  tRewardOn.enable();
}

void rewardOff() {
  Serial.println("reward off");
  digitalWrite(rewardPin[rewardChan], LOW);
}

void respWinOpen() {
  inRespWin = true;
  Serial.println("resp win on");
}

void respWinClose() {
  inRespWin = false;
  Serial.println("resp win off");
}

void rewardRemovalOn() {
  Serial.println("reward removal on");
  digitalWrite(rewardRemovalPin[rewardChan], HIGH);
  tRewardRemovalOff.enableDelayed();
}

void rewardRemovalOff() {
  Serial.println("reward removal off");
  digitalWrite(rewardRemovalPin[rewardChan], LOW);
}

void stopTasks() {
  trialRunning = false;
  taskManager.disableAll();
}

void endTrial() {
  detachInterrupt(responsePin[0]);
  detachInterrupt(responsePin[1]);
  //detachInterrupt(responsePin[2]);
  digitalWrite(13, LOW);
  for ( int i = 0; i < sizeof(rewardPin); ++i ) {
    digitalWrite(rewardPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(stimPin); ++i ) {
    digitalWrite(stimPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(punishPin); ++i ) {
    digitalWrite(punishPin[i], LOW);
  }
  for( int i = 0; i < sizeof(rewardRemovalPin);  ++i ) {
    digitalWrite(rewardRemovalPin[i], LOW);
  }
  txData();
  txResults();
  resetConfig();
}

void txResults() {
  // generate results
  long firstResponse = respWinResponses[0];
  if (respReq == 0) { // if no response rquired
    if (firstResponse) { // responded when not supposed to
      incorrect = true;
      fa = true;
    }
    else {
      correct = true;
      cr = true;
    }
  }
  else { // if response is required
    if (firstResponse == respReq) {
      correct = true;
      hit = true;
    }
    else if (firstResponse == 0) {
      incorrect = true;
      miss = true;
    }
    else {
      incorrect = true;
      fa = true;
    }
  }

  // transmit results
  String resultsString;
  resultsString = "{firstresponse:" + String(firstResponse) + "|";
  resultsString = resultsString + "correct:" + String(correct) + "|";
  resultsString = resultsString + "incorrect:" + String(incorrect) + "|";
  resultsString = resultsString + "hit:" + String(hit) + "|";
  resultsString = resultsString + "miss:" + String(miss) + "|";
  resultsString = resultsString + "fa:" + String(fa) + "|";
  resultsString = resultsString + "cr:" + String(cr) + "}";
  delay(100);
  Serial.println(resultsString);
  delay(250);
  Serial.println("{DONE}");
}

void resetConfig() {
  trialConfigured = false;
  trialRunning = false;
  configReceived = false;
  configStringStarted = false;
  configStringEnded = false;
  respWinRespIdx = 0;
  for ( int i = 0; i < sizeof(respWinResponses); ++i ) {
    respWinResponses[i] = (char)0;
  }
  correct = false;
  incorrect = false;
  hit = false;
  miss = false;
  cr = false;
  fa = false;
}

void testReward(int pinNumber) {
  Serial.print("reward ");
  Serial.println(pinNumber);
  digitalWrite(pinNumber, HIGH);
  delay(100);
  digitalWrite(pinNumber, LOW);
}

void testStim(int pinNumber) {
  Serial.print("stim ");
  Serial.println(pinNumber);
  digitalWrite(pinNumber, HIGH);
  delay(100);
  digitalWrite(pinNumber, LOW);
}
