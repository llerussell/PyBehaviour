////////////////////////////
// PyBehaviour            //
// (c) 2015 Lloyd Russell //
////////////////////////////

#include <elapsedMillis.h>
#include <TaskScheduler.h>

//---------------------------------------------------------
// INITIALISE
//---------------------------------------------------------

Task tTransmit;
Task tCueOn;
Task tCueOff;
Task tStimStart;
Task tStimStop;
Task tRespWinOpen;
Task tRespWinClose;
Task tAutoReward;
Task tRewardOn;
Task tRewardOff;
Task tRewardRemovalOn;
Task tRewardRemovalOff;
Task tPunishOn;
Task tPunishOff;
Task tEndTrial;

Scheduler taskManager;

elapsedMillis witholdTimer;

// pin numbers
const int responsePin[] = {0, 1}; // interrupt numbers 0:2 1:3 [2:21 do not use] 3:20
const int stimPin[] = {22, 23};
const int rewardPin[] = {4, 5};
const int rewardRemovalPin[] = {8, 9};
const int punishPin[] = {6, 7};
const int cuePin[] = {10};
const int stimVarPin[] = {14, 15, 16, 17};
const int respWinPin = 99;
const int trialRunningPin = 13;

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
char TEST_PIN = '!';
char TEST_READY = '?'; // test if ready
char test_command;
int testChanNum;
int testChanDur;
String ConfigString;

// training parameters
int stimChan;
int stimVar;
int respReq;
int rewardChan;
boolean trialCue;
boolean stimCue;
boolean respCue;
int respCueStartTime;
boolean withold;
int witholdReq;
int stimStartTime;
int stimStopTime;
int respWinStartTime;
int respWinStopTime;
int trialDuration;
boolean autoReward;
int autoRewardStartTime;
int punishChan;
int punishLength;
boolean rewardRemoval;
int rewardRemovalDelay;
int cueChan;
boolean postStimCancel;

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
  for ( int i = 0; i < sizeof(cuePin); ++i ) {
    pinMode(cuePin[i], OUTPUT);
    digitalWrite(cuePin[i], LOW);
  }
  for ( int i = 0; i < sizeof(stimPin); ++i ) {
    pinMode(stimPin[i], OUTPUT);
    digitalWrite(stimPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(stimVarPin); ++i ) {
    pinMode(stimVarPin[i], OUTPUT);
    digitalWrite(stimVarPin[i], LOW);
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

    // TEST PIN
    if (incomingByte == INCOMING_COMMAND) {
      delay(1);
      test_command = Serial.read();
      delay(1);
      if (test_command == TEST_PIN) {
        varBufferIndex = 0;
        varBuffer[varBufferIndex] = '\0';
        varIndex = 0;
        while (Serial.available()) {
          delay(1);
          incomingByte = Serial.read();
          if (incomingByte == ';') {
            // when reach seperator ; save the variable
            varIndex++;
            switch (varIndex) {
              case 1:
                testChanNum = atoi(varBuffer);
                break;
              case 2:
                testChanDur = atoi(varBuffer);
                break;
            }
            varBufferIndex = 0;
            varBuffer[varBufferIndex] = '\0';
          }
          else {
            // store in the buffer
            varBuffer[varBufferIndex] = incomingByte;
            varBufferIndex++;
            varBuffer[varBufferIndex] = '\0';
          }
        }
        testPin(testChanNum, testChanDur);
      }

      else if (test_command == TEST_READY) {
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
          break;
        case 2:
          stimVar = atoi(varBuffer) - 1;
          break;
        case 3:
          respReq = atoi(varBuffer);
          break;
        case 4:
          rewardChan = atoi(varBuffer) - 1;
          break;
        case 5:
          trialCue = atoi(varBuffer);
          break;
        case 6:
          stimCue = atoi(varBuffer);
          break;
        case 7:
          respCue = atoi(varBuffer);
          break;
        case 8:
          respCueStartTime = atoi(varBuffer);
          break;
        case 9:
          withold = atoi(varBuffer);
          break;
        case 10:
          witholdReq = atoi(varBuffer);
          break;
        case 11:
          stimStartTime = atoi(varBuffer);
          break;
        case 12:
          stimStopTime = atoi(varBuffer);
          break;
        case 13:
          respWinStartTime = atoi(varBuffer);
          break;
        case 14:
          respWinStopTime = atoi(varBuffer);
          break;
        case 15:
          trialDuration = atoi(varBuffer);
          break;
        case 16:
          autoReward = atoi(varBuffer);
          break;
        case 17:
          autoRewardStartTime = atoi(varBuffer);
          break;
        case 18:
          punishChan = atoi(varBuffer);
          break;
        case 19:
          punishLength = atoi(varBuffer);
          break;
        case 20:
          rewardRemoval = atoi(varBuffer);
          break;
        case 21:
          rewardRemovalDelay = atoi(varBuffer);
          break;
        case 22:
          cueChan = atoi(varBuffer);
          break;
        case 23:
          postStimCancel = atoi(varBuffer);
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
  tCueOn.set(0, 0, &cueOn);
  tCueOff.set(50, 0, &cueOff);
  tStimStart.set(stimStartTime, 1, &stimOn);
  tStimStop.set(stimStopTime, 1, &stimOff);
  tRespWinOpen.set(respWinStartTime, 1, &respWinOpen);
  tRespWinClose.set(respWinStopTime, 1, &respWinClose);
  tRewardOn.set(0, 1, &rewardOn);
  tRewardOff.set(50, 1, &rewardOff);
  tAutoReward.set(autoRewardStartTime, 1, &autoRewardOn);
  tRewardRemovalOn.set(rewardRemovalDelay, 1, &rewardRemovalOn);
  tRewardRemovalOff.set(50, 1, &rewardRemovalOff);
  tPunishOn.set(0, 1, &punishOn);
  tPunishOff.set(50, 1, &punishOff);
  tEndTrial.set(trialDuration, 1, &stopTasks);

  tTransmit.disableOnLastIteration(true);
  tCueOn.disableOnLastIteration(false);
  tCueOff.disableOnLastIteration(false);
  tStimStart.disableOnLastIteration(true);
  tStimStop.disableOnLastIteration(true);
  tRespWinOpen.disableOnLastIteration(true);
  tRespWinClose.disableOnLastIteration(true);
  tRewardOn.disableOnLastIteration(true);
  tRewardOff.disableOnLastIteration(true);
  tAutoReward.disableOnLastIteration(true);
  tRewardRemovalOn.disableOnLastIteration(true);
  tRewardRemovalOff.disableOnLastIteration(true);
  tPunishOn.disableOnLastIteration(true);
  tPunishOff.disableOnLastIteration(true);
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
  taskManager.addTask(tPunishOn);
  taskManager.addTask(tPunishOff);
  taskManager.addTask(tEndTrial);
  taskManager.addTask(tCueOn);
  taskManager.addTask(tCueOff);
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
  digitalWrite(trialRunningPin, HIGH);

  // deliver trial cue
  if (trialCue) {
    Serial.println("trial cue");
    tCueOn.enable();
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
  else if ((timeResponded < respWinStartTime) && (postStimCancel)) {  // if post stim delay, cancel trial
    tEndTrial.enable();
  }
  else if ((inRespWin) && (respWinRespIdx == 0)) {
    if (responseNum == respReq) {
      tRewardOn.enable();
    }
    else {
      if (punishChan > 0) {
      currentTime = millis() - startTime;
      }
      if (punishLength > 0) {
      tEndTrial.setInterval(trialDuration - currentTime + punishLength);
      }
    }
    respWinResponses[respWinRespIdx] = responseNum;
    respWinRespIdx = respWinRespIdx + 1;
  }

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

  // stim variation, binary 'barcode' (1:16)
  int bits[] = {0,0,0,0};
    for (int i = 3; i >= 0; i--) {
        bits[i] = (stimVar & (1 << i)) != 0;
        digitalWrite(stimVarPin[i], bits[i]);
    }
  if (stimCue) {
    tCueOn.enable();
  }
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

void cueOn() {
  Serial.println("cue on");
  digitalWrite(cuePin[cueChan], HIGH);
  tCueOn.setIterations(1);  // reset iterations to allow repeat
  tCueOff.setIterations(1);  // reset iterations to allow repeat
  tCueOff.enableDelayed();
}

void cueOff() {
  Serial.println("cue off");
  digitalWrite(cuePin[cueChan], LOW);
}

void punishOn() {
  Serial.println("punish on");
  digitalWrite(punishPin[punishChan], HIGH);
  tPunishOff.enableDelayed();
}

void punishOff() {
  Serial.println("punish off");
  digitalWrite(punishPin[punishChan], LOW);
}

void respWinOpen() {
  inRespWin = true;
  Serial.println("resp win on");
  if (stimCue) {
    tCueOn.enable();
  }
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
  digitalWrite(trialRunningPin, LOW);

  for ( int i = 0; i < sizeof(responsePin); ++i ) {
    detachInterrupt(responsePin[i]);
  }
  for ( int i = 0; i < sizeof(cuePin); ++i ) {
    digitalWrite(cuePin[i], LOW);
  }
  for ( int i = 0; i < sizeof(rewardPin); ++i ) {
    digitalWrite(rewardPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(stimPin); ++i ) {
    digitalWrite(stimPin[i], LOW);
  }
  for ( int i = 0; i < sizeof(stimVarPin); ++i ) {
    digitalWrite(stimVarPin[i], LOW);
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
  if (firstResponse == respReq) {
    correct = true;
  }
  else if (firstResponse == 0) {
    incorrect = true;
  }
  else {
    incorrect = true;
  }

  // transmit results
  String resultsString;
  resultsString = "{firstresponse:" + String(firstResponse) + "|";
  resultsString = resultsString + "correct:" + String(correct) + "|";
  resultsString = resultsString + "incorrect:" + String(incorrect) + "|";
  resultsString = resultsString + "miss:" + String(miss) + "}";
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

void testPin(int pinNumber, int pinDuration) {
  Serial.print("Test pin: ");
  Serial.print(pinNumber);
  Serial.print(", Test duration: ");
  Serial.println(pinDuration);
  digitalWrite(pinNumber, HIGH);
  delay(pinDuration);
  digitalWrite(pinNumber, LOW);
}

