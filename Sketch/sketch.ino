////////////////////////////
// PyBehaviour            //
// (c) 2015 Lloyd Russell //
////////////////////////////
// 2016.02.06.1

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
Task tResponseWindowOpen;
Task tResponseWindowClose;
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
const int stimPin[] = {30,31,32,33,34,35,36,37};
const int rewardPin[] = {4, 5};
const int rewardRemovalPin[] = {8, 9};
const int punishPin[] = {6, 7};
const int cuePin[] = {10, 11};
const int stimVariationPin[] = {14, 15, 16, 17};
const int responseWindowPin = 12;
const int trialRunningPin = 13;  // LED

// serial communication
char incomingByte;
char varBuffer[30];
int varIndex = 0;
int varBufferIndex = 0;
bool configStringStarted = false;
bool configStringEnded = false;
bool trialStringStarted = false;
bool trialStringEnded = false;

// control
char INCOMING_COMMAND = '@';
char TEST_PIN = '!';
char TEST_READY = '?'; // test if ready
char FORCE_REWARD = 'R';
char test_command;
int testChanNum;
int testChanDur;
String ConfigString;

// training parameters
int stimChan;
int stimVariation;
int responseRequired;
int rewardChan;
bool trialStartCue;
bool stimStartCue;
bool responseWindowCue;
int responseWindowCueStartTime;
bool withold;
int witholdReq;
int stimStartTime;
int stimStopTime;
int responseWindowStartTime;
int responseWindowStopTime;
int trialDuration;
bool autoReward;
int autoRewardStartTime;
bool punishTrigger;
int punishChan;
bool punishDelay;
int punishLength;
bool rewardRemoval;
int rewardRemovalDelay;
int cueChan;
bool postStimCancel;
bool secondChance;
int rewardDuration;
int punishTriggerDuration;

// session/trial states
bool configReceived = false;
bool trialConfigured = false;
bool trialRunning;
bool inResponseWindow;
bool inWithold;

// response states
bool responded = false;
bool rewarded = false;
bool punished = false;
bool cancelled = false;
volatile int firstResponse = 0;

// record times
int preTrialDelayStart;
long startTime;
long now;
int timeRewarded;
int timePunished;
String dataString;
int lengthDataString;
String newDataString;
String txString;
String comString;
int responseNum;
volatile long timeResponded;
volatile long prevTimeResponded;
long currentTime;

// results
bool correct;
bool incorrect;
bool miss;
volatile bool resultsTransmitted;


//---------------------------------------------------------
// RUNS ONCE AT STARTUP
//---------------------------------------------------------

void setup() {
  Serial.begin(19200);
  Serial.println("{READY}");
  delay(10);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  pinMode(responseWindowPin, OUTPUT);
  digitalWrite(responseWindowPin, LOW);

  for ( int i = 0; i < 99; ++i ) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  // for ( int i = 0; i < sizeof(rewardPin); ++i ) {
  //   pinMode(rewardPin[i], OUTPUT);
  //   digitalWrite(rewardPin[i], LOW);
  // }
  // for ( int i = 0; i < sizeof(cuePin); ++i ) {
  //   pinMode(cuePin[i], OUTPUT);
  //   digitalWrite(cuePin[i], LOW);
  // }
  // for ( int i = 0; i < sizeof(stimPin); ++i ) {
  //   pinMode(stimPin[i], OUTPUT);
  //   digitalWrite(stimPin[i], LOW);
  // }
  // for ( int i = 0; i < sizeof(stimVariationPin); ++i ) {
  //   pinMode(stimVariationPin[i], OUTPUT);
  //   digitalWrite(stimVariationPin[i], LOW);
  // }
  // for ( int i = 0; i < sizeof(punishPin); ++i ) {
  //   pinMode(punishPin[i], OUTPUT);
  //   digitalWrite(punishPin[i], LOW);
  // }
  // for( int i = 0; i < sizeof(rewardRemovalPin);  ++i ) {
  //  pinMode(rewardRemovalPin[i], OUTPUT);
  //  digitalWrite(rewardRemovalPin[i], LOW);
  // }
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
          stimVariation = atoi(varBuffer);
          break;
        case 3:
          responseRequired = atoi(varBuffer);
          break;
        case 4:
          rewardChan = atoi(varBuffer) - 1;
          break;
        case 5:
          trialStartCue = atoi(varBuffer);
          break;
        case 6:
          stimStartCue = atoi(varBuffer);
          break;
        case 7:
          responseWindowCue = atoi(varBuffer);
          break;
        case 8:
          responseWindowCueStartTime = atoi(varBuffer);
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
          responseWindowStartTime = atoi(varBuffer);
          break;
        case 14:
          responseWindowStopTime = atoi(varBuffer);
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
          punishTrigger = atoi(varBuffer);
          break;
        case 19:
          punishChan = atoi(varBuffer) - 1;
          break;
        case 20:
          punishDelay = atoi(varBuffer);
          break;
        case 21:
          punishLength = atoi(varBuffer);
          break;
        case 22:
          rewardRemoval = atoi(varBuffer);
          break;
        case 23:
          rewardRemovalDelay = atoi(varBuffer);
          break;
        case 24:
          cueChan = atoi(varBuffer) - 1;
          break;
        case 25:
          postStimCancel = atoi(varBuffer);
          break;
        case 26:
          secondChance = atoi(varBuffer);
          break;
        case 27:
          rewardDuration = atoi(varBuffer);
          break;
        case 28:
          punishTriggerDuration = atoi(varBuffer);
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

  tTransmit.set(200, -1, &txData);
  tCueOn.set(0, 0, &cueOn);
  tCueOff.set(50, 0, &cueOff);
  tStimStart.set(stimStartTime, 1, &stimOn);
  tStimStop.set(stimStopTime, 1, &stimOff);
  tResponseWindowOpen.set(responseWindowStartTime, 1, &responseWindowOpen);
  tResponseWindowClose.set(responseWindowStopTime, 1, &responseWindowClose);
  tRewardOn.set(0, 1, &rewardOn);
  tRewardOff.set(rewardDuration, 1, &rewardOff);
  tAutoReward.set(autoRewardStartTime, 1, &autoRewardOn);
  tRewardRemovalOn.set(rewardRemovalDelay, 1, &rewardRemovalOn);
  tRewardRemovalOff.set(50, 1, &rewardRemovalOff);
  tPunishOn.set(0, 1, &punishOn);
  tPunishOff.set(punishTriggerDuration, 1, &punishOff);
  tEndTrial.set(trialDuration, 1, &stopTasks);

  tTransmit.disableOnLastIteration(true);
  tCueOn.disableOnLastIteration(false);
  tCueOff.disableOnLastIteration(false);
  tStimStart.disableOnLastIteration(true);
  tStimStop.disableOnLastIteration(true);
  tResponseWindowOpen.disableOnLastIteration(true);
  tResponseWindowClose.disableOnLastIteration(true);
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
  taskManager.addTask(tResponseWindowOpen);
  taskManager.addTask(tResponseWindowClose);
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

  int bits[] = {0,0,0,0};
  for (int i = 3; i >= 0; i--) {
    bits[i] = (stimVariation & (1 << i)) != 0;
    digitalWrite(stimVariationPin[i], bits[i]);
  }

  // deliver trial cue
  if (trialStartCue) {
    Serial.println("trial cue");
    tCueOn.setIterations(1);  // reset iterations to allow repeat
    tCueOff.setIterations(1);  // reset iterations to allow repeat
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
    serialListen();
  }

  // end
  endTrial();
}

void serialListen() {
  if (Serial.available()) {
    incomingByte = Serial.read();
    if (incomingByte == INCOMING_COMMAND) {
      delay(1);
      test_command = Serial.read();
      // delay(1);
      if (test_command == FORCE_REWARD) {
        tRewardOn.enable();
      }
    }
  }
}

void enableTasks() {
  tStimStart.enableDelayed();
  tStimStop.enableDelayed();
  tResponseWindowOpen.enableDelayed();
  tResponseWindowClose.enableDelayed();
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

// void response3() {
//   processResponse(3);
// }

void processResponse(int responseNum) {
  uint8_t SaveSREG = SREG; // save interrupt flag
  cli(); // disable interrupts

  timeResponded = millis() - startTime;
  if (timeResponded > prevTimeResponded) {
    newDataString = "";
    newDataString = newDataString + responseNum + ":" + timeResponded + "|";
    dataString = dataString + newDataString;

    if (inWithold) { // include here active initiation trial if response is required
      witholdTimer = 0;
    }
    else if ((timeResponded < responseWindowStartTime) && (postStimCancel)) {  // if post stim delay, cancel trial
      cancelled = true;
      tEndTrial.enable();
    }
    else if (inResponseWindow) {
      if (!responded) {
        firstResponse = responseNum;
        responded = true;
        if (responseNum == responseRequired) {  // correct choice
          if ((!rewarded) && !(punished && !secondChance)) {
            tRewardOn.enable();
            txResults();
          }
        }
        else {  // else must be incorrect
          if ((!punished) && (!rewarded)) {
            if (punishTrigger) {
              tPunishOn.enable();
            }
            if (punishDelay) {
              currentTime = millis() - startTime;
              tEndTrial.setInterval(trialDuration - currentTime + punishLength);
            }
          txResults();
          }
        }
      }
    }
  }
  prevTimeResponded = timeResponded;
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
  digitalWrite(stimPin[stimChan], HIGH);
  if (stimStartCue) {
    tCueOn.setIterations(1);  // reset iterations to allow repeat
    tCueOff.setIterations(1);  // reset iterations to allow repeat
    tCueOn.enable();
  }
  Serial.println("stim on");
}

void stimOff() {
  digitalWrite(stimPin[stimChan], LOW);
  Serial.println("stim off");
}

void rewardOn() {
  digitalWrite(rewardPin[rewardChan], HIGH);
  rewarded = true;
  timeRewarded = millis() - startTime;
  Serial.println("reward on");
  if (rewardRemoval) {
    tRewardRemovalOn.enableDelayed();
  }
  tRewardOff.enableDelayed();
}

void autoRewardOn() {
  tRewardOn.enable();
}

void rewardOff() {
  digitalWrite(rewardPin[rewardChan], LOW);
}

void cueOn() {
  Serial.println("cue on");
  digitalWrite(cuePin[cueChan], HIGH);
  tCueOff.enableDelayed();
}

void cueOff() {
  digitalWrite(cuePin[cueChan], LOW);
}

void punishOn() {
  digitalWrite(punishPin[punishChan], HIGH);
  punished = true;
  timePunished = millis() - startTime;
  Serial.println("punish on");
  tPunishOff.enableDelayed();
}

void punishOff() {
  digitalWrite(punishPin[punishChan], LOW);
}

void responseWindowOpen() {
  inResponseWindow = true;
  digitalWrite(responseWindowPin, HIGH);
  Serial.println("response window open");
  if (responseWindowCue) {
    tCueOn.setIterations(1);  // reset iterations to allow repeat
    tCueOff.setIterations(1);  // reset iterations to allow repeat
    tCueOn.enable();
  }
}

void responseWindowClose() {
  inResponseWindow = false;
  digitalWrite(responseWindowPin, LOW);
  Serial.println("response window closed");
}

void rewardRemovalOn() {
  digitalWrite(rewardRemovalPin[rewardChan], HIGH);
  Serial.println("reward removal on");
  tRewardRemovalOff.enableDelayed();
}

void rewardRemovalOff() {
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
  for ( int i = 0; i < sizeof(stimVariationPin); ++i ) {
    digitalWrite(stimVariationPin[i], LOW);
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
  // delay(250);
  Serial.println("{DONE}");  // tells python gui trial has finished
}

void txResults() {
  if (resultsTransmitted == false) {
    resultsTransmitted = true;

    // generate results
    if (firstResponse == responseRequired) {
      correct = true;
    }
    else if (firstResponse == 0) {
      miss = true;
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

  }
}

void resetConfig() {
  trialConfigured = false;
  trialRunning = false;
  configReceived = false;
  configStringStarted = false;
  configStringEnded = false;
  firstResponse = 0;
  rewarded = false;
  punished = false;
  responded = false;
  correct = false;
  incorrect = false;
  miss = false;
  cancelled = false;
  resultsTransmitted = false;
  prevTimeResponded = 0;
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

