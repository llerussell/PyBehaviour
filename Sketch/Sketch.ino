////////////////////////////
// PyBehaviour            //
// (c) 2015 Lloyd Russell //
////////////////////////////
// 2017.03.17.1

#include "elapsedMillis.h"
#include "TaskScheduler.h"

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
const int responsePin[] = {2, 3}; // interrupt:pin numbers 0:2 1:3 [2:21 do not use?] 3:20
const int stimPin[] = {30, 31, 32, 33, 34, 35, 36, 37};
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
bool trialActuallyStartedCue;
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
volatile bool responded = false;
volatile bool rewarded = false;
volatile bool punished = false;
volatile bool cancelled = false;
volatile int firstResponse = 0;

// record times
int preTrialDelayStart;
long startTime;
long now;
volatile int timeRewarded;
volatile int timePunished;
String dataString;
volatile int lengthDataString;
String newDataString;
String txString;
String comString;
String resultsString;
volatile int responseNum;
volatile long timeResponded;
volatile long prevTimeResponded;
volatile long currentTime;

// results
volatile bool correct;
volatile bool incorrect;
volatile bool miss;
volatile bool cheated;
volatile bool resultsTransmitted;

// running to initiate trials
bool runToInitiate = true;
const int inputPin = A0;
long analogVal;
long analogZero = 123;
long runningTimeTarget = 2000;  // milliseconds
int runningSpeedThresh = 10;
int runningTimeReset = 500;
long runningAccumulator;
long runningVal;
bool isRunning = false;
bool startTrial = false;
elapsedMillis runningTimer;
long lastRunTime;
bool enforceStop = false;


//---------------------------------------------------------
// RUNS ONCE AT STARTUP
//---------------------------------------------------------

void setup() {
  Serial.begin(19200);
  Serial.println("{READY}");
  delay(10);

  // set all pins as output
  for ( int i = 0; i < 54; ++i ) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  digitalWrite(inputPin, INPUT_PULLUP);
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
          trialActuallyStartedCue = atoi(varBuffer);
          break;
        case 7:
          stimStartCue = atoi(varBuffer);
          break;
        case 8:
          responseWindowCue = atoi(varBuffer);
          break;
        case 9:
          responseWindowCueStartTime = atoi(varBuffer);
          break;
        case 10:
          withold = atoi(varBuffer);
          break;
        case 11:
          witholdReq = atoi(varBuffer);
          break;
        case 12:
          stimStartTime = atoi(varBuffer);
          break;
        case 13:
          stimStopTime = atoi(varBuffer);
          break;
        case 14:
          responseWindowStartTime = atoi(varBuffer);
          break;
        case 15:
          responseWindowStopTime = atoi(varBuffer);
          break;
        case 16:
          trialDuration = atoi(varBuffer);
          break;
        case 17:
          autoReward = atoi(varBuffer);
          break;
        case 18:
          autoRewardStartTime = atoi(varBuffer);
          break;
        case 19:
          punishTrigger = atoi(varBuffer);
          break;
        case 20:
          punishChan = atoi(varBuffer) - 1;
          break;
        case 21:
          punishDelay = atoi(varBuffer);
          break;
        case 22:
          punishLength = atoi(varBuffer);
          break;
        case 23:
          rewardRemoval = atoi(varBuffer);
          break;
        case 24:
          rewardRemovalDelay = atoi(varBuffer);
          break;
        case 25:
          cueChan = atoi(varBuffer) - 1;
          break;
        case 26:
          postStimCancel = atoi(varBuffer);
          break;
        case 27:
          secondChance = atoi(varBuffer);
          break;
        case 28:
          rewardDuration = atoi(varBuffer);
          break;
        case 29:
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
    ConfigString.concat("withold_req:");
    ConfigString.concat(witholdReq);
    ConfigString.concat("}");
    Serial.println(ConfigString);
  }
}

void configTrial() {
  // set(delay, repeat, function)
  taskManager.init();

  tTransmit.set(50, -1, &txData);
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

  // tTransmit.disableOnLastIteration(true);
  // tCueOn.disableOnLastIteration(false);
  // tCueOff.disableOnLastIteration(false);
  // tStimStart.disableOnLastIteration(true);
  // tStimStop.disableOnLastIteration(true);
  // tResponseWindowOpen.disableOnLastIteration(true);
  // tResponseWindowClose.disableOnLastIteration(true);
  // tRewardOn.disableOnLastIteration(true);
  // tRewardOff.disableOnLastIteration(true);
  // tAutoReward.disableOnLastIteration(true);
  // tRewardRemovalOn.disableOnLastIteration(true);
  // tRewardRemovalOff.disableOnLastIteration(true);
  // tPunishOn.disableOnLastIteration(true);
  // tPunishOff.disableOnLastIteration(true);
  // tEndTrial.disableOnLastIteration(true);

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
  attachInterrupt(digitalPinToInterrupt(responsePin[0]), response1, RISING);
  attachInterrupt(digitalPinToInterrupt(responsePin[1]), response2, RISING);
  //attachInterrupt(responsePin[2], response3, RISING);
  trialRunning = true;
  digitalWrite(trialRunningPin, HIGH);

  // stim variation, binary 'barcode' (0:15)
  int bits[] = {0, 0, 0, 0};
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

  if (runToInitiate) {
    Serial.println("waiting for running");
    while (!startTrial) {
      // initate the 'running' loop
      runningVal = analogRead(inputPin);
      //  Serial.print("analog: ");
      //  Serial.println(runningVal);
      if (runningVal > analogZero + runningSpeedThresh) {
        // animal is moving
        isRunning = true;
        runningTimer = 0;
        runningAccumulator = 0;
        Serial.println("Running started");

        while (isRunning) {
          runningVal = analogRead(inputPin);
          //      Serial.print("analog: ");
          //      Serial.println(runningVal);
          if (runningVal <= analogZero + runningSpeedThresh) {  // is animal still running?
            if (runningTimer - lastRunTime > runningTimeReset) { // has animal run in the past X milliseconds?
              isRunning = false;
              Serial.println("Running stopped");
            }
          }

          else {  // animal is running
            lastRunTime = runningTimer;
            runningAccumulator = runningAccumulator + runningVal;
//            Serial.println(runningTimer);
          }

          if (runningTimer >= runningTimeTarget) {
            // has been running long enough, now wait for animal to stop?
            if (enforceStop) {

            }
            isRunning = false;
            startTrial = true;
          }

        taskManager.execute();
        }
      }
      taskManager.execute();
    }
  }

  // deliver trial cue
  if (trialActuallyStartedCue) {
    Serial.println("trial cue");
    tCueOn.setIterations(1);  // reset iterations to allow repeat
    tCueOff.setIterations(1);  // reset iterations to allow repeat
    tCueOn.enable();
  }

  // start trial
  now = millis() - startTime;
  startTime = millis();
  comString = "<*:";
  comString.concat(now);
  comString.concat(">");
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
        cheated = true;
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
    newDataString.concat(responseNum);
    newDataString.concat(":");
    newDataString.concat(timeResponded);
    newDataString.concat("|");
    dataString.concat(newDataString);

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
          if ((!rewarded) && !(incorrect && !secondChance)) {
            tRewardOn.enable();
            txResults();
          }
        }
        else {  // else must be incorrect
          incorrect = true;
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
    txString.concat(dataString.substring(0, lengthDataString));
    txString.concat(">");

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
  if ((!rewarded) && !(incorrect && !secondChance)) {
    tRewardOn.enable();
  }
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

  for ( int i = 0; i < 54; ++i ) {
    digitalWrite(i, LOW);
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
    resultsString = "{firstresponse:";
    resultsString.concat(firstResponse);
    resultsString.concat("|correct:");
    resultsString.concat(correct);
    resultsString.concat("|incorrect:");
    resultsString.concat(incorrect);
    resultsString.concat("|miss:");
    resultsString.concat(miss);
    resultsString.concat("|cheated:");
    resultsString.concat(cheated);
    resultsString.concat("}");
    // delay(100);
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
  cheated = false;
  cancelled = false;
  resultsTransmitted = false;
  prevTimeResponded = 0;
  isRunning = false;
  startTrial = false;
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


