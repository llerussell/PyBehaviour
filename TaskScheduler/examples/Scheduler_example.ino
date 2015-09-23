#include <TaskScheduler.h>

Task task;
Scheduler scheduler;

void setup () {
  pinMode(13, OUTPUT);
}

void loop () {
  config();
  runTrial();
}

void runTrial() {
  scheduler.enableAll();
  while(1) {
    scheduler.execute();
  }
}

void config() {
  scheduler.init();
  task.set(1000, 5, &taskCallback);
  scheduler.addTask(task);
  delay(5000);
}

void taskCallback() {
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}