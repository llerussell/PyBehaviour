// Cooperative multitasking library for Arduino
// Copyright (c) 2015 Anatoli Arkhipenko
//
// Changelog:
//     2015-02-24 - Initial release 
//	   2015-02-28 - added support for disable on last iteration

/* ============================================
Cooperative multitasking library code is placed under the MIT license
Copyright (c) 2015 Anatoli Arkhipenko

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "TaskScheduler.h"

// ------------------ Task implementation --------------------

/** Constructor, uses default default values for the parameters
 * so could be called with no parameters.
 */
Task::Task(unsigned long aInterval, long aIterations, void (*aCallback)()) {
	reset();
	set(aInterval, aIterations, aCallback);

}

/** Resets (initializes) the task/
 * Task is not enabled and is taken out 
 * out of the execution chain as a result
 */
void Task::reset() {
	iEnabled = false;
	iPreviousMillis = 0;
	iPrev = NULL;
	iNext = NULL;
	iScheduler = NULL;
	iDisableOnLastIteration = false;
#ifdef _TASK_TIMECRITICAL
	iOverrun = 0;
#endif
}

/** Explicitly set Task execution parameters
 * @param aInterval - execution interval in ms
 * @param aIterations - number of iterations, use -1 for no limet
 * @param aCallback - pointer to the callback function which executes the task actions
 */
void Task::set(unsigned long aInterval, long aIterations, void (*aCallback)()) {
	iInterval = aInterval;
	iSetIterations = iIterations = aIterations;
	iCallback = aCallback;
}

/** Enables the task 
 * and schedules it for execution as soon as possible
 */
void Task::enable() {
	iEnabled = true;
	iPreviousMillis = millis() - iInterval;
	if (iScheduler) iScheduler->requestImmediateExecution();
}

/** Enables the task 
 * and schedules it for execution after a delay = aInterval
 */
void Task::enableDelayed(unsigned long aDelay) {
	iEnabled = true;
	delay(aDelay);
}

/** Delays Task for execution after a delay = aInterval (if task is enabled).
 * leaves task enabled or disabled
 * if aDelay is zero, delays for the original scheduling interval from now
 */
void Task::delay(unsigned long aDelay) {
	if (!aDelay) aDelay = iInterval;
	iPreviousMillis = millis() - iInterval + aDelay;
	if (iScheduler) iScheduler->requestImmediateExecution();
}

/** Sets the execution interval.
 * Task execution is delayed for aInterval
 * Use  enable() to schedule execution ASAP
 * @param aInterval - new execution interval
 */
void Task::setInterval (unsigned long aInterval) {
 	iInterval = aInterval; 
	delay();
}

/** Disables task
 * Task will no loner be executed by the scheduler
 */
void Task::disable() {
	iEnabled = false;
}


// ------------------ Scheduler implementation --------------------

/** Default constructor.
 * Creates a scheduler with an empty execution chain.
 */
Scheduler::Scheduler() {
	setMaxDelay(TASKSCHEDULER_MAXDELAY);
	init();
}

/** Appends task aTask to the tail of the execution chain.
 * @param &aTask - reference to the Task to be appended.
 * @note Task can only be part of the chain once.
 */
 void Scheduler::addTask(Task& aTask) {

	aTask.iScheduler = this;
// First task situation: 
	if (iFirst == NULL) {
		iFirst = &aTask;
		aTask.iPrev = NULL;
	}
	else {
// This task gets linked back to the previous last one
		aTask.iPrev = iLast;
		iLast->iNext = &aTask;
	}
// "Previous" last task gets linked to this one - as this one becomes the last one
	aTask.iNext = NULL;
	iLast = &aTask;
}

/** Deletes specific Task from the execution chain
 * @param &aTask - reference to the task to be deleted from the chain
 */
void Scheduler::deleteTask(Task& aTask) {
	if (aTask.iPrev == NULL) {
		if (aTask.iNext == NULL) {
			iFirst = NULL;
			iLast = NULL;
			return;
		}
		else {
			aTask.iNext->iPrev = NULL;
			iFirst = aTask.iNext;
			aTask.iNext = NULL;
			return;
		}
	}

	if (aTask.iNext == NULL) {
		aTask.iPrev->iNext = NULL;
		iLast = aTask.iPrev;
		aTask.iPrev = NULL;
		return;
	}

	aTask.iPrev->iNext = aTask.iNext;
	aTask.iNext->iPrev = aTask.iPrev;
	aTask.iPrev = NULL;
	aTask.iNext = NULL;
}

/** Disables all tasks in the execution chain
 * Convenient for error situations, when the only
 * task remaining active is an error processing task
 */
void Scheduler::disableAll() {
	Task	*current = iFirst;
	while (current) {
		current->disable();
		current = current->iNext;
	}
}


/** Enables all the tasks in the execution chain
 */
 void Scheduler::enableAll() {
	Task	*current = iFirst;
	while (current) {
		current->enable();
		current = current->iNext;
	}
}

/** Makes one pass through the execution chain.
 * Tasks are executed in the order they were added to the chain
 * There is no concept of priority
 * Different pseudo "priority" could be achieved
 * by running task more frequently 
 */
void Scheduler::execute() {
	Task	*current = iFirst;
	unsigned long currentTimer = millis();
	if (!iImmediateRequisted) { iRunShortestInterval = iMaxDelay; }
	else { iRunShortestInterval = 0; }
	while (current) { 
		do {   		
			if (!current->iEnabled) break; 
	   		if (current->iIterations == 0) {
				if (current->iDisableOnLastIteration) current->disable();
				break;
			}
			if (current->iInterval > 0) {
				unsigned long targetMillis = current->iPreviousMillis + current->iInterval;
				if (targetMillis > currentTimer) {
//	This task is not ready to run yet, let's see how much time is left so we can properly delay until next tick
				       	unsigned long r = targetMillis - millis();
				       	if (r > current->iInterval) r = current->iInterval; //Just in case we roll over
				       	iRunShortestInterval = (iRunShortestInterval > r) ? r : iRunShortestInterval;
				       	break;
				}
				else {
//	Execute the callback first, then do the "scheduling" stuff
					if (current->iIterations > 0) current->iIterations--;  // do not decrement (-1) being a signal of eternal task
					if (current->iCallback) (*(current->iCallback))();

					current->iPreviousMillis += current->iInterval;

#ifdef _TASK_TIMECRITICAL
// Updated_previous+current should put us into the future, so iOverrun should be positive or zero. 
// If negative - the task is behind (next execution time is already in the past) 
					current->iOverrun = (long) (current->iPreviousMillis + current->iInterval - currentTimer);
#endif
					iRunShortestInterval = (iRunShortestInterval > current->iInterval) ? current->iInterval : iRunShortestInterval;
					break;
				}
			}
			// This task executes every time, so no delay
			iRunShortestInterval = 0L;
			if (current->iIterations > 0) current->iIterations--;  // do not decrement (-1) being a signal of eternal task
			if (current->iCallback) (*(current->iCallback))();
		} while (0); //guaranteed single run - allows use of "break" to exit 
		current = current->iNext;
	}

  	if (iRunShortestInterval) delay(iRunShortestInterval);
	iImmediateRequisted = false;
}

