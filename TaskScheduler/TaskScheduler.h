// Cooperative multitasking library for Arduino
// Copyright (c) 2015 Anatoli Arkhipenko
//
// Changelog:
//     2015-02-24 - Initial release 
//	   2015-02-28 - added delay() and disableOnLastIteration() functions

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


#include <Arduino.h>



#ifndef _TASKSCHEDULER_H_
#define _TASKSCHEDULER_H_

//#define _TASK_DEBUG
#define _TASK_TIMECRITICAL


#define TASKSCHEDULER_MAXDELAY	1000  // Tick every one second

class Task; 

class Scheduler {
	public:
		Scheduler();
		inline void init() { iFirst = NULL; iLast = NULL; iImmediateRequisted = false; }
		void addTask(Task& aTask);
		void deleteTask(Task& aTask);
		inline void setMaxDelay(unsigned long aDelay) { iMaxDelay = aDelay; }
		void disableAll();
		void enableAll();
		void execute();
		inline void requestImmediateExecution() { iImmediateRequisted = true; }

	private:
		unsigned long	iMaxDelay;
		Task			*iFirst, *iLast;
		unsigned long	iRunShortestInterval;
		bool			iImmediateRequisted;
};

class Task {
    friend class Scheduler;
    public:
	Task(unsigned long aInterval=0, long aIterations=0, void (*aCallback)()=NULL);

	void enable();
	void enableDelayed(unsigned long aDelay=0);
	void delay(unsigned long aDelay=0);
	void disable();
	inline bool isEnabled() { return iEnabled; }
	void set(unsigned long aInterval, long aIterations, void (*aCallback)());
	void setInterval(unsigned long aInterval);
	inline unsigned long getInterval() { return iInterval; }
	inline void setIterations(long aIterations) { iSetIterations = iIterations = aIterations; }
	inline long getIterations() { return iIterations; }
	inline void setCallback(void (*aCallback)()) { iCallback = aCallback; }
	inline void disableOnLastIteration(bool aBool) { iDisableOnLastIteration = aBool; }
#ifdef _TASK_TIMECRITICAL
	inline long getOverrun() { return iOverrun; }
#endif
	inline bool isFirstIteration() { return (iIterations >= iSetIterations-1); } 
	inline bool isLastIteration() { return (iIterations == 0); }

    private:
	void reset();

    volatile bool			iEnabled;
	volatile bool			iDisableOnLastIteration;
    volatile unsigned long	iInterval;
	volatile unsigned long	iPreviousMillis;
#ifdef _TASK_TIMECRITICAL
	volatile long		iOverrun; 
#endif
	volatile long		iIterations;
	long				iSetIterations; 
	void				(*iCallback)();
	Task				*iPrev, *iNext;
	Scheduler			*iScheduler;
};


#endif /* _TASKSCHEDULER_H_ */
