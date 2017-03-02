# <img src="http://imgur.com/NKyhO5c.png" height=30px> PyBehaviour

## Description
Software for control of stimuli presentation, recording of responses, and delivery of associated consequences in associative learning (operant conditioning) behavioural paradigms. 

## Screenshots
![Imgur](http://imgur.com/2tMiybH.png)

## Prerequisites
* Python (3.6 recommended)
  * numpy
  * matplotlib
  * scipy
  * seaborn
  * PyQt5
  * pyserial
* Arduino (MEGA 2560 recommended)
  * [TaskScheduler](http://playground.arduino.cc/Code/TaskScheduler) - NOTE: use the version included in this repository
  * [ElapsedMillis](http://playground.arduino.cc/Code/ElapsedMillis) - NOTE: use the version included in this repository

## Installation
* Install [Anaconda](https://www.continuum.io/downloads) (Python 3.6)
  * Install PySerial `conda install pyserial`
  * Install seaborn `conda install seaborn`
* Download or clone [PyBehaviour](https://github.com/llerussell/PyBehaviour/archive/master.zip)
  * Unzip to folder of your choice, e.g. `C:/git/PyBehaviour`
* Upload `Sketch/sketch.ino` to the arduino
  * First, install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
* Double click on `execute.bat` (Tip: make a shorcut and place it on your Desktop)

## Known issues
* Interrupt 2 (pin 21) not compatible? Might be device specific.

## To do
* Implement auto transition
* Implement adpatation phase, no stimuli, all responses are rewarded
* Major overhaul: increase flexibility, extensibility:
    * define arduino pin numbers in python GUI (sub window?), not hardcoded in arduino sketch
    * rework arduino functions to be more generic e.g. PinOn rather than StimOn + RewardOn + PunishOn, etc
    * move toward generic timed events that are completely adjustable, including giving names to, rather than defined Stim, Response, Reward, Punish etc events
    * implement control over number of possible stim types in GUI (currently static, 8)
    * dynamically define 'trial types' composed of multiple 'states' wherein actions are associated with triggering 'events'

## Tested compatibility
* Windows 7, 8.1, 10 (64-bit)
* Mac OS X 10.10

## Contributors / beta testers
* Henry
* Oliver
* Christina
* Carmen
* Adam
