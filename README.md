# PyBehaviour
![Imgur](http://i.imgur.com/FoJIX5g.png)

## Prerequisites
* Python (3.4 recommended)
  * numpy
  * matplotlib
  * scipy
  * seaborn
  * PyQt5
* Arduino (MEGA 2560 recommended)
  * [TaskScheduler](http://playground.arduino.cc/Code/TaskScheduler) - NOTE: use the version included in this repository
  * [ElapsedMillis](http://playground.arduino.cc/Code/ElapsedMillis) - NOTE: use the version included in this repository

## Install instructions
Windows 64-bit:
* Install [winpython](http://winpython.github.io) (includes PyQt5)
 * Append python directory (e.g. `C:\WinPython-64bit-3.4.3.5\python-3.4.3.amd64`) to system PATH
* Unzip the PyBehaviour download
* Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
 * Upload `Sketch/sketch.ino` to the arduino
* Double click on `PyBehaviour.exe` (you can also make a shorcut and place it on your Desktop)

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

## Tested compatibility
* Windows 7, 8.1 (64-bit)
* Mac OS X 10.10

## Contributors / beta testers
* Henry
* Oliver
* Christina
* Carmen
* Adam
