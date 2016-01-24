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
* Append python directory (e.g. `C:\WinPython-64bit-3.4.3.5\python-3.4.3.amd64`) to PATH
* Unzip the PyBehaviour download into `C:/PyBehaviour`
* Make shortcut to `GUI/execute.bat`, change icon to cheese, move to Desktop or taskbar
* Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
* place TaskScheduler and ElapsedMillis folders in `Arduino/libraries`
* Upload PyBehaviour.ino to arduino

## Known issues
* INTERRUPT 2 (pin 21) NOT COMPATIBLE

## Features to add
* Auto transition
* Adpatation phase, no stimuli, all responses are rewarded
* ...

## Tested compatibility
* Windows 7, 8.1 (64-bit)
* Mac OS X 10.10

## Contributors / beta testers
* Henry
* Oliver
* Christina
* Carmen
