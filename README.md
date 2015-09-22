# PyBehaviour
Behavioural testing software, Python, arduino

## Disclaimer
In development

## Screenshots
![Imgur](http://i.imgur.com/Fzr6LJq.png)

## Prerequisites
* Python (3.4 recommended)
  * numpy
  * matplotlib
  * scipy
  * seaborn
  * PyQt5
* Arduino (MEGA 2560 recommended)
  * [TaskScheduler](http://playground.arduino.cc/Code/TaskScheduler)
  * [ElapsedMillis](http://playground.arduino.cc/Code/ElapsedMillis)

## Install instructions
Windows 64-bit:
* Install [winpython](http://winpython.github.io) (includes PyQt5)
* Append python directory (e.g. `C:\WinPython-64bit-3.4.3.5\python-3.4.3.amd64`) to PATH
* Clone PyBehaviour into `C:/PyBehaviour`
* Make shortcut to `GUI/execute.bat`, change icon to cheese, move to Desktop or taskbar

## Known issues
* globals
* redundant dictionaries
* needs to be divided into subfunctions
* 'communication' panel has been offloaded to terminal window
* GUI window could be better organised
  * Response channel and reward channel checkboxes are unnecessary
  * ...
* clear trial performance bars at restart session
* pre trial licks should be cleared at start of trial
* display plotting issue: no withold period, stil shows max withold val
* cues not fully implemented

## Features to add
* Specific proportion of stimulus types
* Specify cue channels per stimulus
* Auto transition
* Adpatation phase, no stimuli, all responses are rewarded 
* post-stim, pre-response window responses to abort trial
* auto-transition post stim delay duration
* ...

## Tested compatibility
* Windows 7, 8.1 (64-bit)
* Mac OS X 10.10

## Contributors / beta testers
* Henry
* Oliver
* Christina
* Carmen
