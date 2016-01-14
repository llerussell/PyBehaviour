# !/usr/bin/env python

'''
PyBehaviour
Copyright (c) 2015 Lloyd Russell
'''

import sys
import serial
# from serial.tools import list_ports
import matplotlib
matplotlib.use('Qt5Agg', force=True)
from matplotlib.backends.backend_qt5agg import (FigureCanvasQTAgg as FigureCanvas)
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
from matplotlib import patches
import seaborn
import numpy as np
import scipy.io as sio
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QTimer
from PyQt5.QtWidgets import (QComboBox, QCheckBox, QLineEdit, QSpinBox,
                             QDoubleSpinBox, QFileDialog, QApplication,
                             QDesktopWidget, QMainWindow, QMessageBox,
                             QStyleFactory)
from PyQt5.QtGui import QIcon
from GUI.GUI import Ui_MainWindow
import json
import os
import time
import glob
import warnings
from scipy import stats

warnings.filterwarnings('ignore')


def serial_ports():
    '''
    Lists serial ports.
    Currently non-functional because testing connection resets arduino, which triggers things as pins go high.

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    '''
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(20)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal '/dev/tty'
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        # try:
            # s = serial.Serial(port)
            # s.close()
        result.append(port)
        # except (OSError, serial.SerialException):
            # pass
    return result


# find available serial ports
available_ports = serial_ports()
available_devices = []
for i in range(len(available_ports)):
    available_devices.append(available_ports[i][:])

# setup directories/config files
results_directory = 'Results'
if not os.path.exists(results_directory):
    os.makedirs(results_directory)

config_directory = 'Configs'
if not os.path.exists(config_directory):
    os.makedirs(config_directory)
available_configs = []
for file in os.listdir(config_directory):
    if file.endswith('.cfg'):
        available_configs.append(os.path.splitext(file)[0])

# initialise results and other directories
arduino = {}
arduino['connected'] = False

defaults = {}
trialconfig = {}
p = {}

trials = {}
trials['results'] = []
trials['correctTally'] = 0
running_score = []
trial_log = []
performance_record = []

# start random number seed
np.random.seed()


class TrialRunner(QObject):
    '''
    The background worker. Runs the trials
    '''
    def __init__(self):
        super(TrialRunner, self).__init__()
        self._sessionRunning = False
        self._paused = False

    def startSession(self):
        self._sessionRunning = True
        trial_num = 0
        stim_list = []

        withold_list = []

        if arduino['connected'] is False:
            self.connectArduino()
            #arduino['connected'] = True  # 20150328

        if arduino['connected']:
            # the main seesion loop
            while self._sessionRunning:
                while self._paused:
                    time.sleep(0.1)

                self.runTrial(trial_num, stim_list, trials, withold_list)
                time.sleep(0.1)

                trial_num += 1
                if p['sessionDurationMode'] == 'Trials':
                    if trial_num == p['sessionDuration']:
                        self.stop()
        else:
            self.stop()

    def connectArduino(self):
        self.comm_feed_signal.emit('Connecting Arduino...', '')
        arduino['device'] = serial.Serial(p['device'], p['baudRate'])  # baud should be 19200
        arduino['device'].timeout = 1
        connect_attempts = 3
        current_attempt = 1
        while arduino['connected'] is False and current_attempt <= connect_attempts:
            temp_read = arduino['device'].readline().strip().decode('utf-8')
            self.comm_feed_signal.emit(temp_read, 'arduino')
            if temp_read == '{READY}':
                arduino['connected'] = True
                self.arduino_connected_signal.emit()
            current_attempt += 1
            if current_attempt > connect_attempts:
                self.comm_feed_signal.emit('******** FAILED TO CONNECT ********', '')


    def runTrial(self, trial_num, stim_list, trials, withold_list):
        self.setupResultsDict(trial_num)  # initialise the results for current trial
        self.trial_start_signal.emit(trial_num)  # let GUI know trial number etc
        self.transmitConfig(trial_num, stim_list, withold_list)  # construct and transmit trial instruction to arduino
        self.receiveData(trial_num)  # will sit in this function until it receives the {DONE} command
        self.trial_end_signal.emit(trial_num)  # updates plots, save results

    def setupResultsDict(self, trial_num):
        trials['results'].append({})
        # print(trial_num)
        trials['results'][trial_num]['response1'] = []
        trials['results'][trial_num]['response2'] = []
        trials['results'][trial_num]['response3'] = []
        trials['results'][trial_num]['trialstart'] = []
        trials['results'][trial_num]['withold_req'] = []
        trials['results'][trial_num]['withold_act'] = []
        trials['results'][trial_num]['stim_type'] = []
        trials['results'][trial_num]['response_required'] = []
        trials['results'][trial_num]['reward_channel'] = []
        trials['results'][trial_num]['p'] = p

    def transmitConfig(self, trial_num, stim_list, withold_list):
        '''
        Configure the current trial p.
        Construct a string listing all the various configuration p.
        Send this string to the arduino.
        '''
        # stim type to be delivered

        # elif p['stimOrder'] == 'Repeating':
        #     if trials['correctTally'] == p['stimOrderGroupSize']:
        #         this_stim_idx = prev_stim_idx + 1 if prev_stim_idx < num_stims-1 else 0
        #     else:
        #         this_stim_idx = prev_stim_idx
        # if p['repeatIfIncorrect'] and running_score[-1] == 0:
        #     this_stim_idx = prev_stim_idx

        this_stim_idx = int(p['trialOrder'][trial_num])
        this_var = int(p['trialVariations'][trial_num])

        this_stim = p['stimChannels'][this_stim_idx]

        stim_list.append(this_stim)
        trials['results'][trial_num]['stim_type'] = this_stim

        # response required
        resp_req = p['respRequired'][this_stim_idx]
        trials['results'][trial_num]['response_required'] = resp_req

        # reward channel
        reward_chan = p['rewardedChannels'][this_stim_idx]
        trials['results'][trial_num]['reward_channel'] = reward_chan

        # generate random withold requirement
        if p['witholdBeforeStim']:
            withold_min = p['witholdBeforeStimMin']
            withold_max = p['witholdBeforeStimMax']
            withold_req = np.random.uniform(withold_min, withold_max)
        else:
            withold_req = 0
        trials['results'][trial_num]['withold_req'] = withold_req

        # construct arduino config string
        # format = <KEY:value;KEY:value;>
        config_string = '<' + \
            'STIM_CHAN:' + \
            str(this_stim) + ';' \
            'STIM_VAR:' + \
            str(this_var) + ';' \
            'RESP_REQ:' + \
            str(resp_req) + ';' \
            'REWARD_CHAN:' +  \
            str(reward_chan) + ';' \
            'TRIAL_CUE:' + \
            str(int(p['cueTrial'])) + ';' \
            'STIM_CUE:' + \
            str(int(p['cueStim'])) + ';' \
            'RESP_CUE:' + \
            str(int(p['cueResponse'])) + ';' \
            'RESP_CUE_START:' + \
            str(int(p['responseCueStart']*1000)) + ';' \
            'WITHOLD:' + \
            str(int(p['witholdBeforeStim'])) + ';' \
            'WITHOLD_REQ:' + \
            str(int(withold_req*1000)) + ';' \
            'STIM_START:' + \
            str(int(p['stimStart']*1000)) + ';' \
            'STIM_STOP:' + \
            str(int(p['stimStop']*1000)) + ';' \
            'RESP_START:' + \
            str(int(p['responseStart']*1000)) + ';' \
            'RESP_STOP:' + \
            str(int(p['responseStop']*1000)) + ';' \
            'TRIAL_DURATION:' + \
            str(int(p['trialDuration']*1000)) + ';' \
            'AUTO_REWARD:' + \
            str(int(p['autoReward'])) + ';' \
            'AUTO_REWARD_START:' + \
            str(int(p['autoRewardStart']*1000)) + ';' \
            'PUNISH:' + \
            str(int(p['punish'])) + ';' \
            'PUNISH_LENGTH:' + \
            str(int(p['punishLength']*1000)) + ';' \
            'REWARD_REMOVAL:' + \
            str(int(p['rewardRemoval'])) + ';' \
            'REWARD_REMOVAL_DELAY:' + \
            str(int(p['rewardRemovalDelay']*1000)) + ';' \
            '>'

        # write config string to arduino
        # arduino_ready = 0
        # while ~arduino_ready:
        write_string = '@?'
        self.comm_feed_signal.emit(write_string, 'pc')
        arduino['device'].write(write_string.encode('utf-8'))
        temp_read = arduino['device'].readline().strip().decode('utf-8')
        self.comm_feed_signal.emit(temp_read, 'arduino')
        if temp_read == '{!}':
            # arduino_ready = 1
            write_string = config_string
            self.comm_feed_signal.emit(write_string, 'pc')
            arduino['device'].write(write_string.encode('utf-8'))
            temp_read = arduino['device'].readline().strip().decode('utf-8')
            self.comm_feed_signal.emit(temp_read, 'arduino')

    # signals allow communication between the TrialRunner thread and GUI thread. i.e. send data to main GUI thread where it can be displayed and saved. I don't know why they are here outside of any function...
    response_signal = pyqtSignal(int, int, float, str, name='responseSignal')
    trial_start_signal = pyqtSignal(int, name='trialStartSignal')
    trial_end_signal = pyqtSignal(int, name='trialEndSignal')
    session_end_signal = pyqtSignal(name='sessionEndGUISignal')
    arduino_connected_signal = pyqtSignal(name='arduinoConnectedSignal')
    comm_feed_signal = pyqtSignal(str, str, name='commFeedSignal')

    def receiveData(self, trial_num):
        '''
        The worker thread will sit in a while loop processing incoming communication from the arduino, appending data to the results, unitl the arduino says it has finished the trial.
        '''
        trials['results'][trial_num]['trialstart'] = time.strftime('%Y%m%d_%H%M%S')
        trialRunning = True
        state = 'PRETRIAL'
        # self.response_signal.connect(self.GUI.updatePreTrialRasterPlot)
        while trialRunning:
            # QCoreApplication.processEvents()
            temp_read = arduino['device'].readline().strip().decode('utf-8')

            if temp_read:
                self.comm_feed_signal.emit(temp_read, 'arduino')
                if temp_read[0] == '<' and temp_read[-1] == '>':  # whole data packet received
                    temp_read = temp_read[1:-1]  # only process everything between < and >
                    data = temp_read.split('|')
                    for idx, val in enumerate(data):
                        if val:  # in val is not just
                            ID, val = val.split(':')
                            val = int(val) / 1000
                            if ID == '*':
                                state = 'INTRIAL'
                                # self.response_signal.disconnect()
                                # self.response_signal.connect(GUI.updateRasterPlotData)
                                trials['results'][trial_num]['response1'][:] = [x - val for x in trials['results'][trial_num]['response1']]
                                trials['results'][trial_num]['response2'][:] = [x - val for x in trials['results'][trial_num]['response2']]
                                trials['results'][trial_num]['response3'][:] = [x - val for x in trials['results'][trial_num]['response3']]
                                trials['results'][trial_num]['withold_act'] = val
                            elif ID == '1':
                                trials['results'][trial_num]['response1'].append(val)
                                self.response_signal.emit(int(ID)-1, trial_num, val, state)
                            elif ID == '2':
                                trials['results'][trial_num]['response2'].append(val)
                                self.response_signal.emit(int(ID)-1, trial_num, val, state)
                            elif ID == '3':
                                trials['results'][trial_num]['response3'].append(val)
                                self.response_signal.emit(int(ID)-1, trial_num, val, state)

                elif temp_read[0] == '{' and temp_read[-1] == '}':  # whole trial outcome packet received
                    temp_read = temp_read[1:-1]  # only process everything between { and }
                    if temp_read == 'DONE':
                        trialRunning = False
                        running_score.append((trials['results'][trial_num]['correct']))
                        if trials['results'][trial_num]['correct']:
                            trials['correctTally'] = trials['correctTally'] + 1
                        else:
                            trials['correctTally'] = 0

                    else:
                        data = temp_read.split('|')
                        for idx, val in enumerate(data):
                            if val:  # in val is not just
                                key, val = val.split(':')
                                val = int(val)
                                trials['results'][trial_num][key] = val

    def stop(self):
        self._sessionRunning = False
        self.session_end_signal.emit()


class MainWindow(QMainWindow, Ui_MainWindow):
    '''
    The GUI window
    '''
    def __init__(self):
        QMainWindow.__init__(self)

        self._ready = False
        self.setupUi(self)

        # create the worker thread (run trials in the background)
        self.trialThread = QThread()
        self.trialRunner = TrialRunner()
        self.trialRunner.moveToThread(self.trialThread)
        self.trialThread.started.connect(self.trialRunner.startSession)

        # place figure widgets
        self.addFigures()

        # signal/slot connections
        self.setConnects()

        # configure existing widgets programmatically
        self.device_ComboBox.addItems(available_devices)
        if available_configs != []:
            self.loadPreset_ComboBox.addItems(available_configs)
            self.loadPreset_ComboBox.removeItem(0)
            self.autoTransitionTo_ComboBox.addItems(available_configs)
            self.autoTransitionTo_ComboBox.removeItem(0)

        self._ready = True

        # open the config file and populate GUI with values
        self.GUIRestore()


    def sessionTimerUpdate(self):
        elapsed_time = round(time.time() - p['sessionStartTime'])
        m, s = divmod(elapsed_time, 60)
        h, m = divmod(m, 60)
        self.sessionTimer_label.setText('%d:%02d:%02d' % (h, m, s))

    def begin(self):
        # reset everything
        self.tabWidget.setCurrentIndex(1)
        self.reset()

        p['sessionStartTime'] = time.time()
        p['sessionStartTimeString'] = time.strftime('%Y%m%d_%H%M%S')
        p['sessionID'] = p['subjectID'] + '_' + p['sessionStartTimeString']
        self.setWindowTitle('PyBehaviour | ' + p['sessionID'])
        #self.sessionFeed_textEdit.append('Started')
        self.sessionTimer.start(100)  # start the QTimer, executes every 100ms

        self.trialThread.start()

    def reset(self):
        running_score = 0
        if self.trialRunner._sessionRunning:
            self.trialRunner.stop()
            self.trialThread.quit()
        plots = [self.preTrialRaster_resp1, self.preTrialRaster_resp2,
                 self.preTrialRaster_resp3, self.raster_resp1,
                 self.raster_resp2, self.raster_resp3, self.runningScorePlot]
        for plot in plots:
            plot.set_data([[],[]])
        self.updatePlotLayouts()
        self.preTrialRasterFigCanvas.draw()

        trials = {}
        trials['results'] = []
        running_score = []

    def pause(self):
        self.trialRunner._paused = not self.trialRunner._paused
        if self.trialRunner._paused:
            self.sessionPause_pushButton.setText('Resume')
        else:
            self.sessionPause_pushButton.setText('Pause')

    def quit(self):
        if self.trialRunner._sessionRunning:
            self.trialRunner.stop()
            self.trialThread.quit()

    def updatePlotLayouts(self):
        # response raster
        pre_stim_plot = 1
        self.rasterFigAx.set_xlim(-pre_stim_plot, p['trialDuration'])
        self.rasterFigAx.set_ylim(0, p['sessionDuration'])
        self.raster_respwin.set_x(p['responseStart'])
        self.raster_respwin.set_width(p['responseWindow'])
        self.raster_respwin.set_height(p['sessionDuration'])
        self.raster_stimline.set_ydata([0, p['sessionDuration']])
        self.raster_stimlength.set_x(p['stimStart'])
        self.raster_stimlength.set_width(p['stimLength'])
        self.raster_stimlength.set_height(p['sessionDuration'])
        self.rasterFigCanvas.draw()

        # performance line
        self.performanceFigAx.set_ylim(0, 1)
        self.performanceFigAx.set_xlim(0, p['sessionDuration'])
        self.performanceFigCanvas.draw()

        # performance blocks
        num_stims = len(p['stimChannels'])
        self.rasterFigPerfAx.set_ylim(-0.5, p['sessionDuration']-0.5)
        self.rasterFigPerfAx.set_xlim(-0.5, num_stims-0.5)
        self.performanceFigPerfAx.set_ylim(-0.5, num_stims-0.5)
        self.performanceFigPerfAx.set_xlim(-0.5, p['sessionDuration']-0.5)

    def updateRasterPlotData(self, ID, trial_num, new_data, state):
        if state == 'INTRIAL':
            plots = [self.raster_resp1, self.raster_resp2, self.raster_resp3]
            plot_series = plots[ID]
            old_xdata = plot_series.get_xdata()
            old_ydata = plot_series.get_ydata()
            resp3_xdata = np.append(old_xdata, new_data)
            resp3_ydata = np.append(old_ydata, trial_num)
            plot_series.set_xdata(resp3_xdata)
            plot_series.set_ydata(resp3_ydata)
            # self.rasterFigAx.relim()
            # self.rasterFigAx.autoscale_view()
            self.rasterFigCanvas.draw()
        elif state == 'PRETRIAL':
            plots = [self.preTrialRaster_resp1, self.preTrialRaster_resp2,
                     self.preTrialRaster_resp3]
            plot_series = plots[ID]
            old_xdata = plot_series.get_xdata()
            old_ydata = plot_series.get_ydata()
            resp3_xdata = np.append(old_xdata, new_data)
            resp3_ydata = np.append(old_ydata, 0)
            plot_series.set_xdata(resp3_xdata)
            plot_series.set_ydata(resp3_ydata)
            self.preTrialRasterFigAx.relim()
            self.preTrialRasterFigAx.autoscale_view()
            self.preTrialRasterFigCanvas.draw()

    def updateResultBlockPlots(self, trial_num):
        num_stims = len(p['stimChannels'])
        num_trials = p['sessionDuration']
        performance_record = np.ones([num_trials, num_stims, 3])
        for t in range(trial_num+1):
            if trials['results'][t]['correct']:
                colour = [0, .8, .95]
            elif not trials['results'][t]['miss']:  # means incorrect
                colour = [.95, 0, .05]
            else:  # means miss
                colour = [.66, .66, .66]
            performance_record[t, p['trialOrder'][t]] = colour

        self.rasterFigPerfAx.imshow(performance_record, interpolation='nearest', aspect='auto')
        self.performanceFigPerfAx.imshow(np.rot90(performance_record), interpolation='nearest', aspect='auto')

    def updatePerformancePlot(self, trial_num):
        plot_series = self.runningScorePlot
        old_ydata = plot_series.get_ydata()
        new_ydata = np.append(old_ydata, np.mean(running_score))
        plot_series.set_ydata(new_ydata)
        plot_series.set_xdata(range(len(new_ydata)))

    def setConnects(self):
        # add connects for control button clicks
        self.setDefaults_Button.clicked.connect(self.GUISave)
        self.saveAsPreset_Button.clicked.connect(self.saveAsPreset)
        self.testPin_Button.clicked.connect(self.testPin)
        self.begin_Button.clicked.connect(self.begin)
        self.sessionPause_pushButton.clicked.connect(self.pause)
        self.sessionAbort_pushButton.clicked.connect(self.quit)

        self.loadPreset_ComboBox.currentIndexChanged.connect(self.loadPreset)

        self.sessionTimer = QTimer(self)
        self.sessionTimer.timeout.connect(self.sessionTimerUpdate)
        self.trialRunner.arduino_connected_signal.connect(self.arduinoConnected)
        self.trialRunner.response_signal.connect(self.updateRasterPlotData)
        self.trialRunner.trial_start_signal.connect(self.trialStartGUI)
        self.trialRunner.trial_end_signal.connect(self.trialStopGUI)
        self.trialRunner.session_end_signal.connect(self.sessionEndGUI)
        self.trialRunner.comm_feed_signal.connect(self.updateCommFeed)

        # auto add connects to update p and trial config plot whenever anything changes
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        groups = (self.session_GroupBox, self.arduino_GroupBox,
                  self.trial_GroupBox)
        for group in groups:
            for obj in group.findChildren(widgets):
                if isinstance(obj, QComboBox):
                    obj.currentIndexChanged.connect(self.getValues)
                if isinstance(obj, QCheckBox):
                    obj.stateChanged.connect(self.getValues)
                if isinstance(obj, QLineEdit):
                    obj.textChanged.connect(self.getValues)
                if isinstance(obj, QSpinBox):
                    obj.valueChanged.connect(self.getValues)
                if isinstance(obj, QDoubleSpinBox):
                    obj.valueChanged.connect(self.getValues)

    def testPin(self):
        if arduino['connected'] is False:
            self.trialRunner.connectArduino()
        if arduino['connected'] is True:
            write_string = '@!' + str(p['testPinNum']) + ';' + str(int(p['testPinDuration'])) + ';'
            self.updateCommFeed(write_string, 'pc')
            arduino['device'].write(write_string.encode('utf-8'))
            temp_read = arduino['device'].readline().strip().decode('utf-8')
            self.updateCommFeed(temp_read, 'arduino')

    def arduinoConnected(self):
        self.arduinoConnectedText_Label.setText('Connected')
        self.arduinoConnectedText_Label.setStyleSheet('color:rgb(0, 188, 152);')

    def makeTrialOrder(self):
        if not self.trialRunner._sessionRunning and self._ready:
            num_stims = len(p['stimChannels'])
            if p['stimOrder'] == 'Random':

                # count proportions for normalisation
                total_prop = sum(p['proportions'])

                # build blocks of trial types to allow absolute proportions
                trial_blocks = []
                for i, chan in enumerate(p['stimChannels']):
                    trial_blocks.append(chan * np.ones([np.floor(p['proportions'][i] / total_prop * p['sessionDuration'])]))

                # stack all trial type blocks and shuffle
                p['trialOrder'] = np.hstack(trial_blocks)
                np.random.shuffle(p['trialOrder'])

                # if needed add random trials to end to get desired number
                if len(p['trialOrder']) < p['sessionDuration']:
                    diff = p['sessionDuration'] - len(p['trialOrder'])
                    xk = p['stimChannels']
                    pk = p['proportions']
                    custm = stats.rv_discrete(name='custm', values=(xk, pk))
                    extra_trials = custm.rvs(size=diff)
                    p['trialOrder'] = np.append(p['trialOrder'], extra_trials)

                # trial variations
                p['trialVariations'] = np.zeros(len(p['trialOrder']))
                for i in range(len(p['trialOrder'])):
                    this_trial = p['trialOrder'][i]
                    this_idx = np.where(p['stimChannels'] == this_trial)[0]
                    p['trialVariations'][i] = np.random.randint(p['variations'][this_idx])

                # do the test for consecutive trials (p['stimOrderGroupSize'])
                # passed = False
                # while not passed:
                #   for i in range(num_trials):
                #         this_stim = x
                #         prev_stim = asdf
                #         same_as_prev =
                #         if same_as_prev:
                #             passed = True

            elif p['stimOrder'] == 'Interleaved':
                p['trialOrder'] = np.zeros([p['sessionDuration']])
                stim_idx = 0
                consec_tally = 0

                for trial in range(p['sessionDuration']):
                    p['trialOrder'][trial] = p['stimChannels'][stim_idx]
                    consec_tally += 1
                    if consec_tally >= p['stimOrderGroupSize']:
                        consec_tally = 0
                        stim_idx += 1
                        if stim_idx >= len(p['stimChannels']):
                            stim_idx = 0

            elif p['stimOrder'] == 'External file':
                p['trialOrder'] = np.zeros([p['sessionDuration']])

            # first stim control
            if p['controlFirstStim']:
                if p['controlFirstStimStim']:
                    p['trialOrder'][0:p['firstXStims']] = int(p['firstStim'])
                if p['controlFirstStimVariation']:
                    p['trialVariations'][0:p['firstXStims']] = int(p['firstStimVariation'])

            # p['trialOrder'] = [p['trialOrder']]
            self.trialOrderAx.imshow([p['trialOrder']], interpolation='nearest', cmap='rainbow', vmin=0, vmax=7, aspect='auto')
            self.trialOrderAx2.imshow([p['trialVariations']], interpolation='nearest', cmap='gray', aspect='auto')
            self.trialOrderCanvas.draw()

    def getValues(self):
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        groups = (self.session_GroupBox, self.arduino_GroupBox,
                  self.trial_GroupBox)
        for group in groups:
            for obj in group.findChildren(widgets):
                fullname = str(obj.objectName())
                trimmed_name = fullname.split('_')[0]
                if isinstance(obj, QComboBox):
                    p[trimmed_name] = str(obj.currentText())
                if isinstance(obj, QCheckBox):
                    p[trimmed_name] = bool(obj.isChecked())
                if isinstance(obj, QLineEdit):
                    if 'spinbox' not in fullname:
                        p[trimmed_name] = str(obj.text())
                if isinstance(obj, QSpinBox):
                    p[trimmed_name] = int(obj.value())
                if isinstance(obj, QDoubleSpinBox):
                    p[trimmed_name] = float(obj.value())

        # responseChannels = [p['response1'], p['response2'], p['response3']]
        # rewardChannels = [p['reward1'], p['reward2']]

        stimChannels = [p['stim1'], p['stim2'], p['stim3'], p['stim4'],
                        p['stim5'], p['stim6'], p['stim7'], p['stim8']]

        respRequired = [p['respReq1'], p['respReq2'], p['respReq3'],
                        p['respReq4'], p['respReq5'], p['respReq6'],
                        p['respReq7'], p['respReq8']]

        rewardedChannels = [p['rewardedChan1'], p['rewardedChan2'],
                            p['rewardedChan3'], p['rewardedChan4'],
                            p['rewardedChan5'], p['rewardedChan6'],
                            p['rewardedChan7'], p['rewardedChan8']]

        proportions = [p['proportion1'], p['proportion2'], p['proportion3'],
                       p['proportion4'], p['proportion5'], p['proportion6'],
                       p['proportion7'], p['proportion8']]

        variations = [p['variations1'], p['variations2'], p['variations3'],
                      p['variations4'], p['variations5'], p['variations6'],
                      p['variations7'], p['variations8']]

        # p['responseChannels'] = [idx+1 for idx, val in enumerate(responseChannels) if val]
        # p['rewardChannels'] = [idx+1 for idx, val in enumerate(rewardChannels) if val]

        p['stimChannels'] = [idx for idx, val in enumerate(stimChannels) if val]
        p['respRequired'] = [respRequired[idx] for idx in p['stimChannels']]
        p['rewardedChannels'] = [rewardedChannels[idx] for idx in p['stimChannels']]
        p['proportions'] = [proportions[idx] for idx in p['stimChannels']]
        p['variations'] = [variations[idx] for idx in p['stimChannels']]

        p['totalDuration'] = p['witholdBeforeStimMax'] + p['cueToStimDelay'] + \
                             p['postStimDelay'] + p['responseWindow'] + p['endOfTrialDelay']

        p['trialDuration'] = p['totalDuration'] - p['witholdBeforeStimMax']

        p['trialCueStart'] = 0
        p['stimCueStart'] = 0  # time resets to zero after withold
        p['stimStart'] = p['cueToStimDelay']
        p['stimStop'] = p['stimStart'] + p['stimLength']
        p['responseCueStart'] = p['cueToStimDelay'] + p['postStimDelay']
        p['responseStart'] = p['cueToStimDelay'] + p['postStimDelay']
        p['responseStop'] = p['responseStart'] + p['responseWindow']
        p['autoRewardStart'] = p['cueToStimDelay'] + p['autoRewardDelay']

        self.makeTrialOrder()
        self.updateTrialConfigPlot()
        self.updatePlotLayouts()


    def updateTrialConfigPlot(self):
        self.duration_line.set_width(p['totalDuration'])
        self.trial_cue_rectangle.set_width(p['cueTrial'] * 0.1)

        self.stim_cue_rectangle.set_width(p['cueStim'] * 0.1)
        self.stim_cue_rectangle.set_x(p['witholdBeforeStimMax'])

        self.response_cue_rectangle.set_width(p['cueResponse'] * 0.1)
        self.response_cue_rectangle.set_x(p['witholdBeforeStimMax'] + p['responseCueStart'])

        self.stim_rectangle.set_x(p['witholdBeforeStimMax'] + p['stimStart'])
        self.stim_rectangle.set_width(p['stimLength'])

        self.resp_rectangle.set_x(p['witholdBeforeStimMax'] + p['responseStart'])
        self.resp_rectangle.set_width(p['responseWindow'])

        self.reward_rectangle.set_width(p['autoReward'] * 0.1)
        self.reward_rectangle.set_x(p['witholdBeforeStimMax'] + p['autoRewardStart'])

        self.trialConfigAx.set_xlim([0, p['totalDuration']])
        self.trialConfigCanvas.draw()


    def GUISave(self):
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        groups = (self.session_GroupBox, self.arduino_GroupBox)
        for group in groups:
            for obj in group.findChildren(widgets):
                name = str(obj.objectName())
                if isinstance(obj, QComboBox):
                    defaults[name] = str(obj.currentText())
                if isinstance(obj, QCheckBox):
                    defaults[name] = bool(obj.isChecked())
                if isinstance(obj, QLineEdit):
                    if 'spinbox' not in name:
                        defaults[name] = str(obj.text())
                if isinstance(obj, QSpinBox):
                    defaults[name] = int(obj.value())
                if isinstance(obj, QDoubleSpinBox):
                    defaults[name] = float(obj.value())
        json.dump(defaults, open(os.path.join('GUI', 'GUIdefaults.cfg'), 'w'), sort_keys=True, indent=4)

    def GUIRestore(self):
        if os.path.isfile(os.path.join('GUI', 'GUIdefaults.cfg')):
            defaults = json.load(open(os.path.join('GUI', 'GUIdefaults.cfg'), 'r'))
            widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
            groups = (self.session_GroupBox, self.arduino_GroupBox)
            for group in groups:
                for obj in group.findChildren(widgets):
                    name = str(obj.objectName())
                    try:
                        if isinstance(obj, QComboBox):
                            value = defaults[name]
                            if value == '':
                                continue
                            index = obj.findText(value)  # get idx for string in combo
                            if index == -1:  # add to list if not found
                                continue
                            else:
                                obj.setCurrentIndex(index)  # preselect a combobox value
                        if isinstance(obj, QLineEdit):
                            if 'spinbox' not in name:
                                value = defaults[name]
                                obj.setText(value)  # restore lineEditFile
                        if isinstance(obj, QCheckBox):
                            value = defaults[name]
                            if value is not None:
                                obj.setChecked(value)  # restore checkbox
                        if isinstance(obj, QSpinBox):
                            value = defaults[name]
                            obj.setValue(value)  # restore lineEditFile
                        if isinstance(obj, QDoubleSpinBox):
                            value = defaults[name]
                            obj.setValue(value)  # restore lineEditFile
                    except:
                        continue

    def addFigures(self):
        # trial order ribbon
        self.trialOrderFig = Figure()
        self.trialOrderFig.set_facecolor('white')
        self.trialOrderCanvas = FigureCanvas(self.trialOrderFig)  # a canvas holds a figure
        self.trialOrderCanvas.setFixedHeight(15)
        self.trialOrderAx = self.trialOrderFig.add_axes([0, 0.25, 1, 0.72])
        self.trialOrderAx2 = self.trialOrderFig.add_axes([0, 0, 1, 0.25])
        self.trialOrderAx.axis('off')
        self.trialOrderAx.hold(True)
        self.trialOrderAx2.axis('off')
        self.trialOrderAx2.hold(True)
        self.trialOrder_verticalLayout.addWidget(self.trialOrderCanvas)


        # trial structure config
        self.trialConfigFig = Figure()
        self.trialConfigFig.set_facecolor('white')
        self.trialConfigCanvas = FigureCanvas(self.trialConfigFig)  # a canvas holds a figure
        self.trialConfigCanvas.setFixedHeight(50)
        self.trialConfigAx = self.trialConfigFig.add_axes([0, 0, 1, 1])
        self.trialConfigAx.axis('off')
        self.trialConfigAx.hold(True)

        # make the shapes
        self.duration_line = patches.Rectangle((0, 0), 0, 0.05, fc='k', ec='none')
        self.trial_cue_rectangle = patches.Rectangle((0, 0), 0, 1, fc=[.95, 0, .5], ec='none')
        self.stim_cue_rectangle = patches.Rectangle((0, 0), 0, 1, fc=[.95, 0, .5], ec='none')
        self.response_cue_rectangle = patches.Rectangle((0, 0), 0, 1, fc=[.95, 0, .5], ec='none')
        self.stim_rectangle = patches.Rectangle((0, 0), 0, 1, fc='k', ec='none')
        self.resp_rectangle = patches.Rectangle((0, 0), 0, 0.9, fc=[.66, .66, .66], ec='none')
        self.reward_rectangle = patches.Rectangle((0, 0), 0, 1, fc=[0, .8, .95], ec='none')

        # add the shapes to the figure
        self.trialConfigAx.add_patch(self.stim_rectangle)
        self.trialConfigAx.add_patch(self.resp_rectangle)
        self.trialConfigAx.add_patch(self.reward_rectangle)
        self.trialConfigAx.add_patch(self.trial_cue_rectangle)
        self.trialConfigAx.add_patch(self.stim_cue_rectangle)
        self.trialConfigAx.add_patch(self.response_cue_rectangle)
        self.trialConfigAx.add_patch(self.duration_line)

        # add to GUI
        self.setupTabVerticalLayout.addWidget(self.trialConfigCanvas)


        # pretrial response/withold plot
        self.preTrialRasterFig = Figure()
        self.preTrialRasterFigCanvas = FigureCanvas(self.preTrialRasterFig)  # a canvas holds a figure
        self.preTrialResponseVerticalLayout.addWidget(self.preTrialRasterFigCanvas)
        self.preTrialRasterFigAx = self.preTrialRasterFig.add_axes([0.1, 0.5, .8, 0.1])
        self.preTrialRasterFigAx.hold(True)
        self.preTrialRasterFigAx.axis('on')
        self.preTrialRasterFigAx.get_yaxis().set_visible(False)
        self.preTrialRasterFigAx.set_title('Pre-trial', loc='left')
        self.preTrialRaster_resp1, = self.preTrialRasterFigAx.plot([], [], 'ro', mec='none', aa=True, clip_on=False)  # the comma is important!
        self.preTrialRaster_resp2, = self.preTrialRasterFigAx.plot([], [], 'bo', mec='none', aa=True, clip_on=False)
        self.preTrialRaster_resp3, = self.preTrialRasterFigAx.plot([], [], 'go', mec='none', aa=True, clip_on=False)
        self.preTrialRasterFigCanvas.setFixedHeight(50)
        self.preTrialRasterFig.set_facecolor('white')

        # main response raster plot
        self.rasterFig = Figure()
        self.rasterFigCanvas = FigureCanvas(self.rasterFig)  # a canvas holds a figure
        self.resultsFigsHorizontalLayout.addWidget(self.rasterFigCanvas)
        self.rasterFigAx = self.rasterFig.add_axes([0.2, 0.15, 0.67, 0.75])
        self.rasterFig.set_facecolor('white')
        self.rasterFigAx.set_title('Responses', loc='left')
        self.rasterFigAx.hold(True)

        self.raster_resp1, = self.rasterFigAx.plot([], [], 'ks', markersize=3, mec='none', aa=True, clip_on=False)  # comma is important!
        self.raster_resp2, = self.rasterFigAx.plot([], [], 'o', mec='none', aa=True, clip_on=False)
        self.raster_resp3, = self.rasterFigAx.plot([], [], 'o', mec='none', aa=True, clip_on=False)
        self.raster_reward, = self.rasterFigAx.plot([], [], 'o', clip_on=False)
        self.raster_punish, = self.rasterFigAx.plot([], [], 'o', clip_on=False)

        self.raster_respwin = patches.Rectangle((0,0), 0, 0, fc=(0, 0, 0, 0.1), ec='none')
        self.raster_stimlength = patches.Rectangle((0, 0), 0, 0, fc=(0, 0, 0, 0.1), ec='none')
        self.rasterFigAx.add_patch(self.raster_respwin)
        self.rasterFigAx.add_patch(self.raster_stimlength)

        self.raster_stimline, = self.rasterFigAx.plot([0, 0], [0, 0], 'k-', linewidth=2)
        self.rasterFigAx.set_xlabel('Time (s)')
        self.rasterFigAx.set_ylabel('Trial')

        self.rasterFigPerfAx = self.rasterFig.add_axes([0.9, 0.15, 0.05, 0.75])
        self.rasterFigPerfAx.axis('off')

        # self.rasterFigCanvas.draw()


        # perfomance blocks


        # results plot
        self.performanceFig = Figure()
        self.performanceFigCanvas = FigureCanvas(self.performanceFig)  # a canvas holds a figure
        self.performanceFigAx = self.performanceFig.add_axes([0.2, 0.15, 0.75, 0.7])
        self.resultsFigsHorizontalLayout.addWidget(self.performanceFigCanvas)
        self.runningScorePlot, = self.performanceFigAx.plot([], [], 'k-', linewidth=2, aa=True, clip_on=False)
        self.performanceFigAx.set_xlabel('Trial')
        self.performanceFigAx.set_ylabel('Performance')
        self.performanceFig.set_facecolor('white')

        self.performanceFigPerfAx = self.performanceFig.add_axes([0.2, 0.86, 0.75, 0.04])
        self.performanceFigPerfAx.axis('off')
        self.performanceFigPerfAx.set_title('Performance', loc='left')

    def saveAsPreset(self):
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        for obj in self.trial_GroupBox.findChildren(widgets):
            name = str(obj.objectName())
            if isinstance(obj, QComboBox):
                trialconfig[name] = str(obj.currentText())
            if isinstance(obj, QCheckBox):
                trialconfig[name] = bool(obj.isChecked())
            if isinstance(obj, QLineEdit):
                if 'spinbox' not in name:
                    trialconfig[name] = str(obj.text())
            if isinstance(obj, QSpinBox):
                trialconfig[name] = int(obj.value())
            if isinstance(obj, QDoubleSpinBox):
                trialconfig[name] = float(obj.value())
        filepath = str(QFileDialog.getSaveFileName(self, 'Save as preset...', 'Configs', 'Config file (*.cfg)')[0])
        json.dump(trialconfig, open(filepath, 'w'), sort_keys=True, indent=4)
        filename = os.path.basename(filepath)
        filename = os.path.splitext(filename)[0]
        self.loadPreset_ComboBox.addItems([filename])
        index = self.loadPreset_ComboBox.findText(filename)
        self.loadPreset_ComboBox.setCurrentIndex(index)

    def loadPreset(self):
        filename = str(self.loadPreset_ComboBox.currentText())
        filepath = os.path.join(config_directory, filename + '.cfg')
        trialconfig = json.load(open(filepath, 'r'))
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        for obj in self.trial_GroupBox.findChildren(widgets):
            name = str(obj.objectName())
            try:
                if isinstance(obj, QComboBox):
                    value = trialconfig[name]
                    index = obj.findText(value)  # get the corresponding index for specified string in combobox
                    obj.setCurrentIndex(index)  # preselect a combobox value by index
                if isinstance(obj, QLineEdit):
                    value = trialconfig[name]
                    if 'spinbox' not in name:
                        obj.setText(value)  # restore lineEditFile
                if isinstance(obj, QCheckBox):
                    value = trialconfig[name]
                    if value is not None:
                        obj.setChecked(value)  # restore checkbox
                if isinstance(obj, QSpinBox):
                    value = trialconfig[name]
                    obj.setValue(value)  # restore lineEditFile
                if isinstance(obj, QDoubleSpinBox):
                    value = trialconfig[name]
                    obj.setValue(value)  # restore lineEditFile
            except:
                continue

    def closeEvent(self, event):
        # result = QMessageBox.question(self, 'Confirm Exit',
        #     'Are you sure you want to exit ?',
        #     QMessageBox.Yes | QMessageBox.No)
        # event.ignore()
        result = QMessageBox.Yes

        if result == QMessageBox.Yes:
            self.trialRunner.stop()
            time.sleep(0.1)
            self.trialThread.quit()
            event.accept()

        if arduino['connected']:
            arduino['connected'] = False
            arduino['device'].close()

    def sessionEndGUI(self):
        self.sessionTimer.stop()
        end_time_string = self.sessionTimer_label.text()
        self.sessionTimer_label.setText('<font color=''#ff0022''>' + end_time_string + '</font>')
        self.updateCommFeed('Finished')

    def trialStartGUI(self, trial_num):
        self.trialNum_label.setText(str(trial_num+1))
        self.updateCommFeed('\n')
        self.updateCommFeed('Trial ' + str(trial_num+1), 'trial')
        plots = [self.preTrialRaster_resp1, self.preTrialRaster_resp2,
                 self.preTrialRaster_resp3]
        for plot_series in plots:
            plot_series.set_data([[], []])

    def trialStopGUI(self, trial_num):
        self.updateResultBlockPlots(trial_num)
        self.updatePerformancePlot(trial_num)
        self.performanceFigCanvas.draw()
        self.rasterFigCanvas.draw()
        self.saveResults()

    def updateCommFeed(self, input_string, device=None):
        input_string = input_string.replace('<', '{').replace('>', '}')
        if device == 'pc':
            input_string = '<font color=''#6600ff''>PC:</font>' + '...... ' + input_string
        elif device == 'arduino':
            input_string = '<font color=''#ff0066''>ARDUINO: </font>' + input_string
        elif device == 'trial':
                input_string = '<b>' + input_string + '</b>'
        trial_log.append(input_string)
        print input_string

        # if device == 'pc':
        #     input_string = '<font color=''#330066''>' + input_string + '</font>'
        # elif device == 'arduino':
        #     input_string = '<font color=''#660033''>' + input_string + '</font>'
        # self.sessionFeed_textEdit.setText(input_string)
        self.sessionFeed_textEdit.append(input_string)

    def saveResults(self):
        save_directory = os.path.join(results_directory, p['subjectID'])
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

        save_name = os.path.join(save_directory, p['sessionID'])

        sio.savemat(save_name + '.mat', trials)
        self.updateCommFeed('Saved to ' + p['sessionID'] + '.mat')

        # save figures to pdf
        with PdfPages(save_name + '.pdf') as pdf:
            pdf.savefig(self.rasterFig)
            pdf.savefig(self.performanceFig)

        # save terminal outputs to text file
        with open(save_name + '.txt', 'w') as log:
            log.write('\n'.join(trial_log))


# Main entry to program.  Sets up the main app and create a new window.
def main(argv):
    # create Qt application
    app = QApplication(argv)
    app.setStyle('fusion')

    # create main window
    GUI = MainWindow()

    # centre the window
    screen_res = QDesktopWidget().screenGeometry()
    GUI.move((screen_res.width() / 2) - (GUI.frameSize().width() / 2),
             (screen_res.height() / 2) - (GUI.frameSize().height() / 2))

    # show it and bring to front
    GUI.show()
    GUI.raise_()

    # show the window icon
    if os.path.isfile(os.path.join('GUI', 'icon.ico')):
        GUI.setWindowIcon(QIcon(os.path.join('GUI', 'icon.ico')))

    # start the app
    sys.exit(app.exec_())

if __name__ == '__main__':
    main(sys.argv)
