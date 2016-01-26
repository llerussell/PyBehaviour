# !/usr/bin/env python

'''
PyBehaviour
(c) 2015 Lloyd Russell
'''

import warnings
warnings.filterwarnings('ignore')
import matplotlib
matplotlib.use('Qt5Agg', force=True)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
from matplotlib import patches
import seaborn
seaborn.set(rc={
    'axes.axisbelow': True,
    'axes.linewidth': 1,
    'axes.facecolor': [1.0, 1.0, 1.0],
    'axes.edgecolor': [0.9, 0.9, 0.9],
    'grid.color': [0.9, 0.9, 0.9],
    # 'image.composite_image': False
    })
import numpy as np
import scipy.io as sio
from scipy import stats
import json
import os
import time
import sys
import serial
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QTimer
from PyQt5.QtWidgets import (QComboBox, QCheckBox, QLineEdit, QSpinBox,
                             QDoubleSpinBox, QFileDialog, QApplication,
                             QDesktopWidget, QMainWindow, QMessageBox)
from PyQt5.QtGui import QIcon
from GUI.GUI import Ui_MainWindow
import serial_ports


# find available serial ports
available_devices = serial_ports.list_ports()

# setup directories
results_directory = 'Results'
if not os.path.exists(results_directory):
    os.makedirs(results_directory)

trial_sequence_directory = 'Trial sequences'
if not os.path.exists(trial_sequence_directory):
    os.makedirs(trial_sequence_directory)

config_directory = 'Configs'
if not os.path.exists(config_directory):
    os.makedirs(config_directory)
available_configs = []
for file in os.listdir(config_directory):
    if file.endswith('.cfg'):
        available_configs.append(os.path.splitext(file)[0])

# initialise results and other directories
trials = {}
arduino = {}
arduino['connected'] = False
p = {}

# start random number seed
np.random.seed()


class TrialRunner(QObject):
    '''
    Background worker. Runs the trials.
    '''
    def __init__(self):
        super(TrialRunner, self).__init__()
        self._session_running = False
        self._paused = False

    def startSession(self):
        self._session_running = True
        trial_num = 0

        if arduino['connected'] is False:
            self.connectArduino()

        if arduino['connected']:
            # the main session loop
            while self._session_running:
                while self._paused:
                    time.sleep(0.1)

                self.runTrial(trial_num, trials)
                time.sleep(0.1)

                trial_num += 1
                if p['sessionDurationMode'] == 'Trials':
                    if trial_num == p['sessionDuration']:
                        self.stop()
        else:
            self.stop()

    def connectArduino(self):
        self.comm_feed_signal.emit('Connecting Arduino on port ' + p['device'], 'pc')
        arduino['device'] = serial.Serial(p['device'], 19200)
        arduino['device'].timeout = 0
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
                self.comm_feed_signal.emit('*** Failed to connect ***', 'pc')
                arduino['device'].close()

    def disconnectArduino(self):
        if arduino['connected']:
            arduino['connected'] = False
            self.comm_feed_signal.emit('Disconnecting Arduino...', '')
            arduino['device'].close()
            self.arduino_disconnected_signal.emit()

    def runTrial(self, trial_num, trials):
        self.setupResultsDict(trial_num)  # initialise the results for current trial
        self.trial_start_signal.emit(trial_num)  # let GUI know trial number etc
        self.transmitConfig(trial_num)  # construct and transmit trial instruction to arduino
        self.receiveData(trial_num)  # will sit in this function until it receives the {DONE} command
        self.trial_end_signal.emit(trial_num)  # updates plots, save results

    def setupResultsDict(self, trial_num):
        trials['results'].append({})
        trials['results'][trial_num]['responses'] = [[], []]
        trials['results'][trial_num]['trialstart'] = []
        trials['results'][trial_num]['withold_req'] = []
        trials['results'][trial_num]['withold_act'] = []
        trials['results'][trial_num]['stim_type'] = []
        trials['results'][trial_num]['stim_var'] = []
        trials['results'][trial_num]['response_required'] = []
        trials['results'][trial_num]['reward_channel'] = []
        trials['results'][trial_num]['parameters'] = p

    def transmitConfig(self, trial_num):
        '''
        Configure the current trial parameters.
        Construct a string listing all the various configuration parameters.
        Send this string to the arduino.
        '''

        # adjust trial order if repeat-if-incorrect
        if trial_num > 0 and p['repeatIfIncorrect']:
            max_repeats = int(p['maximumRepeatsIfIncorrect'])
            if trial_num >= max_repeats:
                prev_trials = p['trialOrder'][trial_num-max_repeats:trial_num]
            else:
                prev_trials = p['trialOrder'][:trial_num]
            if (trial_num >= max_repeats) and (np.sum(trials['running_score'][-max_repeats:]) == -max_repeats) and (prev_trials.min() == prev_trials.max()):
                    pass  # do not repeat any more if max_repeats of a stim has been exceeded
            elif trials['running_score'][trial_num-1] <= 0:
                # make this stim same as previous
                p['trialOrder'][trial_num] = p['trialOrder'][trial_num-1]
                p['trialVariations'][trial_num] = p['trialVariations'][trial_num-1]

                # shift next trials back
                p['trialOrder'][trial_num+1:-1] = p['trialOrder'][trial_num:-2]
                p['trialVariations'][trial_num+1:-1] = p['trialVariations'][trial_num:-2]


        # get the current stim type
        this_stim = int(p['trialOrder'][trial_num])
        this_stim_idx = p['stimChannels'].index(this_stim)
        this_var = int(p['trialVariations'][trial_num])
        trials['results'][trial_num]['stim_type'] = this_stim
        trials['results'][trial_num]['stim_var'] = this_var

        # get response required
        resp_req = p['respRequired'][this_stim_idx]
        trials['results'][trial_num]['response_required'] = resp_req

        # resonse-cancels-trial (post stim delay)
        post_stim_cancel = int(p['postStimCancel'])

        # get reward, cue and punish channels
        reward_chan = p['rewardChannels'][this_stim_idx]
        trials['results'][trial_num]['reward_channel'] = reward_chan
        cue_chan = p['cueChannels'][this_stim_idx]
        punish_chan = p['punishChannels'][this_stim_idx]

        # generate random withold requirement (if enabled)
        if p['witholdBeforeStim']:
            withold_min = p['witholdBeforeStimMin']
            withold_max = p['witholdBeforeStimMax']
            withold_req = np.random.uniform(withold_min, withold_max)
        else:
            withold_req = 0
        trials['results'][trial_num]['withold_req'] = withold_req

        # construct arduino config string. Format = <KEY:value;KEY:value;>
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
            'PUNISH_TRIGGER:' + \
            str(p['punishTrigger']) + ';' \
            'PUNISH_CHAN:' + \
            str(punish_chan) + ';' \
            'PUNISH_DELAY:' + \
            str(p['punishDelay']) + ';' \
            'PUNISH_LENGTH:' + \
            str(int(p['punishLength']*1000)) + ';' \
            'REWARD_REMOVAL:' + \
            str(int(p['rewardRemoval'])) + ';' \
            'REWARD_REMOVAL_DELAY:' + \
            str(int(p['rewardRemovalDelay']*1000)) + ';' \
            'CUE_CHAN:' + \
            str(cue_chan) + ';' \
            'POST_STIM_CANCEL:' + \
            str(post_stim_cancel) + ';' \
            '>'

        # write config string to arduino
        arduino_ready = 0
        while not arduino_ready:
            write_string = '@?'
            self.comm_feed_signal.emit(write_string, 'pc')
            arduino['device'].write(write_string.encode('utf-8'))
            temp_read = arduino['device'].readline().strip().decode('utf-8')
            self.comm_feed_signal.emit(temp_read, 'arduino')
            if temp_read == '{!}':
                arduino_ready = 1
                write_string = config_string
                self.comm_feed_signal.emit(write_string, 'pc')
                arduino['device'].write(write_string.encode('utf-8'))
                temp_read = arduino['device'].readline().strip().decode('utf-8')
                self.comm_feed_signal.emit(temp_read, 'arduino')

    # signals allow communication between the TrialRunner thread and GUI thread. i.e. send data to main GUI thread where it can be displayed and saved. I don't know why they are here outside of any function...
    response_signal = pyqtSignal(int, float, int, str, bool, name='responseSignal')
    trial_start_signal = pyqtSignal(int, name='trialStartSignal')
    trial_end_signal = pyqtSignal(int, name='trialEndSignal')
    session_end_signal = pyqtSignal(name='sessionEndGUISignal')
    arduino_connected_signal = pyqtSignal(name='arduinoConnectedSignal')
    arduino_disconnected_signal = pyqtSignal(name='arduinoDisconnectedSignal')
    comm_feed_signal = pyqtSignal(str, str, name='commFeedSignal')

    def receiveData(self, trial_num):
        '''
        The worker thread will sit in a while loop processing incoming
        communication from the arduino, appending data to the results,
        unitl the arduino says it has finished the trial.
        '''
        trials['results'][trial_num]['trialstart'] = time.strftime('%Y%m%d_%H%M%S')
        trialRunning = True
        state = 'PRETRIAL'
        is_first_response = True
        while trialRunning:
            try:
                temp_read = arduino['device'].readline().strip().decode('utf-8')
                if temp_read:
                    self.comm_feed_signal.emit(temp_read, 'arduino')
                    if temp_read[0] == '<' and temp_read[-1] == '>':  # whole data packet received
                        temp_read = temp_read[1:-1]  # only process everything between < and >
                        data = temp_read.split('|')
                        for idx, val in enumerate(data):
                            if val:  # in val is not just
                                ID, val = val.split(':')
                                val = float(val) / 1000
                                if ID == '*':
                                    state = 'INTRIAL'
                                    trials['results'][trial_num]['responses'][0] = [x - val for x in trials['results'][trial_num]['responses'][0]]
                                    trials['results'][trial_num]['withold_act'] = val
                                else:
                                    ID = int(ID)
                                    trials['results'][trial_num]['responses'][0].append(val)  # time
                                    trials['results'][trial_num]['responses'][1].append(ID)  # channel
                                    self.response_signal.emit(trial_num, val, ID, state, is_first_response)
                                    if state == 'INTRIAL':
                                        is_first_response = False  # first response has now happened

                    elif temp_read[0] == '{' and temp_read[-1] == '}':  # whole trial outcome packet received
                        temp_read = temp_read[1:-1]  # only process everything between { and }
                        if temp_read == 'DONE':
                            trialRunning = False
                            if trials['results'][trial_num]['correct']:
                                score = 1
                                trials['correct_tally'] += 1
                            elif trials['results'][trial_num]['incorrect']:
                                score = -1
                                trials['correct_tally'] = 0
                            else:  # miss
                                score = 0
                            trials['running_score'] = np.append(trials['running_score'], score)

                        else:
                            data = temp_read.split('|')
                            for idx, val in enumerate(data):
                                if val:  # in val is not just
                                    key, val = val.split(':')
                                    val = int(val)
                                    key = str(key)
                                    trials['results'][trial_num][key] = val
            except:
                if self._session_running:
                    self.comm_feed_signal.emit('Something went wrong', 'pc')
                else:
                    self.stop()

    def stop(self):
        if self._session_running:
            self._session_running = False  # will stop while loop
            self.disconnectArduino()
            self.session_end_signal.emit()  # will update gui


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

        # set up dictionaries
        self.defaults = {}
        self.trial_config = {}
        self.trial_log = []

        # ready
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
        self.setWindowTitle('PyBehaviour - ' + p['sessionID'])
        self.sessionTimer.start(100)  # start the QTimer, executes every 100ms
        self.sessionTimer_label.setStyleSheet('font-size: 18pt; font-weight: bold; color:''black'';')

        self.trialThread.start()

    def reset(self):
        # if self.trialRunner._session_running:
            # self.trialRunner.stop()
            # self.trialThread.quit()
        plots = [self.runningScorePlot, self.averageScorePlot]
        for plot in plots:
            plot.set_data([[], []])
        for plot in self.subScorePlots:
            plot.set_data([[], []])
        plots = [self.preTrialRaster_responses, self.raster_responses]
        for plot in plots:
            plot.set_offsets(np.empty(0))
            plot.set_array(np.empty(0))
        self.rasterFigPerfAx.imshow(np.ones([1,1,3]))
        self.performanceFigPerfAx.imshow(np.ones([1,1,3]))
        self.updatePlotLayouts()

        global trials
        trials = {}
        trials['results'] = []
        trials['running_score'] = np.empty(0)
        trials['correct_tally'] = 0  # not currently used, but will be used for auto incrementing

    def pause(self):
        self.trialRunner._paused = not self.trialRunner._paused
        if self.trialRunner._paused:
            self.sessionPause_pushButton.setText('Resume')
        else:
            self.sessionPause_pushButton.setText('Pause')

    def abort(self):
        if self.trialRunner._session_running:
            self.trialRunner.stop()
            self.trialThread.quit()

    def updatePlotLayouts(self):
        self.preTrialRasterFigAx.set_xlim(0, p['witholdBeforeStimMax'])

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

        # performance line
        self.performanceFigAx.set_ylim(-1, 1)
        self.performanceFigAx.set_xlim(0, p['sessionDuration'])
        self.perf_0_line.set_xdata([0, p['sessionDuration']])

        # performance blocks
        num_stims = len(p['stimChannels'])
        self.rasterFigPerfAx.set_ylim(-0.5, p['sessionDuration']-0.5)
        self.rasterFigPerfAx.set_xlim(-0.5, num_stims-0.5)
        self.performanceFigPerfAx.set_ylim(-0.5, num_stims-0.5)
        self.performanceFigPerfAx.set_xlim(-0.5, p['sessionDuration']-0.5)

        self.preTrialRasterFigCanvas.draw()
        self.rasterFigCanvas.draw()
        self.performanceFigCanvas.draw()

    def updateRasterPlotData(self, trial_num, val, ID, state, is_first_response):
        if state == 'INTRIAL':
            plot_series = self.raster_responses
            ax = self.rasterFigAx
            canvas = self.rasterFigCanvas
            x = val
            y = trial_num
        elif state == 'PRETRIAL':
            plot_series = self.preTrialRaster_responses
            ax = self.preTrialRasterFigAx
            canvas = self.preTrialRasterFigCanvas
            x = val
            y = 0
            if x > p['witholdBeforeStimMax']:
                ax.set_xlim([0, x])
                canvas.draw()

        old_data = plot_series.get_offsets()
        new_data = np.append(old_data, [x, y])
        plot_series.set_offsets(new_data)

        old_colours = plot_series.get_array()
        new_colours = np.append(old_colours, ID)
        plot_series.set_array(new_colours)

        if is_first_response:
            new_size = 26
        else:
            new_size = 2
        old_sizes = plot_series.get_sizes()
        new_sizes = np.append(old_sizes, new_size)
        plot_series.set_sizes(new_sizes)

        ax.draw_artist(plot_series)
        canvas.update()
        canvas.flush_events()

    def updateResultBlockPlots(self, trial_num):
        num_stims = len(p['stimChannels'])
        num_trials = p['sessionDuration']

        performance_record = np.ones([num_trials, num_stims, 3])
        for t in range(trial_num+1):
            if trials['results'][t]['correct']:
                colour = [0, 0.85, 0.45]
            elif not trials['results'][t]['miss']:  # means incorrect
                colour = [1, 0.1, 0.4]
            else:  # means miss
                colour = [.7, .7, .7]
            stim_idx = p['stimChannels'].index(p['trialOrder'][t])
            performance_record[t, stim_idx] = colour

        self.rasterFigPerfAx.imshow(performance_record, interpolation='nearest', aspect='auto')
        self.performanceFigPerfAx.imshow(np.rot90(performance_record), interpolation='nearest', aspect='auto')

    def updatePerformancePlot(self, trial_num):
        plot_series = self.runningScorePlot
        old_ydata = plot_series.get_ydata()
        moving_avg_size = 10
        if trial_num >= moving_avg_size:
            new_ydata = np.append(old_ydata, np.mean(trials['running_score'][-moving_avg_size:]))
        else:
            new_ydata = np.append(old_ydata, np.mean(trials['running_score']))
        plot_series.set_ydata(new_ydata)
        plot_series.set_xdata(range(len(new_ydata)))

        plot_series = self.averageScorePlot
        plot_series.set_ydata([np.mean(trials['running_score']), np.mean(trials['running_score'])])
        plot_series.set_xdata([0, p['sessionDuration']])

        for stim_type in p['stimChannels']:
            plot_series = self.subScorePlots[stim_type-1]
            mask = p['trialOrder'][:trial_num+1] == stim_type
            if mask.any():
                plot_series.set_ydata([np.mean(trials['running_score'][mask]), np.mean(trials['running_score'][mask])])
                plot_series.set_xdata([0, p['sessionDuration']])

    def setConnects(self):
        # add connects for control button clicks
        self.setDefaults_Button.clicked.connect(self.GUISave)
        self.saveAsPreset_Button.clicked.connect(self.saveAsPreset)
        self.testPin_Button.clicked.connect(self.testPin)
        self.begin_Button.clicked.connect(self.begin)
        self.sessionPause_pushButton.clicked.connect(self.pause)
        self.sessionAbort_pushButton.clicked.connect(self.abort)
        self.loadTrialOrder_pushButton.clicked.connect(self.loadTrialOrder)
        self.saveTrialOrder_pushButton.clicked.connect(self.saveTrialOrder)

        self.loadPreset_ComboBox.currentIndexChanged.connect(self.loadPreset)

        self.sessionTimer = QTimer(self)
        self.sessionTimer.timeout.connect(self.sessionTimerUpdate)
        self.trialRunner.arduino_connected_signal.connect(self.arduinoConnected)
        self.trialRunner.arduino_disconnected_signal.connect(self.arduinoDisconnected)
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

    def arduinoDisconnected(self):
        self.arduinoConnectedText_Label.setText('Not connected')
        self.arduinoConnectedText_Label.setStyleSheet('color:rgb(255, 0, 100);')

    def makeTrialOrder(self):
        if not self.trialRunner._session_running:
            num_stims = len(p['stimChannels'])
            num_trials = p['sessionDuration']
            if num_stims > 0:
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
                    if len(p['trialOrder']) < num_trials:
                        diff = p['sessionDuration'] - len(p['trialOrder'])
                        xk = p['stimChannels']
                        pk = p['proportions']
                        custm = stats.rv_discrete(name='custm', values=(xk, pk))
                        extra_trials = custm.rvs(size=diff)
                        p['trialOrder'] = np.append(p['trialOrder'], extra_trials)

                    # test for consecutive trial limit
                    passed = False
                    running_count = 0
                    attempt = 0
                    failed_on = 0
                    if num_trials > 1:
                        while not passed:
                            prev_failed_on = failed_on
                            for i in range(num_trials):
                                if i > 0:
                                    this_stim = p['trialOrder'][i]
                                    prev_stim = p['trialOrder'][i-1]
                                    if this_stim == prev_stim:
                                        running_count +=1
                                    else:
                                        running_count = 0
                                    if running_count >= p['stimOrderGroupSize']:
                                        np.random.shuffle(p['trialOrder'][i:])
                                        failed_on = i
                                        break

                                    if i == num_trials-1:
                                        passed = True
                            if failed_on == prev_failed_on:  # if gets stuck (usually at end)
                                attempt += 1
                            else:
                                attempt = 0
                            if attempt >= 10:
                                passed = True

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

                # trial variations
                p['trialVariations'] = np.zeros(len(p['trialOrder']))
                for i in range(len(p['trialOrder'])):
                    this_trial = p['trialOrder'][i]
                    this_idx = p['stimChannels'].index(this_trial)
                    p['trialVariations'][i] = np.random.randint(p['variations'][this_idx])

                # first stim control
                if p['controlFirstStim']:
                    if p['controlFirstStimStim']:
                        p['trialOrder'][0:p['firstXStims']] = int(p['firstStim'])
                    if p['controlFirstStimVariation']:
                        p['trialVariations'][0:p['firstXStims']] = int(p['firstStimVariation'])

                self.plotTrialOrder()

    def plotTrialOrder(self):
        self.trialOrderAx.imshow([p['trialOrder']], interpolation='nearest', cmap='rainbow', vmin=1, vmax=8, aspect='auto')
        self.trialOrderAx2.imshow([p['trialVariations']], interpolation='nearest', cmap='gray', aspect='auto')
        self.trialOrderCanvas.draw()

    def saveTrialOrder(self):
        filepath = str(QFileDialog.getSaveFileName(self, 'Save trial sequence', trial_sequence_directory, 'Trial sequence (*.txt)')[0])
        arr = np.vstack((p['trialOrder'], p['trialVariations']))
        np.savetxt(filepath, arr, fmt='%1i', delimiter=',')

    def loadTrialOrder(self):
        filepath = str(QFileDialog.getOpenFileName(self, 'Load trial sequence', trial_sequence_directory, '*.txt')[0])
        arr = np.genfromtxt(filepath, delimiter=',')
        p['trialOrder'] = arr[0]
        p['trialVariations'] = arr[1]

        filename = os.path.splitext(os.path.basename(filepath))[0]
        self.stimOrder_ComboBox.setCurrentIndex(2)
        self.loadedTrialSequence_label.setText('Loaded: ' + filename)

        self.getValues()

    def getValues(self):
        # extract gui values
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

        # process gui values
        stimChannels = [p['stim1'], p['stim2'], p['stim3'], p['stim4'],
                        p['stim5'], p['stim6'], p['stim7'], p['stim8']]

        respRequired = [p['respReq1'], p['respReq2'], p['respReq3'], p['respReq4'],
                        p['respReq5'], p['respReq6'], p['respReq7'], p['respReq8']]

        rewardChannels = [p['rewardChan1'], p['rewardChan2'], p['rewardChan3'], p['rewardChan4'],
                          p['rewardChan5'], p['rewardChan6'], p['rewardChan7'], p['rewardChan8']]

        cueChannels = [p['cueChan1'], p['cueChan2'], p['cueChan3'], p['cueChan4'],
                       p['cueChan5'], p['cueChan6'], p['cueChan7'], p['cueChan8']]

        punishChannels = [p['punishChan1'], p['punishChan2'], p['punishChan3'], p['punishChan4'],
                          p['punishChan5'], p['punishChan6'], p['punishChan7'], p['punishChan8']]

        proportions = [p['proportion1'], p['proportion2'], p['proportion3'], p['proportion4'],
                       p['proportion5'], p['proportion6'], p['proportion7'], p['proportion8']]

        variations = [p['variations1'], p['variations2'], p['variations3'], p['variations4'],
                      p['variations5'], p['variations6'], p['variations7'], p['variations8']]

        p['stimChannels'] = [int(idx+1) for idx, val in enumerate(stimChannels) if val]
        p['cueChannels'] = [cueChannels[idx-1] for idx in p['stimChannels']]
        p['punishChannels'] = [punishChannels[idx-1] for idx in p['stimChannels']]
        p['respRequired'] = [respRequired[idx-1] for idx in p['stimChannels']]
        p['rewardChannels'] = [rewardChannels[idx-1] for idx in p['stimChannels']]
        p['proportions'] = [proportions[idx-1] for idx in p['stimChannels']]
        p['variations'] = [variations[idx-1] for idx in p['stimChannels']]

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

        # update gui
        if self._ready:
            self.makeTrialOrder()
            self.updateTrialConfigPlot()
            self.updatePlotLayouts()

            # colour stim names if selected
            stimCheckBoxes = [self.stim1_CheckBox, self.stim2_CheckBox, self.stim3_CheckBox, self.stim4_CheckBox,
                              self.stim5_CheckBox, self.stim6_CheckBox, self.stim7_CheckBox, self.stim8_CheckBox]
            cmap = matplotlib.cm.get_cmap(name='rainbow', lut=len(stimCheckBoxes))
            for idx, stimCheckBox in enumerate(stimCheckBoxes):
                colour = list(cmap(idx))
                colour_string = 'rgb(' + str(int(colour[0]*255)) + ',' + str(int(colour[1]*255)) + ',' + str(int(colour[2]*255)) + ')'
                stimCheckBox.setStyleSheet('QCheckBox::indicator:checked { background-color:' + colour_string +
                    '; border: 1px solid #b1b1b1;}')

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
                    self.defaults[name] = str(obj.currentText())
                if isinstance(obj, QCheckBox):
                    self.defaults[name] = bool(obj.isChecked())
                if isinstance(obj, QLineEdit):
                    if 'spinbox' not in name:
                        self.defaults[name] = str(obj.text())
                if isinstance(obj, QSpinBox):
                    self.defaults[name] = int(obj.value())
                if isinstance(obj, QDoubleSpinBox):
                    self.defaults[name] = float(obj.value())
        json.dump(self.defaults, open(os.path.join('GUI', 'GUIdefaults.cfg'), 'w'), sort_keys=True, indent=4)

    def GUIRestore(self):
        if os.path.isfile(os.path.join('GUI', 'GUIdefaults.cfg')):
            self.defaults = json.load(open(os.path.join('GUI', 'GUIdefaults.cfg'), 'r'))
            widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
            groups = (self.session_GroupBox, self.arduino_GroupBox)
            for group in groups:
                for obj in group.findChildren(widgets):
                    name = str(obj.objectName())
                    try:
                        if isinstance(obj, QComboBox):
                            value = self.defaults[name]
                            if value == '':
                                continue
                            index = obj.findText(value)  # get idx for string in combo
                            if index == -1:  # add to list if not found
                                continue
                            else:
                                obj.setCurrentIndex(index)  # preselect a combobox value
                        if isinstance(obj, QLineEdit):
                            if 'spinbox' not in name:
                                value = self.defaults[name]
                                obj.setText(value)  # restore lineEditFile
                        if isinstance(obj, QCheckBox):
                            value = self.defaults[name]
                            if value is not None:
                                obj.setChecked(value)  # restore checkbox
                        if isinstance(obj, QSpinBox):
                            value = self.defaults[name]
                            obj.setValue(value)  # restore lineEditFile
                        if isinstance(obj, QDoubleSpinBox):
                            value = self.defaults[name]
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
        self.duration_line = patches.Rectangle([0, 0], 0, 0.05, fc=[.8, .8, .8], ec='none', zorder=0)
        self.trial_cue_rectangle = patches.Rectangle([0, 0], 0, 0.9, fc=[1, .7, 0], ec='none')
        self.stim_cue_rectangle = patches.Rectangle([0, 0], 0, 0.9, fc=[1, .7, 0], ec='none')
        self.response_cue_rectangle = patches.Rectangle([0, 0], 0, 0.9, fc=[1, .7, 0], ec='none')
        self.stim_rectangle = patches.Rectangle([0, 0], 0, 1, fc=[.3, .3, .3], ec='none')
        self.resp_rectangle = patches.Rectangle([0, 0], 0, 0.9, fc=[.8, .8, .8], ec='none')
        self.reward_rectangle = patches.Rectangle([0, 0], 0, 0.9, fc=[0, .75, .95], ec='none')

        # add the shapes to the figure
        self.trialConfigAx.add_patch(self.stim_rectangle)
        self.trialConfigAx.add_patch(self.resp_rectangle)
        self.trialConfigAx.add_patch(self.reward_rectangle)
        self.trialConfigAx.add_patch(self.trial_cue_rectangle)
        self.trialConfigAx.add_patch(self.stim_cue_rectangle)
        self.trialConfigAx.add_patch(self.response_cue_rectangle)
        self.trialConfigAx.add_patch(self.duration_line)

        # add to GUI
        self.advancedGroupBox_layout.addWidget(self.trialConfigCanvas)

        # pretrial response/withold plot
        self.preTrialRasterFig = Figure()
        self.preTrialRasterFigCanvas = FigureCanvas(self.preTrialRasterFig)  # a canvas holds a figure
        self.preTrialResponseVerticalLayout.addWidget(self.preTrialRasterFigCanvas)
        self.preTrialRasterFigAx = self.preTrialRasterFig.add_axes([0.1, 0.5, .8, 0.1])
        self.preTrialRasterFigAx.hold(True)
        self.preTrialRasterFigAx.axis('on')
        self.preTrialRasterFigAx.get_yaxis().set_visible(False)
        self.preTrialRasterFigAx.set_title('Pre-trial', loc='left')
        self.preTrialRaster_responses = self.preTrialRasterFigAx.scatter(np.empty(0), np.empty(0), c=np.empty(0), edgecolor='', antialiased=True, clip_on=False, vmin=1, vmax=4, cmap='rainbow', zorder=9)
        self.preTrialRasterFigCanvas.setFixedHeight(50)
        self.preTrialRasterFig.set_facecolor('white')
        self.preTrialRasterFigCanvas.draw()

        # main response raster plot
        self.rasterFig = Figure()
        self.rasterFigCanvas = FigureCanvas(self.rasterFig)  # a canvas holds a figure
        self.resultsFigsHorizontalLayout.addWidget(self.rasterFigCanvas)
        self.rasterFigAx = self.rasterFig.add_axes([0.2, 0.15, 0.67, 0.75])
        self.rasterFig.set_facecolor('white')
        self.rasterFigAx.set_title('Responses', loc='left')
        self.rasterFigAx.hold(True)

        self.raster_respwin = patches.Rectangle([0, 0], 0, 0, fc=[0, 0, 0, 0.05], ec='none', zorder=1)
        self.raster_stimlength = patches.Rectangle([0, 0], 0, 0, fc=[0, 0, 0, 0.05], ec='none', zorder=3)
        self.rasterFigAx.add_patch(self.raster_respwin)
        self.rasterFigAx.add_patch(self.raster_stimlength)
        self.raster_stimline, = self.rasterFigAx.plot([0, 0], [0, 0], '-', c=[0.3, 0.3, 0.3], linewidth=2, clip_on=False, zorder=8)
        self.raster_responses = self.rasterFigAx.scatter(np.empty(0), np.empty(0), c=np.empty(0), edgecolor='', antialiased=True, clip_on=False, vmin=1, vmax=4, cmap='rainbow', zorder=9)
        # self.raster_reward, = self.rasterFigAx.plot([], [], 'o', clip_on=False)  # currently unused
        # self.raster_punish, = self.rasterFigAx.plot([], [], 'o', clip_on=False)  # currently unused

        self.rasterFigAx.set_xlabel('Time (s)')
        self.rasterFigAx.set_ylabel('Trial')

        self.rasterFigPerfAx = self.rasterFig.add_axes([0.9, 0.15, 0.05, 0.75])
        self.rasterFigPerfAx.axis('off')

        # results plot
        self.performanceFig = Figure()
        self.performanceFigCanvas = FigureCanvas(self.performanceFig)  # a canvas holds a figure
        self.performanceFigAx = self.performanceFig.add_axes([0.2, 0.15, 0.75, 0.7])
        self.resultsFigsHorizontalLayout.addWidget(self.performanceFigCanvas)
        self.perf_0_line, = self.performanceFigAx.plot([0, 0], [0, 0], '-', c=[0.3, 0.3, 0.3], linewidth=1, zorder=6, clip_on=False)

        self.runningScorePlot, = self.performanceFigAx.plot([], [], 'k-', linewidth=2, aa=True, clip_on=False, zorder=9)
        self.averageScorePlot, = self.performanceFigAx.plot([], [], '-', c=[0.7, 0.7, 0.7], linewidth=2, aa=True, clip_on=False, zorder=8)

        NUM_STIMS = 8
        cmap = matplotlib.cm.get_cmap(name='rainbow', lut=NUM_STIMS-1)
        self.subScorePlots = []
        for sub_plot in range(NUM_STIMS):
            temp, = self.performanceFigAx.plot([], [], '-', c=cmap(sub_plot), linewidth=1, aa=True, clip_on=False, zorder=7)
            self.subScorePlots.append(temp)

        self.performanceFigAx.set_xlabel('Trial')
        self.performanceFigAx.set_ylabel('Performance (%)')
        self.performanceFigAx.set_yticks(np.linspace(-1,1,9))
        # self.performanceFigAx.get_yaxis().set_minor_locator(matplotlib.ticker.AutoMinorLocator())
        # self.performanceFigAx.grid(b=True, which='minor', alpha=0.5)
        self.performanceFigAx.set_yticklabels([-100,-75,-50,-25,0,25,50,75,100])
        self.performanceFig.set_facecolor('white')

        self.performanceFigPerfAx = self.performanceFig.add_axes([0.2, 0.86, 0.75, 0.04])
        self.performanceFigPerfAx.axis('off')
        self.performanceFigPerfAx.set_title('Performance', loc='left')

    def saveAsPreset(self):
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        for obj in self.trial_GroupBox.findChildren(widgets):
            name = str(obj.objectName())
            if isinstance(obj, QComboBox):
                self.trial_config[name] = str(obj.currentText())
            if isinstance(obj, QCheckBox):
                self.trial_config[name] = bool(obj.isChecked())
            if isinstance(obj, QLineEdit):
                if 'spinbox' not in name:
                    self.trial_config[name] = str(obj.text())
            if isinstance(obj, QSpinBox):
                self.trial_config[name] = int(obj.value())
            if isinstance(obj, QDoubleSpinBox):
                self.trial_config[name] = float(obj.value())
        filepath = str(QFileDialog.getSaveFileName(self, 'Save as preset...', 'Configs', 'Config file (*.cfg)')[0])
        json.dump(self.trial_config, open(filepath, 'w'), sort_keys=True, indent=4)
        filename = os.path.basename(filepath)
        filename = os.path.splitext(filename)[0]
        self.loadPreset_ComboBox.addItems([filename])
        index = self.loadPreset_ComboBox.findText(filename)
        self.loadPreset_ComboBox.setCurrentIndex(index)

    def loadPreset(self):
        filename = str(self.loadPreset_ComboBox.currentText())
        filepath = os.path.join(config_directory, filename + '.cfg')
        self.trial_config = json.load(open(filepath, 'r'))
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        for obj in self.trial_GroupBox.findChildren(widgets):
            name = str(obj.objectName())
            try:
                if isinstance(obj, QComboBox):
                    value = self.trial_config[name]
                    index = obj.findText(value)  # get the corresponding index for specified string in combobox
                    obj.setCurrentIndex(index)  # preselect a combobox value by index
                if isinstance(obj, QLineEdit):
                    value = self.trial_config[name]
                    if 'spinbox' not in name:
                        obj.setText(value)  # restore lineEditFile
                if isinstance(obj, QCheckBox):
                    value = self.trial_config[name]
                    if value is not None:
                        obj.setChecked(value)  # restore checkbox
                if isinstance(obj, QSpinBox):
                    value = self.trial_config[name]
                    obj.setValue(value)  # restore lineEditFile
                if isinstance(obj, QDoubleSpinBox):
                    value = self.trial_config[name]
                    obj.setValue(value)  # restore lineEditFile
            except:
                continue

    def closeEvent(self, event):
        self.trialRunner.stop()
        time.sleep(0.1)
        self.trialThread.quit()
        event.accept()

    def sessionEndGUI(self):
        self.sessionTimer.stop()
        end_time_string = self.sessionTimer_label.text()
        self.sessionTimer_label.setText('<font color=''#ff0066''>' + end_time_string + '</font>')
        self.updateCommFeed('Finished')

    def trialStartGUI(self, trial_num):
        self.trialNum_label.setText(str(trial_num+1))
        self.updateCommFeed('\n')
        self.updateCommFeed('Trial ' + str(trial_num+1), 'trial')
        plots = [self.preTrialRaster_responses]
        for plot_series in plots:
            plot_series.set_offsets(np.empty(0))
            plot_series.set_array(np.empty(0))
        self.preTrialRasterFigAx.set_xlim([0, p['witholdBeforeStimMax']])
        self.preTrialRasterFigCanvas.draw()

    def trialStopGUI(self, trial_num):
        self.updatePerformancePlot(trial_num)
        self.updateResultBlockPlots(trial_num)
        self.performanceFigCanvas.draw()
        # self.rasterFigCanvas.draw()
        self.saveResults()

    def updateCommFeed(self, input_string, device=None):
        # input_string = input_string.replace('<', '{').replace('>', '}')
        if device == 'pc':
            self.toSessionFeed_textEdit.append(input_string)
            input_string = 'PC:      ' + input_string
        elif device == 'arduino':
            self.fromSessionFeed_textEdit.append(input_string)
            input_string = 'ARDUINO: ' + input_string
        elif device == 'trial':
            self.toSessionFeed_textEdit.setText(input_string)
            self.fromSessionFeed_textEdit.setText('')
        self.trial_log.append(input_string)

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
            log.write('\n'.join(self.trial_log))


# Main entry to program.  Sets up the main app and create a new window.
def main(argv):
    # create Qt application
    app = QApplication(argv)
    app.setStyle('fusion')

    # create main window
    GUI = MainWindow()

    # resize if larger than desktop
    screen_res = QDesktopWidget().availableGeometry()
    if GUI.frameSize().height() > screen_res.height():
        GUI.resize(GUI.frameSize().width(), screen_res.height())

    # centre the window
    screen_res = QDesktopWidget().availableGeometry()
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
