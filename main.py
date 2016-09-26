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
from matplotlib.colors import LinearSegmentedColormap
from cycler import cycler
from matplotlib import cm
import seaborn
seaborn.set(rc={
    'axes.axisbelow': True,
    'axes.linewidth': 1,
    'axes.facecolor': [1.0, 1.0, 1.0],
    'axes.edgecolor': [0.9, 0.9, 0.9],
    'grid.color': [0.9, 0.9, 0.9]
    })
import numpy as np
import scipy.io as sio
from scipy import stats
from statsmodels.stats.proportion import proportion_confint
import json
import os
import time
import sys
import serial
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QThread, QTimer
from PyQt5.QtWidgets import (QComboBox, QCheckBox, QLineEdit, QSpinBox,
                             QDoubleSpinBox, QFileDialog, QApplication,
                             QDesktopWidget, QMainWindow, QMessageBox)
from PyQt5.QtGui import QColor, QIcon, QPalette
from GUI import GUI
from GUI import serial_ports
import logging
import pickle
#from scipy.signal import savgol_filter
#from scipy.ndimage.filters import gaussian_filter1d

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# create a file handler
handler = logging.FileHandler('errors.log')
handler.setLevel(logging.DEBUG)

# create a logging format
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)

# add the handlers to the logger
logger.addHandler(handler)

logger.info('Started application')

class TrialRunner(QObject):
    '''
    Background worker. Runs the trials.
    '''
    def __init__(self):
        super(TrialRunner, self).__init__()
        self._session_running = False
        self._paused = False
        self._exiting = False
        self.trial_num = 0

    def startSession(self):
        while not self._exiting:
            time.sleep(0.01)
            # the main session loop
            try:
                while self._session_running:
                    if arduino['connected'] is False:
                       self.connectArduino()
                    if arduino['connected']:
                        while self._paused:
                            time.sleep(0.01)

                        self.runTrial(self.trial_num, trials)

                        self.trial_num += 1
                        if p['sessionDurationMode'] == 'Trials':
                            if self.trial_num == p['sessionDuration']:
                                self._session_running = False
                                # self.session_end_signal.emit()
                    else:
                        self._session_running = False

                    if self._session_running is False:
                        self.session_end_signal.emit()
            except Exception as e:
                logger.exception(e)


    def connectArduino(self):
        self.comm_feed_signal.emit('Connecting Arduino on port ' + p['device'], 'pc')
        try:
            arduino['device'] = serial.Serial(p['device'], 19200)
            arduino['device'].timeout = 1
            connect_attempts = 1
            current_attempt = 1
            while arduino['connected'] is False and current_attempt <= connect_attempts:
                temp_read = arduino['device'].readline().strip().decode('utf-8')
                self.comm_feed_signal.emit(temp_read, 'arduino')
                if temp_read == '{READY}':
                    arduino['connected'] = True
                    self.arduino_connected_signal.emit()
                else:
                    current_attempt += 1
                if current_attempt > connect_attempts:
                    self.comm_feed_signal.emit('Failed to connect', 'pc')
                    arduino['device'].close()
        except serial.SerialException as e:
            logger.exception(e)
            self.comm_feed_signal.emit('Serial error', 'pc')

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
        trials['results'][trial_num]['firstresponse'] = []
        trials['results'][trial_num]['correct'] = []
        trials['results'][trial_num]['incorrect'] = []
        trials['results'][trial_num]['miss'] = []

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
            if (trial_num >= max_repeats) and (np.sum(trials['running_score'][trial_num-max_repeats:trial_num]) <= 0) and (prev_trials.min() == prev_trials.max()):
                pass  # do not repeat any more if max_repeats of a stim has been exceeded
                    # add in here the option of changing to a different stim type if desired
            elif trials['running_score'][trial_num-1] <= 0:
                # shift next trials back
                p['trialOrder'][trial_num+1:-1] = p['trialOrder'][trial_num:-2]
                p['trialVariations'][trial_num+1:-1] = p['trialVariations'][trial_num:-2]

                # make this stim same as previous
                p['trialOrder'][trial_num] = p['trialOrder'][trial_num-1]
                p['trialVariations'][trial_num] = p['trialVariations'][trial_num-1]




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
        post_stim_cancel = p['postStimCancel']

        # allow second chance if correc choice made after incorrect choice?
        second_chance = p['secondChance']

        # get reward, cue and punish channels
        reward_chan = p['rewardChannels'][this_stim_idx]
        trials['results'][trial_num]['reward_channel'] = reward_chan
        cue_chan = p['cueChannels'][this_stim_idx]
        punish_chan = p['punishChannels'][this_stim_idx]

        # auto reward?
        auto_reward = p['autoRewards'][this_stim_idx]
        trials['results'][trial_num]['auto_reward'] = auto_reward
        # add in here the option to trigger auto reward if previous X trials were wrong

        # generate withold requirement (if enabled)
        if p['witholdBeforeStim']:
            withold_min = p['witholdBeforeStimDuration'] - p['witholdBeforeStimRandomise']
            withold_max = p['witholdBeforeStimDuration'] + p['witholdBeforeStimRandomise']
            withold_req = np.random.uniform(withold_min, withold_max)
        else:
            withold_req = 0
        trials['results'][trial_num]['withold_req'] = withold_req

        # get random pre stim delay (if enabled)
        pre_stim_delay_min = p['startToStimDelay'] - p['startToStimDelayRandomise']
        pre_stim_delay_max = p['startToStimDelay'] + p['startToStimDelayRandomise']
        pre_stim_delay = np.random.uniform(pre_stim_delay_min, pre_stim_delay_max)
        trials['results'][trial_num]['pre_stim_delay'] = pre_stim_delay

        # get random post stim delay (if enabled)
        post_stim_delay_min = p['postStimDelay'] - p['postStimDelayRandomise']
        post_stim_delay_max = p['postStimDelay'] + p['postStimDelayRandomise']
        post_stim_delay = np.random.uniform(post_stim_delay_min, post_stim_delay_max)
        trials['results'][trial_num]['post_stim_delay'] = post_stim_delay

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
            str(int(auto_reward)) + ';' \
            'AUTO_REWARD_START:' + \
            str(int(p['autoRewardStart']*1000)) + ';' \
            'PUNISH_TRIGGER:' + \
            str(int(p['punishTrigger'])) + ';' \
            'PUNISH_CHAN:' + \
            str(punish_chan) + ';' \
            'PUNISH_DELAY:' + \
            str(int(p['punishDelay'])) + ';' \
            'PUNISH_DELAY_LENGTH:' + \
            str(int(p['punishDelayLength']*1000)) + ';' \
            'REWARD_REMOVAL:' + \
            str(int(p['rewardRemoval'])) + ';' \
            'REWARD_REMOVAL_DELAY:' + \
            str(int(p['rewardRemovalDelay']*1000)) + ';' \
            'CUE_CHAN:' + \
            str(cue_chan) + ';' \
            'POST_STIM_CANCEL:' + \
            str(int(post_stim_cancel)) + ';' \
            'SECOND_CHANCE:' + \
            str(int(second_chance)) + ';' \
            'REWARD_DURATION:' + \
            str(int(p['rewardDuration']*1000)) + ';' \
            'PUNISH_TRIGGER_DURATION:' + \
            str(int(p['punishTriggerDuration']*1000)) + ';' \
            '>'

        # write config string to arduino
        arduino_ready = 0
        while not arduino_ready:
            try:
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
            except Exception as e:
                logger.exception(e)
                self.comm_feed_signal.emit('Something went wrong', 'pc')
                self.disconnectArduino()
                self.connectArduino()

    # signals allow communication between the TrialRunner thread and GUI thread. i.e. send data to main GUI thread where it can be displayed and saved. I don't know why they are here outside of any function...
    response_signal = pyqtSignal(int, float, int, str, bool, name='responseSignal')
    results_signal = pyqtSignal(int, name='resultsSignal')
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
                if arduino['connected'] == False:
                    self.comm_feed_signal.emit('Arduino not connected', 'pc')
                    trials['running_score'][trial_num] = 0
                    trialRunning = False  # abort current trial if arduino disconnects

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
                        elif temp_read == 'READY':  # arduino has reset
                            trialRunning = False
                        else:
                            data = temp_read.split('|')
                            for idx, val in enumerate(data):
                                if val:  # in val is not just
                                    # firstresponse, correct, incorrect, miss
                                    key, val = val.split(':')
                                    val = int(val)
                                    key = str(key)
                                    trials['results'][trial_num][key] = val
                            if trials['results'][trial_num]['correct']:
                                score = 1
                                trials['correct_tally'] += 1
                            elif trials['results'][trial_num]['incorrect']:
                                score = -1
                                trials['correct_tally'] = 0
                            else:  # miss
                                score = 0
                            trials['running_score'][trial_num] = score
                            self.results_signal.emit(trial_num)
            except Exception as e:
                logger.exception(e)
                if self._session_running:
                    self.comm_feed_signal.emit('Something went wrong', 'pc')
                    self.connectArduino()
                    trials['running_score'][trial_num] = 0
                    trialRunning = False

    def stop(self):
        if self._session_running:
            self._session_running = False  # will stop while loop
            self.disconnectArduino()
            self.session_end_signal.emit()  # will update gui


class MainWindow(QMainWindow, GUI.Ui_MainWindow):
    '''
    The GUI window
    '''
    def __init__(self):
        QMainWindow.__init__(self)
        self._gui_ready = False
        self.setupUi(self)

        # create the worker thread (run trials in the background)
        self.trialThread = QThread()
        self.trialRunner = TrialRunner()
        self.trialRunner.moveToThread(self.trialThread)
        self.trialThread.started.connect(self.trialRunner.startSession)
        self.trialThread.start()

        # make colormaps
        self.NUM_STIMS = 8
        self.NUM_VARS = 16
        self.defineColormaps()

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
        self.stimOrder_ComboBox.model().item(2).setEnabled(False)

        # set up dictionaries
        self.defaults = {}
        self.trial_config = {}
        self.trial_log = []

        # ready
        self._gui_ready = True

        # open the config file and populate GUI with values
        self.GUIRestore()

    def defineColormaps(self):
        # define colormap for different stims
        self.cmap_stims = cm.get_cmap(name='Spectral_r', lut=self.NUM_STIMS)

        # define colormap for score bars
        low  = [1, 0.1, 0.4]
        mid  = [.7, .7, .7]
        high = [0, 0.85, 0.45]
        colors = [low, mid, high] 
        n_bins = 100  
        self.cmap_score = LinearSegmentedColormap.from_list('hitmiss', colors, N=n_bins)
        #self.cmap_score = cm.get_cmap(name='coolwarm', lut=n_bins)

        # define colormap for response
        self.cmap_resps = cm.get_cmap(name='gray') 


    def sessionTimerUpdate(self):
        elapsed_time = int((time.time() - p['sessionStartTime']) * 1000)
        s,ms = divmod(elapsed_time, 1000)
        m, s = divmod(s, 60)
        h, m = divmod(m, 60)
        self.sessionTimer_label.setText('%d:%02d:%02d:%d' % (h, m, s, int(ms/100)))

    def begin(self):
        if self.trialRunner._session_running is False:
            # reset everything
            self.begin_Button.setEnabled(False)
            self.tabWidget.setCurrentIndex(1)
            self.sessionAbort_pushButton.setEnabled(True)
            self.forceReward_pushButton.setEnabled(True)
            self.reset()
            p['sessionStartTime'] = time.time()
            p['sessionStartTimeString'] = time.strftime('%Y%m%d_%H%M%S')
            p['sessionID'] = p['subjectID'] + '_' + p['sessionStartTimeString']
            self.setWindowTitle('PyBehaviour - ' + p['sessionID'])
            self.sessionTimer.start(100)  # start the QTimer, executes every 100ms
            self.sessionTimer_label.setStyleSheet('font-size: 18pt; font-weight: bold; color:''black'';')
            self.trialRunner._session_running = True
            logger.info('Started session: ' + p['sessionID'])

            num_stims = len(p['stimChannels'])
            #f, axs = self.reactionTimesCanvas.subplots(1, num_stims, sharey=True)

            # self.reactionTimesFig.clf()
            # ax = list()
            # for i in range(num_stims):
            #     if i > 0:
            #         ax.append(self.reactionTimesFig.add_subplot(1,num_stims,i+1, sharey=ax[0]))
            #         ax[-1].set_yticklabels('')
            #         ax[-1].set_title(str(i+1))
            #     else:
            #         ax.append(self.reactionTimesFig.add_subplot(1,num_stims,i+1))
            #         ax[-1].set_ylabel('Count')
            #         ax[-1].set_title(str(i+1))
            # self.reactionTimesCanvas.draw()

    def reset(self):
        self.trialRunner.trial_num = 0
        NUM_STIMS = 8
        num_stims = len(p['stimChannels'])
        num_trials = p['sessionDuration']
        num_max_variations = max(p['variations'])

        # reset perfromance plots
        plots = [self.runningScorePlot, self.averageScorePlot]
        for plot in plots:
            plot.set_data([np.empty(0), np.empty(0)])

        # reset raster plots
        plots = [self.preTrialRaster_responses, self.raster_responses]
        for plot in plots:
            plot.set_offsets(np.empty(0))
            plot.set_array(np.empty(0))
            plot.set_sizes(np.empty(0))

        global plot_data
        plot_data = {}
        plot_data['offsets'] = np.empty(0)

        self.raster_current_trial.set_data([0,0])

        # reset the performance block plots
        self.rasterFigPerfAxIm.set_data(np.full([num_trials,num_stims], np.nan))
        self.rasterFigPerfAxVarIm.set_data(np.full([num_trials,num_stims], np.nan))
        self.performanceFigPerfAxIm.set_data(np.full([num_stims,num_trials], np.nan))

        # reset the trial results dictionary
        global trials
        trials = {}
        trials['results'] = []
        trials['running_score'] = np.full([num_trials,1], np.nan)
        trials['reaction_time'] = np.full([num_trials,1], np.nan)
        trials['correct_tally'] = 0  # not currently used, but will be used for auto incrementing

        self.trial_log = []

        # reset reaction time hists
        self.hist_bins = np.arange(0, p['trialDuration'], 0.1)
        counts,edges = np.histogram(trials['reaction_time'], bins=self.hist_bins)

        for stim in range(NUM_STIMS):
            self.reactionTimeHists[stim].set_xy(np.empty([1,2]))
            #self.reactionTimeHists[stim].set_xy(np.hstack([np.random.rand(4,1), np.random.randint(0,5,[4,1])]))
            if stim+1 in p['stimChannels']:
                self.reactionTimeHists[stim].set_visible(True)
            else:
                self.reactionTimeHists[stim].set_visible(False)


        # reset summary results plot
        if num_max_variations > 1:
            self.summaryResultsAx.set_xlabel('Variation')
            self.summaryResultsAx.set_xlim([-1,num_max_variations])
            self.summaryResultsAx.set_xticks(np.arange(0,num_max_variations))
            self.summaryResultsAx.set_xticklabels(np.arange(0,num_max_variations)+1)
        else:
            self.summaryResultsAx.set_xlabel('Stimulus type')
            self.summaryResultsAx.set_xlim([-1,num_stims])
            self.summaryResultsAx.set_xticks(np.arange(0,num_stims))
            self.summaryResultsAx.set_xticklabels(np.arange(0,num_stims)+1)

        for stim in range(self.NUM_STIMS):
            self.summaryResultsPlot[stim].set_xdata(np.nan)
            self.summaryResultsPlot[stim].set_ydata(np.nan)

            for var in range(self.NUM_VARS):
                self.summaryResultsPlotErr[stim][var].set_xdata(np.nan)
                self.summaryResultsPlotErr[stim][var].set_ydata(np.nan)

            if stim+1 in p['stimChannels']:
                self.summaryResultsPlot[stim].set_visible(True)
            else:
                self.summaryResultsPlot[stim].set_visible(False)

        self.updatePlotLayouts()


    def pause(self):
        self.trialRunner._paused = not self.trialRunner._paused
        if self.trialRunner._paused:
            self.updateCommFeed('Paused', 'pc')
            self.sessionPause_pushButton.setText('Resume')
        else:
            self.updateCommFeed('Resumed', 'pc')
            self.sessionPause_pushButton.setText('Pause')

    def abort(self):
        if self.trialRunner._session_running:
            self.trialRunner._session_running = False
            # self.sessionEndGUI()
            self.updateCommFeed('Aborted', 'pc')
            self.sessionAbort_pushButton.setEnabled(False)
            self.forceReward_pushButton.setEnabled(False)

    def forceReward(self):
        if self.trialRunner._session_running is False:
            self.updateCommFeed('Force current reward only works while session is running', 'pc')
        if self.trialRunner._session_running is True:
            self.updateCommFeed('Forcing current reward', 'pc')
            write_string = '@R'  # arduino knows this means deliver reward
            self.updateCommFeed(write_string, 'pc')
            arduino['device'].write(write_string.encode('utf-8'))

    def updatePlotLayouts(self):
                
        self.preTrialRasterFigAx.set_xlim(0, np.max([1, p['witholdBeforeStimPlotVal']]))

        num_trials = p['sessionDuration']

        trial_num = self.trialRunner.trial_num

        self.updateResultBlockPlots(trial_num)

        if p['autoScalePlots']:
            if trial_num<20:
                trials_ax_lim = [0,20]
            else:
                if not self.trialRunner._session_running:
                    trials_ax_lim = [0, trial_num]
                    self.raster_current_trial.set_xdata(np.empty(0))
                    self.raster_current_trial.set_ydata(np.empty(0))
                else:
                    trials_ax_lim = [0, trial_num+2]
        else:
            trials_ax_lim = [0, num_trials]


        # response raster
        pre_stim_plot = 1
        self.rasterFigAx.set_xlim(-pre_stim_plot, p['trialDuration'])
        self.rasterFigAx.set_ylim(trials_ax_lim[0], trials_ax_lim[1])
        self.raster_respwin.set_x(p['responseStart'])
        self.raster_respwin.set_width(p['responseWindow'])
        self.raster_respwin.set_height(trials_ax_lim[1]+1)
        self.raster_stimline.set_ydata(trials_ax_lim)
        self.raster_stimlength.set_x(p['stimStart'])
        self.raster_stimlength.set_width(p['stimLength'])
        self.raster_stimlength.set_height(trials_ax_lim[1]+1)
        self.raster_autorewardline.set_ydata(trials_ax_lim)
        self.raster_autorewardline.set_xdata([p['autoRewardStart'], p['autoRewardStart']])
        self.raster_autorewardline.set_alpha(int(np.any(p['autoRewards'])))

        # reaction time plot
        self.rxn_respwin.set_x(p['responseStart'])
        self.rxn_respwin.set_width(p['responseWindow'])
        self.rxn_respwin.set_height(trials_ax_lim[1]+1)
        self.rxn_stimline.set_ydata(trials_ax_lim)
        self.rxn_stimlength.set_x(p['stimStart'])
        self.rxn_stimlength.set_width(p['stimLength'])
        self.rxn_stimlength.set_height(trials_ax_lim[1]+1)
        self.rxn_autorewardline.set_ydata(trials_ax_lim)
        self.rxn_autorewardline.set_xdata([p['autoRewardStart'], p['autoRewardStart']])
        self.rxn_autorewardline.set_alpha(int(np.any(p['autoRewards'])))

        # performance line
        if p['negativeMarking'] == 0:
            self.performanceFigAx.set_ylim([0, 1])
            self.performanceFigAx.set_ylabel('Correct (%)')
            self.performanceFigAx.set_yticks(np.linspace(0, 1, 11))
            self.performanceFigAx.set_yticklabels([0,'',20,'',40,'',60,'',80,'',100])

            self.summaryResultsAx.set_ylim([0, 1])
            self.summaryResultsAx.set_ylabel('Correct (%)')
            self.summaryResultsAx.set_yticks(np.linspace(0, 1, 11))
            self.summaryResultsAx.set_yticklabels([0,'',20,'',40,'',60,'',80,'',100])

            self.chance_line1.set_ydata([0.5,0.5])
            self.chance_line2.set_ydata([0.5,0.5])
            #self.summaryResultsIm.set_clim([0,1])
        else:
            self.performanceFigAx.set_ylim([-1, 1])
            self.performanceFigAx.set_ylabel('Score (%)')
            self.performanceFigAx.set_yticks(np.linspace(-1, 1, 9))
            self.performanceFigAx.set_yticklabels([-100, -75, -50, -25, 0, 25, 50, 75, 100])

            self.summaryResultsAx.set_ylim([-1, 1])
            self.summaryResultsAx.set_ylabel('Score (%)')
            self.summaryResultsAx.set_yticks(np.linspace(-1, 1, 9))
            self.summaryResultsAx.set_yticklabels([-100, -75, -50, -25, 0, 25, 50, 75, 100])

            self.chance_line1.set_ydata([0,0])
            self.chance_line2.set_ydata([0,0])
            #self.summaryResultsIm.set_clim([-1,1])
        self.performanceFigAx.set_xlim(trials_ax_lim[0], trials_ax_lim[1])
        self.chance_line1.set_xdata(trials_ax_lim)
        self.chance_line2.set_xdata([-1,16])

        # performance blocks
        num_max_variations = max(p['variations'])
        self.rasterFigPerfAxIm.set_extent(   [0,0.75, 0,(num_trials)/(trials_ax_lim[1])])
        self.rasterFigPerfAxVarIm.set_extent([0.85,1, 0,(num_trials)/(trials_ax_lim[1])])
        self.performanceFigPerfAxIm.set_extent([      0,(num_trials)/(trials_ax_lim[1]),0,1])

        self.rasterFigPerfAxVarIm.set_clim([0,num_max_variations-1])

        self.preTrialRasterFigCanvas.draw()
        self.rasterFigCanvas.draw()
        self.performanceFigCanvas.draw()

    def updateRasterPlotData(self, trial_num, val, ID, state, is_first_response):
        if state == 'INTRIAL':
            plot_series = self.raster_responses
            ax = self.rasterFigAx
            canvas = self.rasterFigCanvas
            x = val
            y = trial_num + 1

            # update plot_data dictionary record
            plot_data['offsets'] = np.append(plot_data['offsets'], [x,y])
        elif state == 'PRETRIAL':
            plot_series = self.preTrialRaster_responses
            ax = self.preTrialRasterFigAx
            canvas = self.preTrialRasterFigCanvas
            x = val
            y = 0
            if x > trials['results'][trial_num]['withold_req']:
                ax.set_xlim([0, x])
                canvas.draw()

        if is_first_response:
            new_size = 28
            # update reaction time histogram
            trials['reaction_time'][trial_num] = val
            stim_type = int(p['trialOrder'][trial_num])
            rxn_times = trials['reaction_time']
            rxn_times = rxn_times[p['trialOrder']==stim_type]
            counts,edges = np.histogram(rxn_times, bins=self.hist_bins)
            # self.reactionTimeHists[stim_type-1].set_xdata(edges[:-1])
            #self.reactionTimeHists[stim_type-1].set_ydata(counts)
            edges = np.hstack([np.array([0]), edges])
            counts = np.hstack([np.array([0]), counts, np.array([0])])
            self.reactionTimeHists[stim_type-1].set_xy(np.vstack([edges, counts]).T)
            self.reactionTimesAx.set_ylim([0, max(counts.max()+1, max(self.reactionTimesAx.get_ylim()))])
            # self.reactionTimesAx.draw_artist(self.reactionTimeHists[stim_type])
        else:
            new_size = 8

        # get old data
        old_xy      = plot_series.get_offsets()
        old_sizes   = plot_series.get_sizes()
        old_colours = plot_series.get_array()

        # make new data
        new_xy      = np.append(old_xy,      [x,y])
        new_sizes   = np.append(old_sizes,   new_size)
        new_colours = np.append(old_colours, ID)

        # set new data
        plot_series.set_offsets(new_xy)
        plot_series.set_sizes  (new_sizes)
        plot_series.set_array  (new_colours)

        # # update/draw plots
        ax.draw_artist(plot_series)
        canvas.update()
        canvas.flush_events()

    def summaryResults(self, trial_num):
        # get details
        score = np.copy(trials['running_score'][:trial_num+1])
        trial_order = p['trialOrder'][:trial_num+1]
        variations = p['trialVariations'][:trial_num+1]

        num_diff_stims = len(p['stimChannels'])
        num_max_variations = max(p['variations'])

        # initialise
        results = np.full([num_max_variations, num_diff_stims], np.nan)
        errs_hi = np.full([num_max_variations, num_diff_stims], np.nan)
        errs_lo = np.full([num_max_variations, num_diff_stims], np.nan)

        # compute results
        if not p['negativeMarking']:
            score[score<0] = 0
        for stim_idx,stim_num in enumerate(p['stimChannels']):
            for var in range(num_max_variations):
                values = score[(trial_order==stim_num) & (variations==var)]
                if len(values) > 0:
                    # calculations
                    count = np.float(sum(values==1))
                    ntrials = np.float(len(values))
                    mu = values.mean()
                    sd = values.std()
                    sem = sd / np.sqrt(ntrials)
                    ci_lo, ci_hi = proportion_confint(count, ntrials, alpha=0.1)

                    if ntrials < 5:
                        sd = 0
                        sem = 0
                        ci_lo = mu
                        ci_hi = mu

                    # mean
                    results[var, stim_idx] = mu

                    # confidence intervals
                    errs_lo[var, stim_idx] = ci_lo
                    errs_hi[var, stim_idx] = ci_hi

                    # # std dev
                    # errs_lo[var, stim_idx] = mu - sd
                    # errs_hi[var, stim_idx] = mu + sd

                    # # sem
                    # errs_lo[var, stim_idx] = mu - sem
                    # errs_hi[var, stim_idx] = mu + sem

        # update plot
        for stim_idx,stim_num in enumerate(p['stimChannels']):
            if num_max_variations > 1:
                x = np.arange(0,num_max_variations)
            else:
                x = [stim_idx]
                
            y = results[:,stim_idx]
            yerr_hi = errs_hi[:,stim_idx]
            yerr_lo = errs_lo[:,stim_idx]

            self.summaryResultsPlot[stim_num-1].set_ydata(y)
            self.summaryResultsPlot[stim_num-1].set_xdata(x)

            for erridx in range(len(y)):
                self.summaryResultsPlotErr[stim_num-1][erridx].set_ydata([yerr_lo[erridx], yerr_hi[erridx]])
                self.summaryResultsPlotErr[stim_num-1][erridx].set_xdata([x[erridx], x[erridx]])


    def updateResultBlockPlots(self, trial_num):
        # initialise
        stored_variations = self.live_trialvariations.get_array()[0]
        stored_trialorder = self.live_trialorder.get_array()[0]
        num_stims = len(np.unique(stored_trialorder))
        num_trials = len(stored_trialorder)

        if (trial_num > 0) and (trial_num <= num_trials):
            performance_record = np.full([num_trials, num_stims], np.nan)
            variation_record = np.array(stored_variations).reshape([-1,1])
            variation_record[trial_num:] = np.nan

            # build the performance record
            for t in range(trial_num):
                s = p['stimChannels'].index(stored_trialorder[t])
                performance_record[t,s] = trials['running_score'][t]
        
            # sort the raster plots?
            if p['sortPlots']:
                # sort the performance bar
                order = np.arange(num_trials)
                order[:trial_num+1] = np.lexsort([stored_variations[:trial_num+1], stored_trialorder[:trial_num+1]])

                # sort the response raster
                offsets = np.copy(plot_data['offsets'])
                y = np.copy(offsets[1::2]) -1  # trial number
                new_y = np.copy(y)
                for val in np.unique(y):
                    new_val = np.argwhere(order==val)
                    new_y[y==val] = new_val
                offsets[1::2] = new_y

            else:
                order = np.arange(num_trials)
                offsets = np.copy(plot_data['offsets'])
            
            self.rasterFigPerfAxIm.set_data(performance_record[order])
            self.rasterFigPerfAxVarIm.set_data(variation_record[order])

            self.raster_responses.set_offsets(offsets)

            if trial_num < num_trials:
                self.raster_current_trial.set_xdata(self.rasterFigAx.get_xlim())
                if p['sortPlots']:
                    self.raster_current_trial.set_ydata([np.argwhere(order==trial_num), np.argwhere(order==trial_num)])
                else:
                    self.raster_current_trial.set_ydata([np.argwhere(order==trial_num)+1, np.argwhere(order==trial_num)+1])

            # update the performance bar (never sorted)
            self.performanceFigPerfAxIm.set_data(np.rot90(performance_record))

    def updateRunningPerformancePlot(self, trial_num):
        if not self.trialRunner._session_running:
            trial_num -= 1

        ydata = np.full([trial_num+1], np.nan, dtype=np.float)
        moving_avg_size = int(p['slidingWindowSize'])
        running_score = np.copy(trials['running_score'])

        if p['negativeMarking'] == 0:
            running_score[running_score<0] = 0

        # make moving average
        for t in range(trial_num+1):
            if t >= moving_avg_size:
                ydata[t] = np.mean(running_score[t-moving_avg_size:t])
            else:
                ydata[t] = np.mean(running_score[:t+1])

        self.runningScorePlot.set_ydata(ydata)
        self.runningScorePlot.set_xdata(range(1, len(ydata)+1))

        self.averageScorePlot.set_ydata([np.nanmean(running_score), np.nanmean(running_score)])
        self.averageScorePlot.set_xdata([0, p['sessionDuration']])

    def setConnects(self):
        # add connects for control button clicks
        self.setDefaults_Button.clicked.connect(self.GUISave)
        self.saveAsPreset_Button.clicked.connect(self.saveAsPreset)
        self.arduinoConnectDisconnect_pushButton.clicked.connect(self.manualConnectDisconnect)
        self.testPin_Button.clicked.connect(self.testPin)
        self.begin_Button.clicked.connect(self.begin)
        self.sessionPause_pushButton.clicked.connect(self.pause)
        self.sessionAbort_pushButton.clicked.connect(self.abort)
        self.forceReward_pushButton.clicked.connect(self.forceReward)
        self.loadTrialOrder_pushButton.clicked.connect(self.loadTrialOrder)
        self.saveTrialOrder_pushButton.clicked.connect(self.saveTrialOrder)

        self.loadPreset_ComboBox.currentIndexChanged.connect(self.loadPreset)

        self.sessionTimer = QTimer(self)
        self.sessionTimer.timeout.connect(self.sessionTimerUpdate)
        self.trialRunner.arduino_connected_signal.connect(self.arduinoConnected)
        self.trialRunner.arduino_disconnected_signal.connect(self.arduinoDisconnected)
        self.trialRunner.response_signal.connect(self.updateRasterPlotData)
        self.trialRunner.results_signal.connect(self.updatePerformancePlots)
        self.trialRunner.trial_start_signal.connect(self.trialStartGUI)
        self.trialRunner.trial_end_signal.connect(self.trialStopGUI)
        self.trialRunner.session_end_signal.connect(self.sessionEndGUI)
        self.trialRunner.comm_feed_signal.connect(self.updateCommFeed)

        # auto add connects to update p and trial config plot whenever anything changes
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        groups = (self.session_GroupBox, self.arduino_GroupBox,
                  self.trial_GroupBox, self.plotOptions_groupBox)
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

    def manualConnectDisconnect(self):
        if arduino['connected'] is False:
            self.trialRunner.connectArduino()
        elif arduino['connected'] is True:
            self.trialRunner.disconnectArduino()

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
        self.arduinoConnectDisconnect_pushButton.setText('Disconnect')
        self.arduino_GroupBox.setStyleSheet('QGroupBox {\n    border: 1px solid rgb(225, 225, 225);\n    margin-top: 1.1em;\n   background-color: rgb(226, 255, 242);\n}\n\nQGroupBox::title {\n    subcontrol-origin: margin;\n}')

    def arduinoDisconnected(self):
        self.arduinoConnectDisconnect_pushButton.setText('Connect')
        self.arduino_GroupBox.setStyleSheet('QGroupBox {\n    border: 1px solid rgb(225, 225, 225);\n    margin-top: 1.1em;\n   background-color: rgb(255, 234, 238);\n}\n\nQGroupBox::title {\n    subcontrol-origin: margin;\n}')

    def makeTrialOrder(self):
        if not self.trialRunner._session_running:
            num_stims = len(p['stimChannels'])
            num_trials = p['sessionDuration']
            if num_stims > 0:
                if not p['stimOrder'] == 'External file':
                    # reset widgets that may have been disabled:
                    stimCheckBoxes = [self.stim1_CheckBox, self.stim2_CheckBox, self.stim3_CheckBox, self.stim4_CheckBox,
                                      self.stim5_CheckBox, self.stim6_CheckBox, self.stim7_CheckBox, self.stim8_CheckBox]
                    propSpinBoxes = [self.proportion1_doubleSpinBox, self.proportion2_doubleSpinBox, self.proportion3_doubleSpinBox, self.proportion4_doubleSpinBox,
                                     self.proportion5_doubleSpinBox, self.proportion6_doubleSpinBox, self.proportion7_doubleSpinBox, self.proportion8_doubleSpinBox]
                    varSpinBoxes = [self.variations1_spinBox, self.variations2_spinBox, self.variations3_spinBox, self.variations4_spinBox,
                                    self.variations5_spinBox, self.variations6_spinBox, self.variations7_spinBox, self.variations8_spinBox]
                    for i, check_box in enumerate(stimCheckBoxes):
                        check_box.setEnabled(True)
                        if check_box.isChecked():
                            propSpinBoxes[i].setEnabled(True)
                            varSpinBoxes[i].setEnabled(True)

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
        self.trialOrderAx.imshow([p['trialOrder']], interpolation='none', cmap=self.cmap_stims, vmin=1, vmax=8, aspect='auto')
        self.trialOrderAx2.imshow([p['trialVariations']], interpolation='none', cmap='gray', aspect='auto')
        self.trialOrderCanvas.draw()

    def saveTrialOrder(self):
        filepath = str(QFileDialog.getSaveFileName(self, 'Save trial sequence', trial_sequence_directory, 'Trial sequence (*.txt)')[0])
        arr = np.vstack((p['trialOrder'], p['trialVariations']))
        np.savetxt(filepath, arr, fmt='%1i', delimiter=',')

    def loadTrialOrder(self):
        filepath = str(QFileDialog.getOpenFileName(self, 'Load trial sequence', trial_sequence_directory, '*.txt')[0])
        if filepath:
            arr = np.genfromtxt(filepath, delimiter=',')
            p['trialOrder'] = arr[0]
            p['trialVariations'] = arr[1]
            self.plotTrialOrder()

            filename = os.path.splitext(os.path.basename(filepath))[0]
            self.stimOrder_ComboBox.setCurrentIndex(2)
            self.loadedTrialSequence_label.setText('(' + filename + ')')

            # set num trials
            self.sessionDuration_SpinBox.setValue(len(p['trialOrder']))

            # set the appropriate stim check boxes
            unique_stims = np.unique(p['trialOrder'])
            stimCheckBoxes = [self.stim1_CheckBox, self.stim2_CheckBox, self.stim3_CheckBox, self.stim4_CheckBox,
                              self.stim5_CheckBox, self.stim6_CheckBox, self.stim7_CheckBox, self.stim8_CheckBox]
            propSpinBoxes = [self.proportion1_doubleSpinBox, self.proportion2_doubleSpinBox, self.proportion3_doubleSpinBox, self.proportion4_doubleSpinBox,
                              self.proportion5_doubleSpinBox, self.proportion6_doubleSpinBox, self.proportion7_doubleSpinBox, self.proportion8_doubleSpinBox]
            varSpinBoxes = [self.variations1_spinBox, self.variations2_spinBox, self.variations3_spinBox, self.variations4_spinBox,
                             self.variations5_spinBox, self.variations6_spinBox, self.variations7_spinBox, self.variations8_spinBox]
            for i, check_box in enumerate(stimCheckBoxes):
                if i+1 in unique_stims:
                    check_box.setChecked(True)
                    propSpinBoxes[i].setEnabled(False)
                    varSpinBoxes[i].setEnabled(False)
                else:
                    check_box.setChecked(False)
                    check_box.setEnabled(False)

            self.getValues()

    def getValues(self):
        # extract gui values
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        groups = (self.session_GroupBox, self.arduino_GroupBox,
                  self.trial_GroupBox, self.plotOptions_groupBox)
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

        autoRewards = [p['autoReward1'], p['autoReward2'], p['autoReward3'], p['autoReward4'],
                       p['autoReward5'], p['autoReward6'], p['autoReward7'], p['autoReward8']]

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
        p['autoRewards'] = [autoRewards[idx-1] for idx in p['stimChannels']]
        p['proportions'] = [proportions[idx-1] for idx in p['stimChannels']]
        p['variations'] = [variations[idx-1] for idx in p['stimChannels']]

        if p['witholdBeforeStim']:
            p['witholdBeforeStimPlotVal'] = p['witholdBeforeStimDuration']
        else:
            p['witholdBeforeStimPlotVal'] = 0

        self.updateTrialTimings()

        # update gui
        if self._gui_ready:
            self.makeTrialOrder()
            self.updateTrialConfigPlot()
            self.updatePlotLayouts()

            # colour stim names if selected
            stimCheckBoxes = [self.stim1_CheckBox, self.stim2_CheckBox, self.stim3_CheckBox, self.stim4_CheckBox,
                              self.stim5_CheckBox, self.stim6_CheckBox, self.stim7_CheckBox, self.stim8_CheckBox]
            
            for idx, stimCheckBox in enumerate(stimCheckBoxes):
                colour = list(self.cmap_stims(idx))
                colour_string = 'rgb(' + str(int(colour[0]*255)) + ',' + str(int(colour[1]*255)) + ',' + str(int(colour[2]*255)) + ')'
                stimCheckBox.setStyleSheet('QCheckBox::indicator:checked { background-color:' + colour_string +
                    '; border: 1px solid #b1b1b1;}')

    def updateTrialTimings(self, for_plot=False):
        p['totalDuration'] = p['witholdBeforeStimPlotVal'] + p['startToStimDelay'] + \
                             p['postStimDelay'] + p['responseWindow'] + p['endOfTrialDelay']
        p['trialDuration'] = p['totalDuration'] - p['witholdBeforeStimPlotVal']
        p['trialCueStart'] = 0
        p['stimCueStart'] = 0  # time resets to zero after withold
        p['stimStart'] = p['startToStimDelay']
        p['stimStop'] = p['stimStart'] + p['stimLength']
        p['responseCueStart'] = p['startToStimDelay'] + p['postStimDelay']
        p['responseStart'] = p['startToStimDelay'] + p['postStimDelay']
        p['responseStop'] = p['responseStart'] + p['responseWindow']
        p['autoRewardStart'] = p['startToStimDelay'] + p['autoRewardDelay']

    def updateTrialConfigPlot(self):
        self.withold_line.set_width(p['witholdBeforeStimPlotVal'])

        self.duration_line.set_width(p['trialDuration'])
        self.duration_line.set_x(p['witholdBeforeStimPlotVal'])

        self.trial_cue_rectangle.set_width(p['cueTrial'] * 0.1)

        self.stim_cue_rectangle.set_width(p['cueStim'] * 0.1)
        self.stim_cue_rectangle.set_x(p['witholdBeforeStimPlotVal'])

        self.response_cue_rectangle.set_width(p['cueResponse'] * 0.1)
        self.response_cue_rectangle.set_x(p['witholdBeforeStimPlotVal'] + p['responseCueStart'])

        self.stim_rectangle.set_x(p['witholdBeforeStimPlotVal'] + p['stimStart'])
        self.stim_rectangle.set_width(p['stimLength'])

        self.resp_rectangle.set_x(p['witholdBeforeStimPlotVal'] + p['responseStart'])
        self.resp_rectangle.set_width(p['responseWindow'])

        self.reward_rectangle.set_width(np.any(p['autoRewards']) * p['rewardDuration'])
        self.reward_rectangle.set_x(p['witholdBeforeStimPlotVal'] + p['autoRewardStart'])

        self.trialConfigAx.set_xlim([0, p['totalDuration']])
        self.trialConfigCanvas.draw()

    def GUISave(self):
        widgets = (QComboBox, QCheckBox, QLineEdit, QSpinBox, QDoubleSpinBox)
        groups = (self.session_GroupBox, self.arduino_GroupBox, self.plotOptions_groupBox)
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
            groups = (self.session_GroupBox, self.arduino_GroupBox, self.plotOptions_groupBox)
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
        self.trialOrderAx = self.trialOrderFig.add_axes([0, 0.25, 1, 0.75])
        self.trialOrderAx2 = self.trialOrderFig.add_axes([0, 0, 1, 0.25])
        self.trialOrderAx.axis('off')
        self.trialOrderAx.hold(True)
        self.trialOrderAx2.axis('off')
        self.trialOrderAx2.hold(True)
        self.trialOrder_verticalLayout.addWidget(self.trialOrderCanvas)

        # trial order ribbon (results tab)
        self.trialOrderLiveTabFig = Figure()
        self.trialOrderLiveTabFig.set_facecolor('white')
        self.trialOrderLiveTabCanvas = FigureCanvas(self.trialOrderLiveTabFig)  # a canvas holds a figure
        self.trialOrderLiveTabCanvas.setFixedHeight(15)
        self.trialOrderLiveTabAx = self.trialOrderLiveTabFig.add_axes([0, 0.66, 1, 0.33])  # stim order
        self.trialOrderLiveTabAx2 = self.trialOrderLiveTabFig.add_axes([0, 0.33, 1, 0.33])  # var order
        self.trialOrderLiveTabAx3 = self.trialOrderLiveTabFig.add_axes([0, 0, 1, 0.33])  # progress
        self.trialOrderLiveTabAx.axis('off')
        self.trialOrderLiveTabAx.hold(True)
        self.trialOrderLiveTabAx2.axis('off')
        self.trialOrderLiveTabAx2.hold(True)
        self.trialOrderLiveTabAx3.axis('off')
        self.trialOrderLiveTabAx3.hold(True)

        self.trialNumMarkerIm = self.trialOrderLiveTabAx3.imshow(np.zeros([1,1]), vmin=-1, vmax=1, cmap='bwr', interpolation='none', aspect='auto')
        self.live_trialorder = self.trialOrderLiveTabAx.imshow(np.zeros([1,1]), interpolation='none', cmap=self.cmap_stims, vmin=1, vmax=8, aspect='auto')
        self.live_trialvariations = self.trialOrderLiveTabAx2.imshow(np.zeros([1,1]), interpolation='none', cmap='gray', aspect='auto')
        self.liveTab_VerticalLayout.addWidget(self.trialOrderLiveTabCanvas)

        # trial structure config
        self.trialConfigFig = Figure()
        self.trialConfigFig.set_facecolor('white')
        self.trialConfigCanvas = FigureCanvas(self.trialConfigFig)  # a canvas holds a figure
        self.trialConfigCanvas.setFixedHeight(50)
        self.trialConfigAx = self.trialConfigFig.add_axes([0, 0, 1, 1])
        self.trialConfigAx.axis('off')
        self.trialConfigAx.hold(True)

        # make the shapes
        self.withold_line = patches.Rectangle([0, 0], 0, 0.05, ec=[.8, .8, .8], linewidth=0, zorder=0, fill=None, hatch='||')
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
        self.trialConfigAx.add_patch(self.withold_line)

        # add to GUI
        self.advancedGroupBox_layout.addWidget(self.trialConfigCanvas)


        # pretrial response/withold plot
        self.preTrialRasterFig = Figure()
        self.preTrialRasterFigCanvas = FigureCanvas(self.preTrialRasterFig)  # a canvas holds a figure
        self.preTrialResponseVerticalLayout.addWidget(self.preTrialRasterFigCanvas)
        self.preTrialRasterFigAx = self.preTrialRasterFig.add_axes([0.15, 0.5, .72, 0.1])
        self.preTrialRasterFigAx.hold(True)
        self.preTrialRasterFigAx.axis('on')
        self.preTrialRasterFigAx.get_yaxis().set_visible(False)
        self.preTrialRasterFigAx.set_xlim([0,1])
        self.preTrialRasterFigAx.set_title('Pre-trial', loc='left')
        self.preTrialRaster_responses = self.preTrialRasterFigAx.scatter(np.empty(0), np.empty(0), c=np.empty(0), s=np.empty(0), edgecolor='', antialiased=True, clip_on=False, vmin=1, vmax=3, cmap=self.cmap_resps, zorder=9)
        self.preTrialRasterFigCanvas.setFixedHeight(50)
        self.preTrialRasterFig.set_facecolor('white')
        self.preTrialRasterFigCanvas.draw()


        # main response raster plot
        self.rasterFig = Figure()
        self.rasterFigCanvas = FigureCanvas(self.rasterFig)  # a canvas holds a figure
        self.resultsFigsHorizontalLayout.addWidget(self.rasterFigCanvas)
        self.rasterFigAx = self.rasterFig.add_axes([0.15, 0.25, 0.72, 0.7])
        self.rasterFig.set_facecolor('white')
        self.rasterFigAx.set_title('Responses', loc='left')
        self.rasterFigAx.hold(True)

        self.raster_respwin = patches.Rectangle([0, 0], 0, 0, fc=[0, 0, 0, 0.05], ec='none', zorder=1)
        self.raster_stimlength = patches.Rectangle([0, 0], 0, 0, fc=[0, 0, 0, 0.05], ec='none', zorder=3)
        self.rasterFigAx.add_patch(self.raster_respwin)
        self.rasterFigAx.add_patch(self.raster_stimlength)
        self.raster_stimline, = self.rasterFigAx.plot([0, 0], [0, 0], '-', c=[0.3, 0.3, 0.3], linewidth=2, clip_on=True, zorder=8)
        self.raster_autorewardline, = self.rasterFigAx.plot([0, 0], [0, 0], '-', c=[0, .75, .95], linewidth=2, clip_on=True, zorder=8)
        self.raster_responses = self.rasterFigAx.scatter(np.empty(0), np.empty(0), c=np.empty(0), s=np.empty(0), edgecolor='', antialiased=True, clip_on=True, vmin=1, vmax=3, cmap=self.cmap_resps, zorder=9)
        
        self.raster_current_trial, = self.rasterFigAx.plot([],[], 'y-', alpha=0.3, lw=5, clip_on=False)
        # self.raster_reward, = self.rasterFigAx.plot([], [], 'o', clip_on=False)  # currently unused
        # self.raster_punish, = self.rasterFigAx.plot([], [], 'o', clip_on=False)  # currently unused

        # self.rasterFigAx.set_xlabel('Time (s)')
        self.rasterFigAx.set_ylabel('Trial')
        self.rasterFigAx.get_xaxis().set_visible(False)

        self.rasterFigPerfAx = self.rasterFig.add_axes([0.88, 0.25, 0.1, 0.7])
        self.rasterFigPerfAx.axis('off')
        self.rasterFigPerfAx.set_ylim([0,1])
        self.rasterFigPerfAx.autoscale(False)
        self.rasterFigPerfAxIm = self.rasterFigPerfAx.imshow(np.full([1,1], np.nan), interpolation='none', aspect='auto', origin='lower', extent=[0, 0.8, 0, 1], vmin=-1, vmax=1, cmap=self.cmap_score)
        self.rasterFigPerfAxVarIm = self.rasterFigPerfAx.imshow(np.full([1,1], np.nan), interpolation='none', aspect='auto', origin='lower', extent=[0.85, 1, 0, 1], vmin=0, vmax=8, cmap='gray')


        # summary results 1 plot (reaction times on left)
        self.reactionTimesAx = self.rasterFig.add_axes([0.15, 0.1, 0.72, 0.12], sharex=self.rasterFigAx)
        self.rxn_respwin = patches.Rectangle([0, 0], 0, 0, fc=[0, 0, 0, 0.05], ec='none', zorder=1)
        self.rxn_stimlength = patches.Rectangle([0, 0], 0, 0, fc=[0, 0, 0, 0.05], ec='none', zorder=3)
        self.reactionTimesAx.add_patch(self.rxn_respwin)
        self.reactionTimesAx.add_patch(self.rxn_stimlength)
        self.rxn_stimline, = self.reactionTimesAx.plot([0, 0], [0, 0], '-', c=[0.3, 0.3, 0.3], linewidth=2, clip_on=True, zorder=8)
        self.rxn_autorewardline, = self.reactionTimesAx.plot([0, 0], [0, 0], '-', c=[0, .75, .95], linewidth=2, clip_on=True, zorder=8)
        self.reactionTimesAx.set_xlabel('Time (s)')
        self.reactionTimesAx.set_ylabel('Count')
        self.reactionTimesAx.set_ylim([0,5])
        self.reactionTimesAx.yaxis.grid(False)

        self.reactionTimeHists = []
        for sub_plot in range(self.NUM_STIMS):
            temp = patches.Polygon(np.empty([1,2]), closed=True, color=self.cmap_stims(sub_plot), alpha=0.5, lw=1)
            self.reactionTimeHists.append(temp)
            self.reactionTimesAx.add_patch(self.reactionTimeHists[-1])



        # results plot
        self.performanceFig = Figure()
        self.performanceFigCanvas = FigureCanvas(self.performanceFig)  # a canvas holds a figure
        self.performanceFigAx = self.performanceFig.add_axes([0.15, 0.55, 0.78, 0.35])
        self.resultsFigsHorizontalLayout.addWidget(self.performanceFigCanvas)
        self.chance_line1, = self.performanceFigAx.plot([0, 0], [0, 0], ':', c=[0.3, 0.3, 0.3], linewidth=1, zorder=6, clip_on=True)

        self.runningScorePlot, = self.performanceFigAx.plot(0, 0, 'k-', linewidth=2, aa=True, clip_on=False, zorder=20)
        self.averageScorePlot, = self.performanceFigAx.plot(0, 0, '-', c=[0.7, 0.7, 0.7], linewidth=3, aa=True, clip_on=True, zorder=19)

        self.performanceFigAx.set_xlabel('Trial')
        self.performanceFigAx.set_ylabel('Performance (%)')
        self.performanceFigAx.set_yticks(np.linspace(-1, 1, 9))
        self.performanceFigAx.set_yticklabels([-100, -75, -50, -25, 0, 25, 50, 75, 100])
        self.performanceFig.set_facecolor('white')

        self.performanceFigPerfAx = self.performanceFig.add_axes([0.15, 0.91, 0.78, 0.04])
        self.performanceFigPerfAx.axis('off')
        self.performanceFigPerfAx.set_title('Performance', loc='left')
        self.performanceFigPerfAx.autoscale(False)
        self.performanceFigPerfAx.set_ylim([0,1])
        self.performanceFigPerfAxIm = self.performanceFigPerfAx.imshow(np.full([1,1], np.nan), interpolation='none', aspect='auto', origin='lower', extent=[0, 1, 0, 1], vmin=-1, vmax=1, cmap=self.cmap_score)

        # summary results 2 plot (overall scores by stim type)
        self.summaryResultsAx = self.performanceFig.add_axes([0.15, 0.1, 0.78, 0.35])
        self.chance_line2, = self.summaryResultsAx.plot([0, 0], [0, 0], ':', c=[0.3, 0.3, 0.3], linewidth=1, zorder=6, clip_on=True)
        self.summaryResultsAx.set_ylabel('Score (%)')
        self.summaryResultsAx.set_xlabel('Stimulus type')
        self.summaryResultsAx.set_xlim([-1,1])
        #self.summaryResultsAx.set_xticklabels(['',1,''])
        self.summaryResultsAx.xaxis.grid(False)
        
        self.summaryResultsPlot = []
        self.summaryResultsPlotErr = []
        for sub_plot in range(self.NUM_STIMS):
            temp, = self.summaryResultsAx.plot(np.nan, np.nan, marker='o', lw=2, color=self.cmap_stims(sub_plot), clip_on=False, zorder=sub_plot+10)
            self.summaryResultsPlot.append(temp)

            # error bars
            errorbars = []
            for err in range(self.NUM_VARS):
                temp_err, = self.summaryResultsAx.plot(np.nan, np.nan, lw=2, color=self.cmap_stims(sub_plot), clip_on=True, zorder=sub_plot+10, alpha=0.5)
                errorbars.append(temp_err)
            self.summaryResultsPlotErr.append(errorbars)

        self.summaryResultsAx.set_ylim([0,1])


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
        self.reset()

    def closeEvent(self, event):
        self._exiting = True
        time.sleep(0.25)
        self.trialRunner.stop()
        self.trialThread.quit()
        event.accept()

    def sessionEndGUI(self):
        self.sessionTimer.stop()
        end_time_string = self.sessionTimer_label.text()
        self.sessionTimer_label.setText('<font color=''#ff0066''>' + end_time_string + '</font>')
        self.updateCommFeed('Finished', 'pc')
        self.updatePerformancePlots(self.trialRunner.trial_num)
        self.saveResults()
        self.saveMat()
        self.sessionAbort_pushButton.setEnabled(False)
        self.forceReward_pushButton.setEnabled(False)
        self.begin_Button.setEnabled(True)

    def trialStartGUI(self, trial_num):
        self.trialNum_label.setText(str(trial_num+1))
        self.trialType_label.setText(str(int(p['trialOrder'][trial_num])))
        self.trialVar_label.setText(str(int(p['trialVariations'][trial_num]+1)))
        self.updateCommFeed('\n')
        self.updateCommFeed('Trial ' + str(trial_num+1), 'trial')
        plots = [self.preTrialRaster_responses]
        for plot_series in plots:
            plot_series.set_offsets(np.empty(0))
            plot_series.set_array(np.empty(0))
        
        # make trial progress bar
        progress_bar = np.zeros([1, len(p['trialOrder'])])
        progress_bar[0,:trial_num] = 0.5
        progress_bar[0,trial_num] = 1
        self.trialNumMarkerIm.set_data(progress_bar)

        self.live_trialorder.set_data([p['trialOrder']])
        self.live_trialvariations.set_data([p['trialVariations']])
        self.live_trialvariations.set_clim([0, max(p['trialVariations'])])
        self.trialOrderLiveTabCanvas.draw()
        
        self.updatePlotLayouts()


    def trialStopGUI(self, trial_num):
        self.saveResults()

    def updatePerformancePlots(self, trial_num):
        self.summaryResults(trial_num)
        self.updateRunningPerformancePlot(trial_num)
        self.updatePlotLayouts()


    def updateCommFeed(self, input_string, device=None):
        # input_string = input_string.replace('<', '{').replace('>', '}')
        if device == 'pc':
            # self.toSessionFeed_textEdit.append(input_string)
            input_string = 'PC:      ' + input_string
        elif device == 'arduino':
            # self.fromSessionFeed_textEdit.append(input_string)
            input_string = 'ARDUINO: ' + input_string
        elif device == 'trial':
            # self.toSessionFeed_textEdit.setText(input_string)
            # self.fromSessionFeed_textEdit.setText('')
            input_string = input_string
        self.trial_log.append(input_string)
        print(input_string)

    def saveResults(self):
        #if not divmod(self.trialRunner.trial_num, 10)[1]:
        save_directory = os.path.join(results_directory, p['subjectID'])
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)
        save_name = os.path.join(save_directory, p['sessionID'])

        # save data to pickle
        with open(save_name + '.pkl', 'wb') as pkl:
            pickle.dump(trials, pkl, -1)
        self.updateCommFeed('Saved to ' + p['sessionID'] + '.pkl')
        
        # save figures to pdf (slow, take 0.5s to complete)
        with PdfPages(save_name + '.pdf') as pdf:
           pdf.savefig(self.rasterFig)
           pdf.savefig(self.performanceFig)

        # save terminal outputs to text file
        with open(save_name + '.txt', 'w') as log:
            log.write('\n'.join(self.trial_log))           

    def saveMat(self):
        save_directory = os.path.join(results_directory, p['subjectID'])
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)
        save_name = os.path.join(save_directory, p['sessionID'])

        sio.savemat(save_name + '.mat', trials)
        self.updateCommFeed('Saved to ' + p['sessionID'] + '.mat')


# Main entry to program.  Sets up the main app and create a new window.
def main(argv):
    # create Qt application
    app = QApplication(argv)
    # app.setStyle('fusion')

    # create main window
    GUI = MainWindow()

    # resize if larger than desktop
    screen_res = QDesktopWidget().availableGeometry()
    if GUI.frameSize().height() > screen_res.height():
        GUI.resize(GUI.frameSize().width(), screen_res.height())

    # centre the window
    height = screen_res.bottom() - screen_res.top()
    GUI.move((screen_res.width()/2) - (GUI.frameSize().width()/2),
             (height/2) - ((GUI.frameSize().height()+40)/2))  # +40, fudge for window borders

    # show it and bring to front
    GUI.show()
    GUI.raise_()

    # show the window icon
    if os.path.isfile(os.path.join('GUI', 'icon.ico')):
        GUI.setWindowIcon(QIcon(os.path.join('GUI', 'icon.ico')))

    # start the app
    sys.exit(app.exec_())

if __name__ == '__main__':
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
    global trials
    global arduino
    global p
    global plot_data

    trials = {}
    trials['results'] = np.full([1,1], np.nan)
    trials['running_score'] = np.full([1,1], np.nan)
    trials['reaction_time'] = np.full([1,1], np.nan)
    trials['correct_tally'] = 0  # not currently used, but will be used for auto incrementing

    plot_data = {}
    plot_data['offsets'] = []

    arduino = {}
    arduino['connected'] = False
    p = {}

    # start random number seed
    np.random.seed()

    # launch program
    try:
        main(sys.argv)
    except Exception as e:
        logger.exception(e)
