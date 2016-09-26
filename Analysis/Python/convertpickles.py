import pickle
from scipy.io import savemat
import glob
import os
import time
import sys
try:
	# for Python2
	import Tkinter as tk
	import tkFileDialog as filedialog
except ImportError:
	# for Python3
	import tkinter as tk
	from tkinter import filedialog

def pickle2mat(filename, overwrite=False):
	matname = filename.replace('.pkl', '.mat')
	if not os.path.exists(matname) or (os.path.exists(matname) and overwrite):
		savemat(matname, pickle.load(open(filename, 'rb'), encoding='latin1'))

def update_progress(workdone):
	sys.stdout.write("\rProgress: [{0:50s}] {1:.1f}%".format('#' * int(workdone * 50), workdone*100))

if __name__ == '__main__':
	# select main directory
	root = tk.Tk()
	root.withdraw()
	directory = filedialog.askdirectory()

	# build file list
	filelist = glob.glob(os.path.join(directory, '**', '*.pkl'))
	num_files = len(filelist)

	# convert file list
	for f, filename in enumerate(filelist):
		print('[' + str(f+1) + ' of ' + str(num_files) + '] - ' + filename)
		#update_progress(f/num_files)
		pickle2mat(filename)

	print('Done')