import numpy as np
import matplotlib as mpl, matplotlib.pyplot as plt
import time, os, threading
# import modules from other python files
from data_process import data, live_data
from arduino_manager import arduino
from moment_data_process import data_frame
from Pendulum_Control_Console import cart_pendulum
plt.rcParams['axes.grid'] = True
plt.rcParams["figure.autolayout"] = True
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
mpl.use('TkAgg')
# Initialisation of some constants and variables
port = 'COM6' 
baudrate = 230400 # TODO: extract all constants from a larger project file?
MAX_COUNT = 10 # Number of points waited to plot a frame TODO: change this to manipulate the fps
ANGLE_ROTATION = 55 # Rotation of the y-label

if(__name__ == "__main__"):
    fft_lengths = 1024
    sampling_divs = 0.04
    wait_to_stables = 0
    #  Initialisation of the arduino board and the data class
    arduino_board = arduino(port, baudrate)
    df = data_frame()
    datum = data(fft_length = fft_lengths, 
                sampling_div = sampling_divs, 
                wait_to_stable = wait_to_stables)
    temp_datum = live_data(fft_length = fft_lengths, 
                sampling_div = sampling_divs, 
                wait_to_stable = wait_to_stables) # variable for thread plotting
    cartER = cart_pendulum(arduino_board, datum)
    
    # TODO: read reference parameters from a csv file and then implement cartER.main_auto_freq_scan()