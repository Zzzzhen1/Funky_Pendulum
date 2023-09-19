import numpy as np
import matplotlib as mpl, matplotlib.pyplot as plt
import time, os, threading, csv
from datetime import datetime
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
baudrate = 230400 # TODO: extract all constants from a project file?
MAX_COUNT = 10 # Number of points waited to plot a frame, used to set the fps
ANGLE_ROTATION = 55 # Rotation of the y-label

if(__name__ == "__main__"):
    fft_lengths = 1024
    sampling_divs = 0.04
    wait_to_stables = 1
    #  Initialisation of the arduino board and the data class
    arduino_board = arduino(port, baudrate)
    df = data_frame()
    datum = data(fft_length = fft_lengths, 
                sampling_div = sampling_divs, 
                wait_to_stable = wait_to_stables)
    temp_datum = live_data(fft_length = fft_lengths, 
                sampling_div = sampling_divs, 
                wait_to_stable = wait_to_stables) # variable for thread plotting
    cartER = cart_pendulum(arduino_board, datum, temp_datum, df)
    
    cartER.path = os.getcwd()
    
    refer_dirs = cartER.path + r'\auto_freq_scan'
    try:
        os.mkdir(refer_dirs)
    except OSError:
        pass
    # Trial run, comment this when executing the auto_scan
    # cartER.main_auto_freq_scan(
    #     auto_freq = 1.5,
    #     auto_amp = 100,
    #     duration = 60,
    # )
    
    # Initialisation of the auto_scan parameters
    start_freq = 0.9
    end_freq = 1.3
    num_freq = 20
    freq_array = np.linspace(start_freq, end_freq, num_freq)
    start_amp = 10
    end_amp = 200
    num_amp = 20
    amp_array = np.linspace(start_amp, end_amp, num_amp)
    duration = 1.8 * fft_lengths * sampling_divs
    ref_csv_dir = refer_dirs + r'\reference_parameters-' + \
        datetime.now().strftime("init-%d-%m.csv")
    with open(ref_csv_dir, 'w', newline = '') as f:
        writer = csv.writer(f)
        writer.writerow(['index' ,'start_time', 'freq', 'amp_0'])
        f.close()
    index = 0
    for freq in freq_array:
        for amp in amp_array:
            index += 1
            with open(ref_csv_dir, 'a', newline = '') as f:
                writer = csv.writer(f)
                writer.writerow([index, datetime.now().strftime("%d-%m-%Y %H:%M:%S"), freq, amp])
                f.close()
            time.sleep(0.1)
            cartER.main_auto_freq_scan(
                auto_freq = freq,
                auto_amp = amp,
                duration = duration,
            )
    
    
    # TODO: read reference parameters from a csv file and then implement cartER.main_auto_freq_scan()