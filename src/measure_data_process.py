import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl, scipy
from scipy.fft import fft, fftfreq
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
import matplotlib.animation as animation
import csv, os
mpl.use('TkAgg')
plt.rcParams['axes.grid'] = True
# plt.rcParams['text.usetex'] = True

max_buffer_length = 16 * 65536

time = np.zeros(max_buffer_length)
angle = np.zeros(max_buffer_length)
pos = np.zeros(max_buffer_length)

# put your directory path here
dirc_path = r'C:\Programming\Python\Funky Pendulum_local\cart_pendulum_data\measure_test'

def damp_sin(time, gamma, omega, phi, amp, offset):
    return amp * np.exp(- 0.5 * gamma * time) * np.sin(omega * time + phi) + offset

def read_csv(filename):
    with open(dirc_path + '\\' +  filename, newline='') as csvfile:
        time = np.zeros(max_buffer_length)
        angle = np.zeros(max_buffer_length)
        pos = np.zeros(max_buffer_length)
        count = 0
        temp_time = 0.
        temp_index = 0
        buffer_length = 0
        flag_temp_index = True
        reader = csv.reader(csvfile)
        header_list = ['t', 's', 'o', 'p', 'a']
        for row in reader:

            if(row[0][0] in header_list):
                continue
            elif(row[0] == "0.0" and row[1] == "0.0" and row[2] == "0.0"):
                continue
            else:
                time[count], angle[count], pos[count] = \
                    float(row[0]), float(row[1]), float(row[2])
                if(flag_temp_index):
                    if(time[count] == 0):
                        continue
                    if(time[count] > temp_time):
                        temp_time = time[count]
                    elif(time[count] < temp_time):
                        temp_index = count - 1
                        flag_temp_index = False
                count += 1
                buffer_length = int((count) / 2)
    return time, angle, pos, temp_index, buffer_length

def peak(x, c):
    return np.exp(-np.power(x - c, 2) / 16.0)

def lin_interp(x, y, i, half):
    return x[i] + (x[i+1] - x[i]) * ((half - y[i]) / (y[i+1] - y[i]))

def half_max_x(x, y):
    half = max(y)/2
    signs = np.sign(np.add(y, -half))
    zero_crossings = (signs[0:-2] != signs[1:-1])
    zero_crossings_i = np.where(zero_crossings)[0]
    try:
        return [lin_interp(x, y, zero_crossings_i[0], half),
            lin_interp(x, y, zero_crossings_i[1], half)]
    except IndexError:
        print("Didn't find two zero crossings")
        return [0, 0]

def measure_fft(time, angle, buffer_length):
    fft_angle = fft(angle) / np.max(abs(fft(angle)))
    avg_spacing = (time[buffer_length - 1] - time[0]) / (len(time) - 1)
    fft_freq = fftfreq(len(time), avg_spacing)
    return abs(fft_angle) * abs(fft_angle), fft_freq, avg_spacing

def measure_fit_exp(time, angle):
    '''Fit the decaying exponential to the data'''
    popt, pcov = curve_fit(damp_sin, time, angle, 
                           p0 = [0.2, 2*np.pi*1.1, 0., 1.1, 0.], 
                           bounds = ((0.1, 2*np.pi*1., -np.pi, 0., -0.2), (1, 2*np.pi*1.2, np.pi, np.inf, 0.2)),
                           maxfev = 200000000)
    return popt, pcov

def measure_plot(save = False, filename = 'measure.csv', block_plot = True):
    time, angle, pos, temp_index, buffer_length = read_csv(filename)
    if(temp_index == 0):
        print("Error: no data with file" + filename)
        os.remove(dirc_path + '\\' +  filename)
    else:
        start_index = 50
        buffer_length -= 70
        low_ind = temp_index + 1 + start_index
        high_ind = temp_index + 1 + buffer_length
        fft_angle, fft_freq, avg_spacing = measure_fft(time[low_ind:high_ind], angle[low_ind:high_ind], buffer_length - start_index)
        figure, axes = plt.subplots(1, 2,  figsize = (10, 5))
        filename = os.path.splitext(filename)[0]
        window_name = filename
        figure.suptitle("sampling rate = " + str(1 / avg_spacing)[:4] + " Hz")
        figure.canvas.manager.set_window_title(window_name)
        line_ang, = axes[0].plot(time[low_ind:high_ind], angle[low_ind:high_ind], 'r-', markersize = 2)
        line_fft, = axes[1].plot(fft_freq[:int(len(fft_freq)/2)], abs(fft_angle[:int(len(fft_freq)/2)]), 'r-', markersize = 2)
        axes[1].set_xlim(-0.1, 2.5)
        axes[0].set_xlabel('Time/s')
        axes[0].set_ylabel('Angle/rad')
        axes[1].set_xlabel('Frequency/Hz')
        axes[1].set_ylabel('Arbitrary Unit')
        peaks, _ = find_peaks(abs(fft_angle[:int(len(fft_freq)/2)]), height = 0.9, width = 0)
        axes[1].plot(fft_freq[peaks], abs(fft_angle[peaks]), 'x')

        hmx = half_max_x(fft_freq[:int(len(fft_freq)/2)], abs(fft_angle[:int(len(fft_freq)/2)]))
        width = hmx[1] - hmx[0]

        for i,j in zip(peaks, abs(fft_angle[peaks])):
            axes[1].annotate(str(fft_freq[i])[:5] + "Hz", xy=(fft_freq[i], j))
            axes[1].text(1.25, 0.2, "resolution = " + str(1 / (time[high_ind - 1] - time[low_ind]))[:6] + " Hz")
            axes[1].text(1.25, 0.25, "2pi *width = " + str(width*2*np.pi)[:5] + " rad/s")
            axes[1].text(1.25, 0.3, "width = " + str(width)[:5] + " Hz")

        popt, pcov = measure_fit_exp(time[low_ind:high_ind], angle[low_ind:high_ind])
        axes[0].plot(time[low_ind:high_ind], damp_sin(time[low_ind:high_ind], *popt), 'b--', markersize = 2)

        axes[1].axhline(y = 1 / 2, color = 'k', linestyle = '--')
        axes[0].text(0.7,0.9, 'gamma = ' + str(popt[0])[:5] + " rad/s", horizontalalignment='center',
            verticalalignment='center', transform=axes[0].transAxes)
        axes[0].text(0.7,0.2, 'omega = ' + str(popt[1]/2/np.pi)[:5] + " Hz", horizontalalignment='center',
            verticalalignment='center', transform=axes[0].transAxes)
        q1 = fft_freq[peaks[0]] / width
        q2 = popt[1] / popt[0]
        axes[1].text(1.4, 1 / 2 - 0.06, 'Q1 = ' + str(q1)[:5])
        axes[0].text(0.7, 0.8, 'Q2 = ' + str(q2)[:5], horizontalalignment='center',
            verticalalignment='center', transform=axes[0].transAxes)
        print(popt, "\n", np.sqrt(np.diag(pcov)))
        plt.show(block = block_plot)
        return (*popt, *np.sqrt(np.diag(pcov)), fft_freq[peaks[0]], width)
        # TODO: return the frequency and damping factor and their errors

param_list = []

for filename in os.listdir(dirc_path):
    if(filename[0] == 'm'):
        # param_list.append(measure_plot(save = False, filename = filename))
        print('check')
        measure_plot(save = False, filename = filename)
        # pass
    elif(filename[0] == 'N'):
        # NR_plot_init(save = False, filename = filename)
        pass
    elif(filename[0] == 'p'):
        pass

# Put your parent path here
# param_path = r"C:\Programming\Python\Funky Pendulum\cart_pendulum_data\Natural_Freqency_Check"

# with open(param_path + '\\' + 'param.csv', 'w', newline = '') as param_file:
#     param_writer = csv.writer(param_file, delimiter = ',')
#     param_writer.writerow(['gamma', 'omega', 'phase', 'amp', 'offset', 
#                            'gamma_err', 'omega_err', 'phase_err', 'amp_err', 'offset_err',
#                            'freq', 'width'])
#     for param in param_list:
#         param_writer.writerow(param)
