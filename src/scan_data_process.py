'''Extract from NR file, need to determine the amplitude and the phase response
   This is in general the data analysis class, currently compatible with the 
   measure and scan data analysis'''
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import os, csv, scipy, statistics, tkinter
from statistics import mean, stdev
from scipy.fft import fft, fftfreq
from scipy.signal import find_peaks
from scipy.optimize import curve_fit
plt.rcParams['axes.grid'] = True
plt.rcParams["figure.autolayout"] = True
mpl.use('TkAgg')

def sinusoid(time, omega, phi, amp, offset):
    '''Fit to the amplitude response scan plot'''
    return amp * np.sin(2 * np.pi * omega * time + phi) + offset

def damp_sin(time, gamma, omega, phi, amp, offset):
    '''Fit to the natural frequency measurement plot'''
    return amp * np.exp(- 0.5 * gamma * time) * np.sin(2 * np.pi * omega * time + phi) + offset

class data_analysis():
    '''Analysis class to analyze the data'''
    def __init__(self):
        self.buffer_length = 16 * 65536 # The buffer length of the data
        self.dirc = os.getcwd() # Get the csv file directory, either by input
        self.csv_list = [] # A list of csv file names
        self.data_flag_dict = {
            'measure': False,
            'scan': False,
            'pid': False,
        }
        self.properties = {} # A dictionary of numbers read from the header
        self.header = [
            'special_info: ',
            'temp_index: ',
            'omega: ',
            'amplitude: ',
            'phase/pi: ',
            'time,angle,position,angular_velocity,cart_velocity',
            'Kp,Ki,Kd,Kp_pos,Ki_pos,Kd_pos',
        ]
        self.data = np.zeros((5,self.buffer_length), dtype = float)
        self.count = 0
        self.phase_list = []
        self.amp_list = []
        
    def clear_flag(self):
        '''Clear the flag'''
        for key in self.data_flag_dict:
            self.data_flag_dict[key] = False
    
    def clear_data(self):
        '''Clear the data'''
        self.csv_list = []
        self.properties = {}
        self.data = np.zeros((5,self.buffer_length), dtype = float)
        self.count = 0
        self.temp_data = np.zeros((5,self.buffer_length), dtype = float)
        self.phase_list = []
        self.amp_list = []
        
    def load_csv(self):
        '''Load the csv file'''
        self.clear_data()
        self.dirc = input('Please input the directory of the csv file: ')
        for file in os.listdir(self.dirc):
            if file.endswith('.csv'):
                self.csv_list.append(file)
            if file.startswith('measure'):
                self.data_flag_dict['measure'] = True
            if file.startswith('NR'):
                self.data_flag_dict['scan'] = True
            if file.startswith('pid'):
                self.data_flag_dict['pid'] = True
        if (len(self.csv_list) == 0):
            print('No csv file found in the directory!')
            return False
        else:
            return True
    
    def read_csv(self, file_name, flag_pid = False):
        '''Read a single csv file and return the properties and data'''
        path = self.dirc + '\\' + file_name
        self.path = path
        with open(path, 'r') as file:
            reader = csv.reader(file)
            try: 
                for index, row in enumerate(reader):
                    if(flag_pid):
                        if(index == 2):
                            self.properties.update({header:row[0]}) # Read the pid parameters
                            continue
                        if(row[0].startswith('Kp')):
                            continue
                        for header in self.header:
                            if(row[0].startswith(header)):
                                self.properties.update({header:row[0][len(header):]})
                                break
                        if(row[0].startswith('time')):
                            continue
                        if(index >= 8):
                            self.load_data(row, file)
                    else:
                        for header in self.header:
                            if(row[0].startswith(header)):
                                self.properties.update({header:row[0][len(header):]})
                                break
                        if(row[0].startswith('time')):
                            continue
                        if(index >= 6):
                            self.load_data(row, file)
            except ValueError:
                return False
        return True
    
    def check_csv_type(self):
        '''Check whether all the csv files are of the same type'''
        sum = 0
        for key in self.data_flag_dict:
            sum += self.data_flag_dict[key]
        if(sum == 0):
            return False
        elif(sum == 1):
            return True
        else:
            print('Multiple data type detected!')
            return False
    
    def load_data(self, row, file):
        '''Load a single row of data'''
        if(float(row[0]) < 0):
            print("Detect negative time stamp at " + self.path)
            file.close()
            os.remove(self.path)
            return
        if(row[0] != '0.0'):
            for i in range(5):
                self.data[i][self.count] = float(row[i])
            self.count += 1
                
    def clean_data(self, file):
        '''Returns an array with correct starting point'''
        temp_index = 0
        time_stamp = self.data[0][0]
        flag = True
        for i in range(len(self.data[0])):
            if(flag):
                if(self.data[0][i] >= time_stamp):
                    time_stamp = self.data[0][i]
                else:
                    flag = False
                    temp_index = i
                    time_stamp = self.data[0][i]
                    for j in range(len(self.data[0])):
                        self.data[0][j] -= time_stamp
                    break
                    
        if(temp_index == 0 and time_stamp == 0):
            print('Empty file found at ' + self.dirc + '\\' + file + " Deleting...")
            os.remove(self.path)
            raise FileNotFoundError
        return self.data[:, temp_index : temp_index + int(self.count / 2)]
    
    def restore_figure(self, start_index = 0, end_index = -1):
        '''Restore the figure'''
        print("Restoring figure...")
        self.figure, axes = plt.subplots(2, 2, figsize = (10, 6))
        self.scan_fft_plot((axes[0, 1], axes[1, 1]), start_index, end_index)
        try:
            self.figure.suptitle(self.properties['special_info: '])
        except KeyError:
            self.figure.suptitle('No special info')
        self.figure.canvas.manager.set_window_title(self.properties['file_name'])
        axes[0, 0].plot(self.temp_data[0][0:len(self.temp_data[0]):5], 
                        self.temp_data[1][0:len(self.temp_data[0]):5], 
                        'b-', 
                        label = 'angle_time')
        axes[0, 0].legend(loc = 'upper left')
        axes[1, 0].plot(self.temp_data[0][0:len(self.temp_data[0]):5], 
                        self.temp_data[2][0:len(self.temp_data[0]):5], 
                        'b-', 
                        label = 'position_time')
        axes[1, 0].legend(loc = 'upper left')
        return self.figure, axes
    
    def fft_index_list(self, time, fft_length, sampling_div):
        '''return the list and average time spacing'''
        current_time = time[len(time) - 1]
        time_stamp = current_time
        index = fft_length - 2
        index_list = np.zeros(fft_length, dtype = int)
        index_list[fft_length - 1] = current_time
        
        for i in range(len(time) - 1, 0, -1):
            if index < 0: 
                index_list = index_list
                avg_spacing = (current_time - time_stamp) / (fft_length - index - 2)
                return index_list, avg_spacing
            if(time_stamp - time[i] >= sampling_div):
                index_list[index] = i
                time_stamp = time[i]
                index -= 1
        if index >= 0:
            avg_spacing = (current_time - time_stamp) / (fft_length - index - 2)
            index_list = index_list[index + 1 : fft_length]
            return index_list, avg_spacing
        else:
            index_list = index_list
            avg_spacing = (current_time - time_stamp) / (fft_length - index - 2)
            return index_list, avg_spacing
    
    def general_fft(self, time, angle, position, fft_length, sampling_div):
        '''Return the power spectrum of the input'''
        index, avg = self.fft_index_list(time, fft_length, sampling_div)
        temp_angle = fft(angle[index])
        temp_pos = fft(position[index])
        fft_angle = temp_angle / np.max(abs(temp_angle))
        fft_position = temp_pos / np.max(abs(temp_pos))
        fft_freq = fftfreq(len(index), avg)
        return fft_angle, fft_position, fft_freq, avg
    
    def measure_fit(self, time, angle,
                    gamma_range = (0.01, 0.3),
                    omega_range = (2*np.pi*1., 2*np.pi*1.3),
                    phi_range = (-np.pi, np.pi),
                    amp_range = (0., np.inf),
                    offset_range = (-0.2, 0.2),
                    maxfev = 200000000):
        '''Fit the decaying sinusoidal exponential to the data'''
        popt, pcov = curve_fit(damp_sin, time, angle, 
                            p0 = [0.5*(gamma_range[0] + gamma_range[1]), 
                                  0.5*(omega_range[0] + omega_range[1]), 
                                  0.5*(phi_range[0] + phi_range[1]), 
                                  0.5*(amp_range[0] + amp_range[1]), 
                                  0.5*(offset_range[0] + offset_range[1])], 
                            bounds = ((gamma_range[0], omega_range[0], phi_range[0], amp_range[0], offset_range[0]), 
                                    (gamma_range[1], omega_range[1], phi_range[1], amp_range[1], offset_range[1])),
                                    maxfev = maxfev)
        return popt, pcov
    
    def measure_plot(self):
        pass
    
    def phase_rectify(self, phase):
        '''Shifts the phase to be between 0.5 * pi and -1.5*pi, which is symmetric abour -0.5*pi'''
        phase = phase - 2 * np.pi * int(phase / (2 * np.pi))
        if phase > 0.5 * np.pi:
            return phase - 2 * np.pi
        else:
            return phase
    
    def phase_calc(self, fft_freq, omega, fft_angle, fft_pos, interpolation = True):
        close_ind = np.argmin(np.abs(fft_freq - omega))
        if interpolation:
            if fft_freq[close_ind] < omega:
                delta_phase_1 = self.phase_rectify(np.angle(fft_angle[close_ind\
                    + 1]) - np.angle(fft_pos[close_ind + 1]) + np.pi)
                delta_phase_0 = self.phase_rectify(np.angle(fft_angle[close_ind])\
                    - np.angle(fft_pos[close_ind]) + np.pi)
                
                phase = delta_phase_0 + (omega - \
                    fft_freq[close_ind]) / (fft_freq[close_ind\
                        + 1] - fft_freq[close_ind]) * \
                            (delta_phase_1 - delta_phase_0)

            elif fft_freq[close_ind] > omega:
                delta_phase_1 =  self.phase_rectify(np.angle(fft_angle[close_ind\
                    - 1]) - np.angle(fft_pos[close_ind - 1]) + np.pi)
                delta_phase_0 = self.phase_rectify(np.angle(fft_angle[close_ind])\
                    - np.angle(fft_pos[close_ind]) + np.pi)
                
                phase = delta_phase_0 + (omega - \
                    fft_freq[close_ind]) / (fft_freq[close_ind\
                        - 1] - fft_freq[close_ind]) * \
                            (delta_phase_1 - delta_phase_0)

            else:
                phase = self.phase_rectify(np.angle(fft_angle[close_ind]) \
                    - np.angle(fft_pos[close_ind]) + np.pi)
        else:
            phase = self.phase_rectify(np.angle(fft_angle[close_ind]) \
                - np.angle(fft_pos[close_ind]) + np.pi)
        
        return phase / np.pi
     
    def scan_fit(self, time, angle,
                 amp_range):
        popt, pcov = curve_fit(sinusoid, time, angle,
                               p0 = [float(self.properties['omega: ']), np.pi, 
                                     0.5*(amp_range[0] + amp_range[1]), 0.],
                               bounds = ((0., 0, amp_range[0], -0.6),
                                         (4., 2 * np.pi, amp_range[1], 0.6)),
                               maxfev = 2000000000)
        return popt, pcov
    
    def scan_fft_plot(self, axs, start_index = 0, end_index = -1):
        
        '''Plot phase curve and fft on the axes objects'''
        for i in range(len(self.temp_data[0][start_index:end_index])):
            if(self.temp_data[0][i + start_index] - self.temp_data[0][0] \
                > self.fft_length * self.sampling_div):
                fft_angle, fft_position, fft_freq, avg = self.general_fft(
                    self.temp_data[0][:i + start_index],
                    self.temp_data[1][:i + start_index],
                    self.temp_data[2][:i + start_index],
                    self.fft_length,
                    self.sampling_div
                )
                phase = self.phase_calc(
                    fft_freq,
                    float(self.properties['omega: ']),
                    fft_angle,
                    fft_position
                )
                self.phase_list.append(phase)
                axs[1].plot(self.temp_data[0][i + start_index], phase, 'bo', markersize = 2)
        if(len(self.temp_data[0]) == 0):
            return
        fft_angle, fft_position, fft_freq, avg = self.general_fft(
            self.temp_data[0][start_index:end_index],
            self.temp_data[1][start_index:end_index],
            self.temp_data[2][start_index:end_index],
            self.fft_length,
            self.sampling_div
        )
        axs[0].plot(fft_freq[1:int(len(fft_freq)/2)], 
                    abs(fft_angle[1:int(len(fft_freq)/2)]), 
                    'b-', 
                    label = 'angle')
        axs[0].legend(loc = 'upper right')
        self.ax0.clear()
        self.ax0.plot(fft_freq[1:int(len(fft_freq)/2)], 
                 abs(fft_position[1:int(len(fft_freq)/2)]), 
                 'r-', 
                 label = 'position')
        self.ax0.legend(loc = 'right')
    
    def scan_process(self, axes, start_time, end_time, rolling_time):
        self.phase_list = []
        self.fft_length = int(rolling_time / self.sampling_div)
        if(self.temp_data[0][0] >= start_time):
            print("Invalid input of time range")
            return
        for i in range(len(self.temp_data[0])):
            if(self.temp_data[0][i] <= start_time):
                start_index = i
            if(self.temp_data[0][i] >= end_time):
                end_index = i
                break
            end_index = -1
        
        self.figure, axes = self.restore_figure(start_index, end_index)
        axes[0, 1].clear()
        axes[1, 1].clear()
        
        self.scan_fft_plot((axes[0, 1], axes[1, 1]), start_index, end_index)
        
        amp_ang_max = np.max(abs(self.temp_data[1][start_index:end_index]))
        amp_pos_max = np.max(abs(self.temp_data[2][start_index:end_index]))
        
        popt_angle, pcov_angle = self.scan_fit(self.temp_data[0][start_index:end_index], 
                                               self.temp_data[1][start_index:end_index],
                                               amp_range = (amp_ang_max - 0.5, amp_ang_max + 0.1))
        # be careful of the negative sign in the angle fit
        popt_position, pcov_position = self.scan_fit(self.temp_data[0][start_index:end_index],
                                                     self.temp_data[2][start_index:end_index],
                                                     amp_range = (amp_pos_max - 5, amp_pos_max + 5))
        axes[0, 0].plot(self.temp_data[0][start_index:end_index],
                        sinusoid(self.temp_data[0][start_index:end_index], *popt_angle),
                        'r--', label = 'best-fit-line')
        axes[0, 0].legend()
        axes[1, 0].plot(self.temp_data[0][start_index:end_index],
                        sinusoid(self.temp_data[0][start_index:end_index], *popt_position),
                        'r--', label = 'best-fit-line')
        axes[1, 0].legend()
        
        print('A_ang = %.4f ' % popt_angle[2] + u"\u00B1" + ' %.4f' % np.sqrt(pcov_angle[2, 2])\
            + " rad")
        print('A_pos = %.2f ' % popt_position[2] + u"\u00B1" + ' %.2f' % np.sqrt(pcov_position[2, 2])\
            + " steps")
        
        # phase calculation
        avg_phase = mean(self.phase_list)
        err_phase = stdev(self.phase_list)
        print('phase = %.4f ' % avg_phase + u"\u00B1" + ' %.4f' % err_phase + " pi")
        # / np.sqrt(len(self.phase_list) - 1)
        
        plt.show()
        self.ax0.clear()
        # self.figure, axes = self.restore_figure()
        return popt_angle[2], np.sqrt(pcov_angle[2, 2]), \
            popt_position[2], np.sqrt(pcov_position[2, 2]), \
                avg_phase, err_phase
          
    def scan_plot(self, file, block = True, save = False):
        '''Plot two graphs:
        1. The angle-time graph with best fit line and parameters
        2. The phase curve and cumulated error
        And save the timestamp, the amplitude of the best-fit, and the 
        phase with errors to a csv file or a temporary data structure'''
        self.temp_data = self.clean_data(file)
        # Default values the rolling fft
        self.fft_length = 512
        self.sampling_div = 0.05
        self.figure, axes = plt.subplots(2, 2, figsize = (10, 6))
        self.ax0 = axes[0, 1].twinx()
        self.scan_fft_plot((axes[0, 1], axes[1, 1]))
        try:
            self.figure.suptitle(self.properties['special_info: '])
        except KeyError:
            self.figure.suptitle('No special info')
        self.figure.canvas.manager.set_window_title(self.properties['file_name'])
        axes[0, 0].plot(self.temp_data[0][0:len(self.temp_data[0]):5], 
                        self.temp_data[1][0:len(self.temp_data[0]):5], 
                        'b-', 
                        label = 'angle_time')
        axes[0, 0].legend(loc = 'upper left')
        axes[1, 0].plot(self.temp_data[0][0:len(self.temp_data[0]):5], 
                        self.temp_data[2][0:len(self.temp_data[0]):5], 
                        'b-', 
                        label = 'position_time')
        axes[1, 0].legend(loc = 'upper left')
        plt.show(block = block)
        self.ax0.clear()
        flag_request = True
        while flag_request:
            try:
                start_time = float(input('Start time of calculation: '))
                end_time = float(input('End time of calculation: '))
                if(end_time > self.temp_data[0][-1]):
                    end_time = self.temp_data[0][-1]
                rolling_time = float(input('Rolling fft time: '))
                exp_data = self.scan_process(axes, start_time, end_time, rolling_time)
                msg = input('Do you want to save the data? (y to save, n to continue, r to adjust): ')
                flag_yn = True
                while flag_yn:
                    if(msg == 'y'):
                        flag_request = False
                        flag_yn = False
                        self.save_scan_data(exp_data, file)
                    elif(msg == 'n'):
                        flag_request = False
                        flag_yn = False
                    elif (msg == 'r'):
                        flag_yn = False
                    else:
                        msg = input('Please enter y or n: ')
            except (ValueError, AssertionError):
                print('Invalid input, please try again')

    def save_scan_data(self, exp_data, file):
        parent_dir = os.path.dirname(self.dirc)
        current_dir_name = os.path.split(self.dirc)[1]
        csv_dir = parent_dir + '\\scan_data.csv'
        flag = True
        
        if(os.path.isfile(csv_dir)):
            flag = False
        
        with open(csv_dir, 'a', newline = '') as csvfile:
            writer = csv.writer(csvfile)
            if(flag):
                writer.writerow(['file_name',
                                 'parent_dir',
                                 'driving_freq',
                                 'response_amp', 
                                 'response_amp_err', 
                                 'driving_amp', 
                                 'driving_amp_err', 
                                 'phase', 
                                 'phase_err'])
            writer.writerow([file,
                             current_dir_name,
                             float(self.properties['omega: ']), 
                             exp_data[0], exp_data[1], 
                             exp_data[2], exp_data[3], 
                             exp_data[4], exp_data[5]])
            csvfile.close()
    
    def main(self):
        self.load_csv()
        if(self.check_csv_type()):
            if(self.data_flag_dict['measure']):
                pass
            elif(self.data_flag_dict['scan']):
                for file in self.csv_list:
                    self.properties.update({'file_name':file})
                    print("processing " + file)
                    if(self.read_csv(file)):
                        self.scan_plot(file)
                        self.clear_data()
            elif(self.data_flag_dict['pid']):
                pass
        else:
            return
        
if __name__ == '__main__':
    data = data_analysis()
    data.main()
        
