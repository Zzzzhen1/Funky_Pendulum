import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time, os, csv
from datetime import datetime
from scipy.fft import fft, fftfreq
from scipy.optimize import curve_fit
plt.rcParams['axes.grid'] = True
plt.rcParams["figure.autolayout"] = True
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
mpl.use('TkAgg')

# Initialisation of some constants and variables
port = 'COM6' 
baudrate = 230400 
MAX_COUNT = 10 # Number of points waited to plot a frame
ANGLE_ROTATION = 55 # Rotation of the y-label

class data_phy():
    '''Put all the physics in this class so that people can look at it'''
    def __init__(
        self,
        fft_length,
        sampling_div,
        wait_to_stable,
        buffer_length = 4 * 8192, # Total number of points stored in the circular buffer
        plot_length = 64, # (Related to) Number of points plotted on the graph
        ):
        self.start_time = 0. # Arduino internal time might not start at zero
        self.sampling_div = sampling_div
        self.avg_spacing = 0. # Average time spacing between the data points
        self.time = np.zeros(2 * buffer_length)
        self.angle = np.zeros(2 * buffer_length)
        self.angular_velocity = np.zeros(2 * buffer_length)
        self.position = np.zeros(2 * buffer_length)
        self.position_velocity = np.zeros(2 * buffer_length)
        self.omega = 2. # driven frequency in Hz
        self.amp = 100. # amplitude of the active driven force
        self.amp_0 = 50.0 # This is used to characterise the constant oscillation
        self.phase = 0. 
        self.NR_Kp = 0.05 # Proportional control of the NR
        self.NR_Kd = 0. # Derivative control of the NR
        self.NR_Ki = 0. # NOT IMPLEMENTED YET
        self.fft_angle = np.zeros(fft_length)
        self.fft_pos = np.zeros(fft_length)
        self.fft_freq = np.zeros(fft_length)
        self.buffer_length = buffer_length
        self.fft_length = fft_length
        self.plot_length = plot_length
        self.index_list = np.zeros(fft_length, dtype = int) # List of indices used for fft
        self.phase_list = [(0., 0.)] * self.plot_length * 10 * (wait_to_stable + 1) # List of phase values
        self.amp_list = [(0., 0.)] * self.plot_length * 10 # List of amplitude values
        self.wait_to_stable = wait_to_stable # NR stage update rate.
        self.index = 0
        self.temp_index = 0
        self.counter = 0
        self.flag_fig_init = True
        self.flag_subplot_init = True
        self.flag_close_event = False
        self.module_name = ""
        self.path = ""
        self.omega_num = 0
        self.omega_list = None
        self.multi_phase_list = None
        self.pos_const = None
        self.pos_active = None
        self.setSpeed_param = None
        self.phase_list_active = None
  
    def fft_index_list(self):
        '''Since the sampled data might not be evenly spaced, we need to find the
        almost evenly spaced data points (spacing indicated by self.sampling_div) 
        to do the fft. This function returns the a list of indices and the average 
        spacing between the data points.'''
        current_time = self.time[self.temp_index + self.buffer_length]
        # Use time_stamp to keep track of the previous recorded time and its index
        time_stamp = current_time
        index = self.fft_length - 2
        index_list = np.zeros(self.fft_length, dtype = int)
        index_list[self.fft_length - 1] = current_time # The last element is the current time, choose as default
        
        # Search for data points that are at least self.sampling_div apart
        for i in range(self.temp_index + self.buffer_length, self.temp_index + 1, -1):
            if index < 0: 
                self.index_list = index_list
                # Calculate the average spacing between the data points, since the data points might 
                # not be evenly spaced
                avg_spacing = (current_time - time_stamp) / (self.fft_length - index - 2)
                return self.index_list, avg_spacing
            if(time_stamp - self.time[i] >= self.sampling_div):
                index_list[index] = i
                time_stamp = self.time[i]
                index -= 1
        if index >= 0:
            avg_spacing = (current_time - time_stamp) / (self.fft_length - index - 2)
            self.index_list = index_list[index + 1 : self.fft_length]
            return self.index_list, avg_spacing
    
    def fft(self):
        '''Does the fft when there are enough data points. Returns True if the fft
        is done, False otherwise.'''
        if(self.time[self.temp_index] > 5 * self.sampling_div):
            index_list, avg_spacing = self.fft_index_list()
            self.avg_spacing = avg_spacing
            
            fft_ang = fft(self.angle[index_list])
            fft_pos = fft(self.position[index_list])
            if(self.pos_const is not None):
                fft_pos_const = fft(self.pos_const[index_list])
            if(self.pos_active is not None):
                fft_pos_active = fft(self.pos_active[index_list])
            fft_freq = fftfreq(len(index_list), avg_spacing)
            
            self.fft_angle = fft_ang[1:int(len(fft_freq) / 2)]
            self.fft_pos = fft_pos[1:int(len(fft_freq) / 2)]
            if(self.pos_const is not None):
                self.fft_pos_const = fft_pos_const[1:int(len(fft_freq) / 2)]
            if(self.pos_active is not None):
                self.fft_pos_active = fft_pos_active[1:int(len(fft_freq) / 2)]
                
            # The frequency array is symmetric about zero, so we only need the positive part
            self.fft_freq = fft_freq[1:int(len(fft_freq) / 2)]
            return True
        else:
            return False
    
    def NR_phase_calc(self, omega, scan, interpolation = True):
        '''Calculates the phase with linear interpolation since the desired frequency
        might not be in the fft_freq array. Returns True if the phase is calculated,
        False otherwise.'''
        if (self.fft()):
            close_ind = np.argmin(np.abs(self.fft_freq - omega))
            if(not scan):
                if interpolation:
                    if self.fft_freq[close_ind] < omega:
                        delta_phase_1 = self.phase_rectify(np.angle(self.fft_angle[close_ind\
                            + 1]) - np.angle(self.fft_pos_const[close_ind + 1]) + np.pi)
                        delta_phase_0 = self.phase_rectify(np.angle(self.fft_angle[close_ind])\
                            - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                        
                        delta_phase_3 = self.phase_rectify(np.angle(self.fft_pos_active[close_ind\
                            + 1]) - np.angle(self.fft_pos_const[close_ind + 1]) + np.pi)
                        delta_phase_2 = self.phase_rectify(np.angle(self.fft_pos_active[close_ind])\
                            - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                        # Linear interpolation
                        self.phase = delta_phase_0 + (omega - \
                            self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                                + 1] - self.fft_freq[close_ind]) * \
                                    (delta_phase_1 - delta_phase_0)
                                    
                        self.phase_active = delta_phase_2 + (omega - \
                            self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                            + 1] - self.fft_freq[close_ind]) * \
                            (delta_phase_3 - delta_phase_2)

                    elif self.fft_freq[close_ind] > omega:
                        delta_phase_1 =  self.phase_rectify(np.angle(self.fft_angle[close_ind\
                            - 1]) - np.angle(self.fft_pos_const[close_ind - 1]) + np.pi)
                        delta_phase_0 = self.phase_rectify(np.angle(self.fft_angle[close_ind])\
                            - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                        
                        delta_phase_3 =  self.phase_rectify(np.angle(self.fft_pos_active[close_ind\
                            - 1]) - np.angle(self.fft_pos_const[close_ind - 1]) + np.pi)
                        delta_phase_2 = self.phase_rectify(np.angle(self.fft_pos_active[close_ind])\
                            - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                        # Linear interpolation
                        self.phase = delta_phase_0 + (omega - \
                            self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                                - 1] - self.fft_freq[close_ind]) * \
                                    (delta_phase_1 - delta_phase_0)
                                    
                        self.phase_active = delta_phase_2 + (omega - \
                            self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                            - 1] - self.fft_freq[close_ind]) * \
                            (delta_phase_3 - delta_phase_2)

                    else:
                        self.phase = self.phase_rectify(np.angle(self.fft_angle[close_ind]) \
                            - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                        self.phase_active = self.phase_rectify(np.angle(self.fft_pos_active[close_ind]) \
                            - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                else:
                    self.phase = self.phase_rectify(np.angle(self.fft_angle[close_ind]) \
                        - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                    self.phase_active = self.phase_rectify(np.angle(self.fft_pos_active[close_ind]) \
                        - np.angle(self.fft_pos_const[close_ind]) + np.pi)
                self.phase_list.pop(0)
                self.phase_list.append((self.time[self.temp_index], self.phase / np.pi))
                self.phase_list_active.pop(0)
                self.phase_list_active.append((self.time[self.temp_index], self.phase_active / np.pi))
                return True
            else:
                if interpolation:
                    if self.fft_freq[close_ind] < omega:
                        delta_phase_1 = self.phase_rectify(np.angle(self.fft_angle[close_ind\
                            + 1]) - np.angle(self.fft_pos[close_ind + 1]) + np.pi)
                        delta_phase_0 = self.phase_rectify(np.angle(self.fft_angle[close_ind])\
                            - np.angle(self.fft_pos[close_ind]) + np.pi)
                        # Linear interpolation
                        self.phase = delta_phase_0 + (omega - \
                            self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                                + 1] - self.fft_freq[close_ind]) * \
                                    (delta_phase_1 - delta_phase_0)

                    elif self.fft_freq[close_ind] > omega:
                        delta_phase_1 =  self.phase_rectify(np.angle(self.fft_angle[close_ind\
                            - 1]) - np.angle(self.fft_pos[close_ind - 1]) + np.pi)
                        delta_phase_0 = self.phase_rectify(np.angle(self.fft_angle[close_ind])\
                            - np.angle(self.fft_pos[close_ind]) + np.pi)
                        # Linear interpolation
                        self.phase = delta_phase_0 + (omega - \
                            self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                                - 1] - self.fft_freq[close_ind]) * \
                                    (delta_phase_1 - delta_phase_0)

                    else:
                        self.phase = self.phase_rectify(np.angle(self.fft_angle[close_ind]) \
                            - np.angle(self.fft_pos[close_ind]) + np.pi)
                else:
                    self.phase = self.phase_rectify(np.angle(self.fft_angle[close_ind]) \
                        - np.angle(self.fft_pos[close_ind]) + np.pi)
                if(self.omega_list is None):
                    self.phase_list.pop(0)
                    self.phase_list.append((self.time[self.temp_index], self.phase / np.pi))
                return True
        else:
            return False
        
    def NR_update(self, scan = False, interpolation = True, manual = True):
        '''Calculates multiple phases at this function, or returns the amp and 
        phase feed back. Needs to be called frequently to update the plot for 
        the phase and amplitude'''
        if(self.omega_list is None):
            if (self.NR_phase_calc(self.omega, scan, interpolation)):
                if scan:
                    return 0., 0.
                else:
                    if(manual):
                        self.amp_list.pop(0)
                        self.amp_list.append((self.time[self.temp_index], self.amp))
                        return self.amp, self.phase
                    else:
                        # This is for the automatic finding of normalised resonance
                        delta_amp_Kp = self.NR_Kp * ((self.phase + np.pi / 2))/ (2 * np.pi)
                        try:
                            delta_amp_Kd = self.NR_Kd * (self.phase_list[-1][1] - self.phase_list[-2][1]) / \
                                (self.phase_list[-1][0] - self.phase_list[-2][0])
                        except RuntimeError:
                            pass
                        self.amp *= (1 - delta_amp_Kp) * (1 - delta_amp_Kd)
                        
                        self.amp_list.pop(0)
                        self.amp_list.append((self.time[self.temp_index], self.amp))
                        return self.amp, self.phase
            else:
                return 0, 0
        else:
            for index, omega in enumerate(self.omega_list):
                if(self.NR_phase_calc(omega, interpolation)):
                    self.multi_phase_list[index].pop(0)
                    self.multi_phase_list[index].append((self.time[self.temp_index], self.phase / np.pi))
            return 0., 0.
    
    def phase_rectify(self, phase):
        '''Shifts the phase to be between 0.5 * pi and -1.5*pi, which is symmetric abour -0.5*pi'''
        phase = phase - 2 * np.pi * int(phase / (2 * np.pi))
        if phase > 0.5 * np.pi:
            return phase - 2 * np.pi
        elif phase <= -1.5 * np.pi:
            return phase + 2 * np.pi
        else:
            return phase
    
    def delay_fit(self, low, high):
        '''Find the delay time between the two waves in the freq_scan module'''
        delay_time = 0.
        
        def delay_func(time, delay):
            return self.amp_0 * np.sin(2 * np.pi * self.omega * (time + self.start_time + delay))
        
        popt, pcov = curve_fit(delay_func, 
                               self.time[low : high], 
                               self.position[low : high],
                               p0 = 0.007,
                               maxfev = 20000)
        # the idea here is that the proposed position of the cart at this moment 
        # is the position of the cart at the next moment
        delay_time = popt[0]
        delay_error = np.sqrt(np.diag(pcov))[0]
        return delay_time, delay_error
    
class data(data_phy):
    
    '''Initialisation of the data class, used to store the data from the arduino
    and plot the graph with blocking'''
    
    def __init__(
        self,
        fft_length,
        sampling_div,
        wait_to_stable = 1,
        buffer_length = 4 * 8192,
        plot_length = 64,
        ):
        super().__init__(fft_length, sampling_div, wait_to_stable, buffer_length, plot_length)
    
    def append_data(
        self,
        data_frame,
        appendPos = True,
        appendVel = False
    ):  
        '''Appends the data from the arduino to the circular buffer'''
        if(self.index == 0):
            self.start_time = data_frame.time
            self.sys_start_time = time.time()
        temp_index = self.index % self.buffer_length
        self.time[temp_index] = data_frame.time - self.start_time
        self.time[temp_index + self.buffer_length] = data_frame.time - self.start_time
        self.angle[temp_index] = data_frame.angle
        self.angle[temp_index + self.buffer_length] = data_frame.angle
        if(appendPos):
            self.position[temp_index] = data_frame.position
            self.position[temp_index + self.buffer_length] = data_frame.position
        if(appendVel):
            self.angular_velocity[temp_index] = data_frame.angular_velocity
            self.angular_velocity[temp_index + self.buffer_length] = data_frame.angular_velocity
            self.position_velocity[temp_index] = data_frame.position_velocity
            self.position_velocity[temp_index + self.buffer_length] = data_frame.position_velocity
        self.index += 1
        self.temp_index = temp_index
    
    def clear_data(self):
        '''Clears the data in the circular buffer, standard routine'''
        self.time = np.zeros(2 * self.buffer_length)
        self.angle = np.zeros(2 * self.buffer_length)
        self.angular_velocity = np.zeros(2 * self.buffer_length)
        self.position = np.zeros(2 * self.buffer_length)
        self.position_velocity = np.zeros(2 * self.buffer_length)
        self.index = 0
        self.temp_index = 0
        self.counter = 0
        self.fft_angle = np.zeros(self.fft_length)
        self.fft_pos = np.zeros(self.fft_length)
        self.fft_freq = np.zeros(self.fft_length)
        self.amp = 100.
        self.phase = 0.
        self.omega = 2.
        self.avg_spacing = 0.
        self.phase_list = [(0., 0.)] * self.plot_length * (self.wait_to_stable + 1) * 10
        self.amp_list = [(0., 0.)] * self.plot_length * 10
        self.index_list = np.zeros(self.fft_length, dtype = int)
        self.omega_num = 0
        self.omega_list = None
        self.multi_phase_list = None
        self.pos_const = None
        self.pos_active = None
        self.setSpeed_param = None
        self.phase_list_active = None
        
    def clear_figure(self):
        '''Clears the figure, standard routine'''
        plt.close("all")
        self.flag_fig_init = True
        self.flag_subplot_init = True
        self.flag_close_event = False
    
    def init_plot(self, module_name, scan = True):
        '''Initialises the plot in terms of different stages'''
        if(self.flag_fig_init):
            self.flag_fig_init = False
            plt.ion() # Turn on interactive mode, important for the non-blocking plot
            if(module_name == "measure"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(1, 2, figsize = (8, 5))
                    self.figure.suptitle('Measure')
                    self.flag_subplot_init = False
                self.line_angle, = self.ax_list[0].plot([], [], 'b-')
                self.line_fft, = self.ax_list[1].plot([], [], 'b-')
                
                # Initiate a new dictionary for all the artists objects
                self.ax_new_list = {self.ax_list[0]: self.line_angle, 
                                    self.ax_list[1]: self.line_fft}
                    
                self.ax_list[0].set_xlabel('Time/s')
                self.ax_list[0].set_ylabel('Angle/rad')
                self.ax_list[1].set_xlabel('Frequency/Hz')
                self.ax_list[1].set_xlim(0, self.omega)
                self.ax_list[1].set_ylabel('Arbitrary Unit')
                
            elif(module_name == "NR"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(2, 2, figsize=(8, 5))
                    self.flag_subplot_init = False
                    self.figure.suptitle('NR')
                    if(self.omega_list is None):
                        self.phase_list = [(0., 0.)] * self.plot_length * (self.wait_to_stable + 1) * 10
                    else: 
                        self.phase_list = None
                        self.multi_phase_list = []
                        for i in range(self.omega_num):
                            self.multi_phase_list.append([(0., 0.)] * self.plot_length* (self.wait_to_stable + 1) * 10)
                    self.amp_list = [(0., 0.)] * self.plot_length * 10
                    if(not scan):
                        self.phase_list_active = [(0., 0.)] * self.plot_length * (self.wait_to_stable + 1) * 10
                self.line_angle, = self.ax_list[0, 0].plot([], [], 'b-')
                self.line_pos, = self.ax_list[1, 0].plot([], [], 'r-')
                self.line_pos_const, = self.ax_list[1, 0].plot([], [], 'g--')
                self.line_fft_ang, = self.ax_list[0, 1].plot([], [], 'b-', label = 'angle')
                ax1 = self.ax_list[0, 1].twinx()
                self.line_fft_pos, = ax1.plot([], [], 'r-', label = 'position')
                if(self.omega_list is None):
                    self.line_phase, = self.ax_list[1, 1].plot([], [], 'b-', label = 'phase')
                else:
                    self.line_phase_list = []
                    for i in range(self.omega_num):
                        _line, = self.ax_list[1, 1].plot([], [], color = colors[i], 
                                                                      label = '%.3f Hz'%(self.omega_list[i]))
                        self.line_phase_list.append(_line)
                if(not scan):
                    self.line_phase_active, = self.ax_list[1, 1].plot([], [], 'r-', label = 'phase_active')
                ax2 = self.ax_list[1, 1].twinx()
                self.line_amp, = ax2.plot([], [], 'k-', label = 'amplitude')
                
                self.ax_list[0, 1].legend(loc = 'upper left')
                self.ax_list[1, 1].legend(loc = 'upper left')
                ax1.legend(loc = 'upper right')
                if(self.omega_list is None):
                    ax2.legend(loc = 'lower left')
                ax1.grid(False)
                ax2.grid(False)
                # Initiate a new dictionary for all the artists objects
                if(self.omega_list is None):
                    self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                        self.ax_list[1, 0]: (self.line_pos, self.line_pos_const),
                                        self.ax_list[0, 1]: self.line_fft_ang,
                                        ax1: self.line_fft_pos,
                                        ax2: self.line_amp,
                                        self.ax_list[1, 1]: self.line_phase,}
                else:
                    self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                        self.ax_list[1, 0]: (self.line_pos, self.line_pos_const),
                                        self.ax_list[0, 1]: self.line_fft_ang,
                                        ax1: self.line_fft_pos,
                                        ax2: self.line_amp,
                                        self.ax_list[1, 1]: self.line_phase_list,
                                        }
                
                if(not scan):
                    self.ax_new_list.update({self.ax_list[1, 1]: (self.line_phase, self.line_phase_active)})
                
                self.ax_list[0, 0].set_xlabel('Time/s')
                self.ax_list[0, 0].set_ylabel('Angle/rad')
                self.ax_list[1, 0].set_xlabel('Time/s')
                self.ax_list[1, 0].set_ylabel('Position/steps')
                self.ax_list[0, 1].set_xlabel('Frequency/Hz')
                if(self.omega_list is None):
                    self.ax_list[0, 1].set_xlim(0, self.omega)
                else:
                    self.ax_list[0, 1].set_xlim(0, self.omega_list[-1])
                self.ax_list[0, 1].set_ylabel('Arbitrary Unit')
                self.ax_list[1, 1].set_xlabel('Time/s')
                self.ax_list[1, 1].set_ylabel('Phase/pi')
                ax2.set_ylabel('Amplitude/steps')
            
            elif(module_name == "freq_scan" or module_name == "auto_freq_scan"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(2, 2, figsize=(8, 5))
                    self.flag_subplot_init = False
                    self.figure.suptitle('NR')
                    if(self.omega_list is None):
                        self.phase_list = [(0., 0.)] * self.plot_length * (self.wait_to_stable + 1) * 10
                    else: 
                        self.phase_list = None
                        self.multi_phase_list = []
                        for i in range(self.omega_num):
                            self.multi_phase_list.append([(0., 0.)] * self.plot_length* (self.wait_to_stable + 1) * 10)
                    self.amp_list = [(0., 0.)] * self.plot_length * 10
                    if(not scan):
                        self.phase_list_active = [(0., 0.)] * self.plot_length * (self.wait_to_stable + 1) * 10
                self.line_angle, = self.ax_list[0, 0].plot([], [], 'b-')
                self.line_pos, = self.ax_list[1, 0].plot([], [], 'r-')
                self.line_pos_const, = self.ax_list[1, 0].plot([], [], 'g--')
                self.line_fft_ang, = self.ax_list[0, 1].plot([], [], 'b-', label = 'angle')
                ax1 = self.ax_list[0, 1].twinx()
                self.line_fft_pos, = ax1.plot([], [], 'r-', label = 'position')
                if(self.omega_list is None):
                    self.line_phase, = self.ax_list[1, 1].plot([], [], 'b-', label = 'phase')
                else:
                    self.line_phase_list = []
                    for i in range(self.omega_num):
                        _line, = self.ax_list[1, 1].plot([], [], color = colors[i], 
                                                                      label = '%.3f Hz'%(self.omega_list[i]))
                        self.line_phase_list.append(_line)
                if(not scan):
                    self.line_phase_active, = self.ax_list[1, 1].plot([], [], 'r-', label = 'phase_active')
                ax2 = self.ax_list[1, 1].twinx()
                self.line_amp, = ax2.plot([], [], 'k-', label = 'amplitude')
                
                self.ax_list[0, 1].legend(loc = 'upper left')
                self.ax_list[1, 1].legend(loc = 'upper left')
                ax1.legend(loc = 'upper right')
                if(self.omega_list is None):
                    ax2.legend(loc = 'lower left')
                ax1.grid(False)
                ax2.grid(False)
                # Initiate a new dictionary for all the artists objects
                if(self.omega_list is None):
                    self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                        self.ax_list[1, 0]: (self.line_pos, self.line_pos_const),
                                        self.ax_list[0, 1]: self.line_fft_ang,
                                        ax1: self.line_fft_pos,
                                        ax2: self.line_amp,
                                        self.ax_list[1, 1]: self.line_phase,}
                else:
                    self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                        self.ax_list[1, 0]: (self.line_pos, self.line_pos_const),
                                        self.ax_list[0, 1]: self.line_fft_ang,
                                        ax1: self.line_fft_pos,
                                        ax2: self.line_amp,
                                        self.ax_list[1, 1]: self.line_phase_list,
                                        }
                
                if(not scan):
                    self.ax_new_list.update({self.ax_list[1, 1]: (self.line_phase, self.line_phase_active)})
                
                self.ax_list[0, 0].set_xlabel('Time/s')
                self.ax_list[0, 0].set_ylabel('Angle/rad')
                self.ax_list[1, 0].set_xlabel('Time/s')
                self.ax_list[1, 0].set_ylabel('Position/steps')
                self.ax_list[0, 1].set_xlabel('Frequency/Hz')
                if(self.omega_list is None):
                    self.ax_list[0, 1].set_xlim(0, self.omega)
                else:
                    self.ax_list[0, 1].set_xlim(0, self.omega_list[-1])
                self.ax_list[0, 1].set_ylabel('Arbitrary units')
                self.ax_list[1, 1].set_xlabel('Time/s')
                self.ax_list[1, 1].set_ylabel('Phase/pi')
                ax2.set_ylabel('Amplitude/steps')
            
            elif(module_name == "pid"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(2, 2, figsize=(8, 5))
                    self.figure.suptitle('PID')
                    self.flag_subplot_init = False
                self.line_angle, = self.ax_list[0, 0].plot([], [], 'b-')
                self.line_pos, = self.ax_list[1, 0].plot([], [], 'r-')
                self.line_angle_vel, = self.ax_list[0, 1].plot([], [], 'b-')
                self.line_pos_vel, = self.ax_list[1, 1].plot([], [], 'r-')
                
                # Initiate a new dictionary for all the artists objects
                self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                    self.ax_list[1, 0]: self.line_pos,
                                    self.ax_list[0, 1]: self.line_angle_vel,
                                    self.ax_list[1, 1]: self.line_pos_vel}
                
                self.ax_list[0, 0].set_xlabel('Time/s')
                self.ax_list[0, 0].set_ylabel('Angle/rad')
                self.ax_list[1, 0].set_xlabel('Time/s')
                self.ax_list[1, 0].set_ylabel('Position/steps')
                self.ax_list[0, 1].set_xlabel('Time/s')
                self.ax_list[0, 1].set_ylabel('Angular Velocity/(rad/s)')
                self.ax_list[1, 1].set_xlabel('Time/s')
                self.ax_list[1, 1].set_ylabel('Cart Velocity/(steps/s)')
                
            elif(module_name == "setSpeed"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(1, 2, figsize=(8, 5))
                    self.figure.suptitle('setSpeed')
                    self.flag_subplot_init = False
                self.line_pos, = self.ax_list[0].plot([], [], 'r-')
                self.line_pos_vel, = self.ax_list[1].plot([], [], 'r-')
                
                # Initiate a new dictionary for all the artists objects
                self.ax_new_list = {self.ax_list[0]: self.line_pos,
                                    self.ax_list[1]: self.line_pos_vel}
                
                self.ax_list[0].set_xlabel('Time/s')
                self.ax_list[0].set_ylabel('Position/steps')
                self.ax_list[1].set_xlabel('Time/s')
                self.ax_list[1].set_ylabel('Cart Velocity/(steps/s)')
                
            # Configure the events
            self.figure.canvas.mpl_connect('close_event', self.handle_close)
            self.figure.canvas.manager.set_window_title(module_name)
            self.figure.canvas.draw_idle()
            plt.tight_layout()
            plt.show(block = False)

    def real_time_plot(self, module_name, scan = False):
        '''Plots the data in real time, non-blocking. Can be improved by combining the 
        similar parts in the if and else statements.'''
        self.module_name = module_name
        if(module_name == "measure"):
            self.fft()
            if(self.index < self.plot_length * 8):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind], 
                                        self.angle[low_ind:high_ind])
                    self.line_fft.set_data(self.fft_freq, 
                                         abs(self.fft_angle))
                    try:
                        txt1 = self.ax_list[1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                        txt2 = self.ax_list[1].text(0.5, 1.1, 'resolution: ' + str(round(1 / len(self.index_list) / self.avg_spacing,3)) + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                            
                    self.ax_list[1].set_xlim(0, 2 * self.omega)
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                    except UnboundLocalError:
                        pass
                        
                self.counter += 1
                
            else:
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.temp_index + 1 + self.buffer_length - self.plot_length * 8
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind], 
                                        self.angle[low_ind:high_ind])
                    self.line_fft.set_data(self.fft_freq, 
                                         abs(self.fft_angle))
                    try: 
                        txt1 = self.ax_list[1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                        txt2 = self.ax_list[1].text(0.5, 1.1, 'resolution: ' + str(round(1 / len(self.index_list) / self.avg_spacing,3)) + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                            
                    self.ax_list[1].set_xlim(0, 2 * self.omega)
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
        
        elif(module_name == "freq_scan" or module_name == "auto_freq_scan"):
            self.fft()
            self.pos_const = self.amp_0 * np.sin(2 * np.pi * \
                self.omega * (self.time + self.start_time))
            delay_time, delay_error = 0., 0.
            if(self.index < self.plot_length):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_fft_ang.set_data(self.fft_freq, 
                                                abs(self.fft_angle))
                    self.line_fft_pos.set_data(self.fft_freq,
                                               abs(self.fft_pos))
                    
                    if(self.omega_list is None):
                        if(self.index > 20 and scan):
                            delay_time, delay_error = self.delay_fit(low_ind, high_ind)
                        self.line_pos_const.set_data(self.time[low_ind:high_ind], 
                                                     self.pos_const[low_ind:high_ind])
                        self.line_phase.set_data(*zip(*self.phase_list))
                    else:
                        for index, line in enumerate(self.line_phase_list):
                            line.set_data(*zip(*self.multi_phase_list[index]))
                    self.line_amp.set_data(*zip(*self.amp_list))
                    
                    if(not scan):
                        self.line_phase_active.set_data(*zip(*self.phase_list_active))
                    
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.03, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        txt2 = self.ax_list[0, 1].text(0.5, 1.12, 'resolution: ' + str(round(1 / len(self.index_list) / self.avg_spacing,3)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        # if(self.index > 20 and scan):
                        #     txt3 = self.ax_list[1, 0].text(0.1, 0.1, 'delay time: ' + str(1000*delay_time)[:6] + 'ms' \
                                # + u"\u00B1" + str(1000*delay_error)[:5] + 'ms', transform = self.ax_list[1, 0].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    if(self.omega_list is None):
                        self.figure.suptitle(module_name + ' Driving Freq: ' + str(self.omega) + 'Hz')
                    else:
                        self.figure.suptitle(module_name + ' Driving Freq: ' + ', '.join("%.3f" % i for i in self.omega_list) + 'Hz')
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                        
                    if(self.omega_list is None):
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega)
                    else:
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega_list[-1])
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                        # txt3.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
                
            else:
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.temp_index + 1 + self.buffer_length - self.plot_length
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_fft_ang.set_data(self.fft_freq, 
                                                abs(self.fft_angle))
                    self.line_fft_pos.set_data(self.fft_freq,
                                               abs(self.fft_pos))
                    
                    if(self.omega_list is None):
                        if(scan):
                            delay_time, delay_error = self.delay_fit(low_ind, high_ind)
                        self.line_phase.set_data(*zip(*self.phase_list))
                        self.line_pos_const.set_data(self.time[low_ind:high_ind], 
                                                     self.pos_const[low_ind:high_ind])
                    else:
                        for index, line in enumerate(self.line_phase_list):
                            line.set_data(*zip(*self.multi_phase_list[index]))
                    self.line_amp.set_data(*zip(*self.amp_list))
                    
                    if(not scan):
                        self.line_phase_active.set_data(*zip(*self.phase_list_active))
                    
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        txt2 = self.ax_list[0, 1].text(0.5, 1.1, 'resolution: ' + str(round(1 / len(self.index_list) / self.avg_spacing,3)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        # if(scan):
                        #     txt3 = self.ax_list[1, 0].text(0.1, 0.1, 'delay time: ' + str(1000*delay_time)[:6] + 'ms' \
                                # + u"\u00B1" + str(1000*delay_error)[:5] + 'ms', transform = self.ax_list[1, 0].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    if(self.omega_list is None):
                        self.figure.suptitle(module_name + ' Driven Freq: ' + str(self.omega) + 'Hz')
                    else:
                        self.figure.suptitle(module_name + ' Driven Freq: ' + ', '.join("%.3f" % i for i in self.omega_list) + 'Hz')
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                
                    if(self.omega_list is None):
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega)
                    else:
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega_list[-1])
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                        # txt3.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
                
        elif(module_name == "NR"):
            self.fft()
            self.pos_const = self.amp_0 * np.sin(2 * np.pi * \
                self.omega * (self.time + self.start_time))
            self.pos_active = self.position - self.pos_const
            delay_time, delay_error = 0., 0.
            if(self.index < self.plot_length):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_fft_ang.set_data(self.fft_freq, 
                                                abs(self.fft_angle))
                    self.line_fft_pos.set_data(self.fft_freq,
                                               abs(self.fft_pos))
                    
                    if(self.omega_list is None):
                        if(self.index > 20 and scan):
                            delay_time, delay_error = self.delay_fit(low_ind, high_ind)
                        self.line_pos_const.set_data(self.time[low_ind:high_ind], 
                                                     self.pos_const[low_ind:high_ind])
                        self.line_phase.set_data(*zip(*self.phase_list))
                    else:
                        for index, line in enumerate(self.line_phase_list):
                            line.set_data(*zip(*self.multi_phase_list[index]))
                    self.line_amp.set_data(*zip(*self.amp_list))
                    
                    if(not scan):
                        self.line_phase_active.set_data(*zip(*self.phase_list_active))
                    
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        txt2 = self.ax_list[0, 1].text(0.5, 1.1, 'resolution: ' + str(round(1 / len(self.index_list) / self.avg_spacing,3)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    if(self.omega_list is None):
                        self.figure.suptitle(module_name + ' Driven Freq: ' + str(self.omega) + 'Hz')
                    else:
                        self.figure.suptitle(module_name + ' Driven Freq: ' + ', '.join("%.3f" % i for i in self.omega_list) + 'Hz')
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                        
                    if(self.omega_list is None):
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega)
                    else:
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega_list[-1])
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
                
            else:
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.temp_index + 1 + self.buffer_length - self.plot_length
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_fft_ang.set_data(self.fft_freq, 
                                                abs(self.fft_angle))
                    self.line_fft_pos.set_data(self.fft_freq,
                                               abs(self.fft_pos))
                    
                    if(self.omega_list is None):
                        if(scan):
                            delay_time, delay_error = self.delay_fit(low_ind, high_ind)
                        self.line_phase.set_data(*zip(*self.phase_list))
                        self.line_pos_const.set_data(self.time[low_ind:high_ind], 
                                                     self.pos_const[low_ind:high_ind])
                    else:
                        for index, line in enumerate(self.line_phase_list):
                            line.set_data(*zip(*self.multi_phase_list[index]))
                    self.line_amp.set_data(*zip(*self.amp_list))
                    
                    if(not scan):
                        self.line_phase_active.set_data(*zip(*self.phase_list_active))
                    
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        txt2 = self.ax_list[0, 1].text(0.5, 1.1, 'resolution: ' + str(round(1 / len(self.index_list) / self.avg_spacing,3)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    if(self.omega_list is None):
                        self.figure.suptitle(module_name + ' Driven Freq: ' + str(self.omega) + 'Hz')
                    else:
                        self.figure.suptitle(module_name + ' Driven Freq: ' + ', '.join("%.3f" % i for i in self.omega_list) + 'Hz')
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                
                    if(self.omega_list is None):
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega)
                    else:
                        self.ax_list[0, 1].set_xlim(0, 2 * self.omega_list[-1])
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
                
        elif(module_name == "pid"):
            if(self.index < self.plot_length and self.index > 1):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_angle_vel.set_data(self.time[low_ind:high_ind],
                                            self.angular_velocity[low_ind:high_ind])
                    self.line_pos_vel.set_data(self.time[low_ind:high_ind],
                                            self.position_velocity[low_ind:high_ind])
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    if(self.pid_param == 'r'):
                        self.figure.suptitle('PID parameters (reusing previous values)')
                    else:
                        self.figure.suptitle('PID parameters(' + self.pid_param + ')')
                    
                    for axes in self.ax_list:
                        for ax in axes:
                            ax.relim()
                            ax.autoscale_view()
                            for label in ax.get_yticklabels():
                                label.set_rotation(ANGLE_ROTATION)
                        
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
                
            else:
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.temp_index + 1 + self.buffer_length - self.plot_length
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_angle_vel.set_data(self.time[low_ind:high_ind],
                                            self.angular_velocity[low_ind:high_ind])
                    self.line_pos_vel.set_data(self.time[low_ind:high_ind],
                                            self.position_velocity[low_ind:high_ind])
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.05, 'sampling rate: ' + str(round(0.5 / self.avg_spacing,1)) + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    if(self.pid_param == 'r'):
                        self.figure.suptitle('PID parameters (reusing previous values)')
                    else:
                        self.figure.suptitle('PID parameters(' + self.pid_param + ')')
                    
                    for axes in self.ax_list:
                        for ax in axes:
                            ax.relim()
                            ax.autoscale_view()
                            for label in ax.get_yticklabels():
                                label.set_rotation(ANGLE_ROTATION)
                        
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1            
        
        elif(module_name == "setSpeed"):
            if(self.index < self.plot_length):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                             self.position[low_ind:high_ind])
                    self.line_pos_vel.set_data(self.time[low_ind:high_ind],
                                               self.position_velocity[low_ind:high_ind])
                    
                    if(self.setSpeed_param is not None):
                        self.figure.suptitle(self.setSpeed_param)
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                            
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    
                self.counter += 1
            else:
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.temp_index + self.buffer_length + 1 - self.plot_length
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                             self.position[low_ind:high_ind])
                    self.line_pos_vel.set_data(self.time[low_ind:high_ind],
                                               self.position_velocity[low_ind:high_ind])
                    
                    if(self.setSpeed_param is not None):
                        self.figure.suptitle(self.setSpeed_param)
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(ANGLE_ROTATION)
                            
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    
                self.counter += 1
        
    def handle_close(self, _):
        '''Turns the original close event to save the figure as well'''
        self.flag_close_event = True
        self.flag_subplot_init = True
            
        try:
            dirc = self.path + '\\' + datetime.now().strftime("%d-%m-pdf")
            os.makedirs(dirc)
        except OSError:
            pass
        filename = dirc + '\\' + self.module_name + \
            datetime.now().strftime("-%H-%M-%S") + ".pdf"
        self.figure.savefig(filename, dpi = 600)
        self.figure.clf()
        self.figure.canvas.flush_events()
        plt.close("all")

    def export_csv(
        self, 
        module_name,
        NR_phase_amp = False,
        input_spec_info = True,
        ):
        '''Exports the data to a csv file'''
        try:
            dirc = self.path + '\\' + datetime.now().strftime("%d-%m-csv")
            dirc_fft = self.path + '\\' + datetime.now().strftime("%d-%m-fft-csv")
            if(NR_phase_amp):
                dirc_phase_amp = self.path + '\\' + datetime.now().strftime("%d-%m-phase_amp-csv")
                os.makedirs(dirc_phase_amp)
            os.makedirs(dirc)
            os.makedirs(dirc_fft)
        except OSError:
            pass
        filename = dirc + '\\' + module_name + \
            datetime.now().strftime("-%H-%M-%S")
        filename_fft = dirc_fft + '\\' + "fft-" + module_name + \
            datetime.now().strftime("-%H-%M-%S")
        try:
            filename_phase_amp = dirc_phase_amp + '\\phase_amp-' + module_name + \
                datetime.now().strftime("-%H-%M-%S")
        except UnboundLocalError:
            pass
        special_info = ""
        with open(filename + '.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if(input_spec_info):
                special_info = input("Title to add to the csv file (if any)\n\n")
            writer.writerow(["special_info", special_info])
            if(module_name == "pid"):
                try:
                    writer.writerow(["Kp", "Ki", "Kd", "Kp_pos", "Ki_pos", "Kd_pos"])
                    writer.writerow([self.pid_param.split(',')[i] for i in range(6)])
                except (AttributeError, IndexError):
                    pass
            writer.writerow(["start_time", str(self.start_time)])
            if(self.omega_list is None):
                writer.writerow(["omega", str(self.omega)])
            else:
                writer.writerow(["multiple_omega", *(str(i) for i in self.omega_list)])
            try:
                writer.writerow(["amplitude", str(self.amp_list[-1][1]), "amp_0", str(self.amp_0)])
                if(self.omega_list is None):
                    writer.writerow(["phase/pi", str(self.phase_list[-1][1])])
                else:
                    writer.writerow(["multiple_phase/pi", *(str(i[-1][1]) for i in self.multi_phase_list)])
            except (AttributeError, IndexError):
                pass
            writer.writerow(["time", "angle", "position", "angular_velocity", "cart_velocity"])
            for i in range(len(self.time)):
                writer.writerow([self.time[i], self.angle[i], self.position[i],\
                    self.angular_velocity[i], self.position_velocity[i]])
            csvfile.close()
        if(module_name != "pid" and module_name != "setSpeed"):
            with open(filename_fft + '.csv', 'w', newline = '') as csvfile:
                writer = csv.writer(csvfile)
                if(input_spec_info):
                    special_info = input("\nAny special info to add to the fft-csv file?\n\n")
                writer.writerow(["special_info", special_info])
                writer.writerow(["start_time", str(self.start_time)])
                if(self.omega_list is None):
                    writer.writerow(["omega", str(self.omega)])
                else:
                    writer.writerow(["multiple_omega", *(str(i) for i in self.omega_list)])
                try:
                    writer.writerow(["amplitude", str(self.amp_list[-1][1])])
                    if(self.omega_list is None):
                        writer.writerow(["phase/pi", str(self.phase_list[-1][1])])
                    else:
                        writer.writerow(["multiple_phase/pi", *(str(i[-1][1]) for i in self.multi_phase_list)])
                except (AttributeError, IndexError):
                    pass
                writer.writerow(['freq', 'fft_angle', 'fft_position'])
                for i in range(len(self.fft_freq)):
                    writer.writerow([self.fft_freq[i], self.fft_angle[i], self.fft_pos[i]])
                csvfile.close()
                
        if(NR_phase_amp):
            # Since the phases have more points than the amplitudes, we need to align them
            with open(filename_phase_amp + ".csv", 'w', newline = '') as csvfile:
                writer = csv.writer(csvfile)
                if(input_spec_info):
                    special_info = input("\nAny special info to add to the phase_amp-csv file?\n\n")
                writer.writerow(["special_info", special_info])
                writer.writerow(["start_time", str(self.start_time)])
                writer.writerow(["omega", str(self.omega)])
                writer.writerow(["NR_Kp", "NR_Ki", "NR_Kd"])
                writer.writerow([str(self.NR_Kp), str(self.NR_Ki), str(self.NR_Kd)])
                if(self.phase_list_active is not None):
                    writer.writerow(['time/s', 'phase/pi', 'amplitude/steps', 'phase_active/pi'])
                else:
                    writer.writerow(['time/s', 'phase/pi'])
                # Following code tries to align the amplitude data with the phase data
                # since they have different lengths in the buffer
                temp_i = 0
                temp_amp = self.amp_list[0][1]
                for i in range(len(self.amp_list) - 1):
                    if(self.amp_list[i + 1][0]):
                        temp_i = i
                        temp_amp = self.amp_list[i][1]
                        break
                for i in range(len(self.phase_list)):
                    if(i == len(self.phase_list) - 1):
                        pass
                    else:
                        if(self.phase_list[i + 1][0] == 0):
                            continue
                    if(self.phase_list_active is not None):
                        if(temp_i == len(self.amp_list) - 1):
                            writer.writerow([self.phase_list[i][0], self.phase_list[i][1],\
                                temp_amp, self.phase_list_active[i][1]])
                        elif(self.phase_list[i][0] < self.amp_list[temp_i + 1][0]):
                            writer.writerow([self.phase_list[i][0], self.phase_list[i][1],\
                                temp_amp, self.phase_list_active[i][1]])
                        else:
                            temp_i += 1
                            temp_amp = self.amp_list[temp_i][1]
                            writer.writerow([self.phase_list[i][0], self.phase_list[i][1],\
                                temp_amp, self.phase_list_active[i][1]])
                    else:
                        if(temp_i == len(self.amp_list) - 1):
                            writer.writerow([self.phase_list[i][0], self.phase_list[i][1]])
                        elif(self.phase_list[i][0] < self.amp_list[temp_i + 1][0]):
                            writer.writerow([self.phase_list[i][0], self.phase_list[i][1]])
                        else:
                            temp_i += 1
                            temp_amp = self.amp_list[temp_i][1]
                            writer.writerow([self.phase_list[i][0], self.phase_list[i][1]])
                csvfile.close()

        print("\nExported to " + filename + "\n")
        if(module_name != 'pid' and module_name != 'setSpeed'):
            print("\nExported to " + filename_fft + "\n")
        if(NR_phase_amp):
            print("\nExported to " + filename_phase_amp + "\n")       
    
class live_data(data):
    
    '''This is the class for live graph plotting without blocking. Inherit from the data class.'''
    
    def __init__(
        self,
        fft_length,
        sampling_div,
        wait_to_stable,
        ):
        super().__init__(fft_length, sampling_div, wait_to_stable)
        
    def copy(self, data, NR = False):
        '''Copy the data from the data class to the live_data class.
        This method is important because then the plotting will be
        independent of the parallel data reading thread as indicted
        in the thread_reader() in cart_pendulum class.'''
        self.time = data.time
        self.angle = data.angle
        self.angular_velocity = data.angular_velocity
        self.position = data.position
        self.position_velocity = data.position_velocity
        self.index = data.index
        self.temp_index = data.temp_index
        self.counter = data.counter
        data.fft_angle = self.fft_angle
        data.fft_pos = self.fft_pos
        data.fft_freq = self.fft_freq
        self.phase = data.phase
        self.omega = data.omega
        self.module_name = data.module_name
        self.path = data.path
        self.avg_spacing = data.avg_spacing
        self.index_list = data.index_list
        self.start_time = data.start_time
        try:
            self.pid_param = data.pid_param
        except AttributeError:
            pass
        # Important, update the amp and phase in the data class
        data.amp_list = self.amp_list
        data.amp = self.amp
        data.phase = self.phase
        data.multi_phase_list = self.multi_phase_list
        data.pos_const = self.pos_const
        data.pos_active = self.pos_active
        self.omega_num = data.omega_num
        self.omega_list = data.omega_list
        self.setSpeed_param = data.setSpeed_param