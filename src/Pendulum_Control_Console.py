import numpy as np
import matplotlib as mpl, matplotlib.pyplot as plt
import serial, time, os, csv, threading
from scipy.fft import fft, fftfreq
from datetime import datetime
import serial.tools.list_ports
from scan_data_process import data_analysis as da
plt.rcParams['axes.grid'] = True
plt.rcParams["figure.autolayout"] = True
mpl.use('TkAgg')

# Initialisation of some constants and variables
port = 'COM6' 
baudrate = 230400 # TODO: extract the baudrate from a larger project file?
MAX_COUNT = 5 # Number of points waited to plot a frame

class pid_param():
    '''Initialisation of the pid parameter class, 
    used to store the pid parameters for machine learning'''
    def __init__(self) -> None:
        pass
# TODO: create a pid parameter class -- deep learning? or add a neural network to the pid parameter class? pytorch required
# TODO: how to achieve higher precision of the accelstepper library??? Using microsteps?

class data():
    
    '''Initialisation of the data class, used to store the data from the arduino
    and plot the graph with blocking'''
    
    def __init__(
        self,
        fft_length,
        sampling_div,
        wait_to_stable,
        buffer_length = 8192,
        plot_length = 512,
        ):
        self.start_time = 0.
        self.sampling_div = sampling_div
        self.avg_spacing = 0.
        self.time = np.zeros(2 * buffer_length)
        self.angle = np.zeros(2 * buffer_length)
        self.angular_velocity = np.zeros(2 * buffer_length)
        self.position = np.zeros(2 * buffer_length)
        self.position_velocity = np.zeros(2 * buffer_length)
        self.omega = 2.
        self.amp = 100.
        self.amp_0 = 200.0 # This is used for characterised the constant oscillation
        self.phase = 0.
        self.NR_Kp = 0.02
        self.NR_Kd = 0.1
        self.NR_Ki = 0.002
        self.fft_angle = np.zeros(fft_length)
        self.fft_pos = np.zeros(fft_length)
        self.fft_freq = np.zeros(fft_length)
        self.buffer_length = buffer_length
        self.fft_length = fft_length
        self.plot_length = plot_length
        self.index_list = np.zeros(fft_length, dtype = int)
        self.phase_list = [(0., 0.)] * self.plot_length
        self.amp_list = [(0., 0.)] * self.plot_length
        self.wait_to_stable = wait_to_stable
        self.index = 0
        self.temp_index = 0
        self.counter = 0
        self.flag_fig_init = True
        self.flag_subplot_init = True
        self.flag_close_event = False
        self.module_name = ""
        self.path = ""
        self.bg_list = []
    
    def append_data(
        self,
        data_frame,
        appendPos = True,
        appendVel = False
    ):  
        if(self.index == 0):
            self.start_time = data_frame.time
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
        self.phase_list = [(0., 0.)] * self.plot_length
        self.amp_list = [(0., 0.)] * self.plot_length
        self.index_list = np.zeros(self.fft_length, dtype = int)
        
    def clear_figure(self):
        plt.close("all")
        self.flag_fig_init = True
        self.flag_subplot_init = True
        self.flag_close_event = False
    
    def init_plot(self, module_name):
        if(self.flag_fig_init):
            self.flag_fig_init = False
            plt.ion()
            if(module_name == "measure"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(1, 2, figsize = (8, 5))
                    self.figure.suptitle('Measure')
                    self.flag_subplot_init = False
                self.line_angle, = self.ax_list[0].plot([], [], 'b-')
                self.line_fft, = self.ax_list[1].plot([], [], 'b-')
                
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
                    self.phase_list = [(0., 0.)] * self.plot_length
                    self.amp_list = [(0., 0.)] * self.plot_length
                    
                self.line_angle, = self.ax_list[0, 0].plot([], [], 'b-')
                self.line_pos, = self.ax_list[1, 0].plot([], [], 'r-')
                self.line_pos_const, = self.ax_list[1, 0].plot([], [], 'g--')
                self.line_fft_ang, = self.ax_list[0, 1].plot([], [], 'b-', label = 'angle')
                ax1 = self.ax_list[0, 1].twinx()
                self.line_fft_pos, = ax1.plot([], [], 'r-', label = 'position')
                self.line_phase, = self.ax_list[1, 1].plot([], [], 'b-', label = 'phase')
                ax2 = self.ax_list[1, 1].twinx()
                self.line_amp, = ax2.plot([], [], 'r-', label = 'amplitude')
                
                self.ax_list[0, 1].legend(loc = 'upper left')
                self.ax_list[1, 1].legend(loc = 'lower right')
                ax1.legend(loc = 'upper right')
                ax2.legend(loc = 'upper right')
                ax1.grid(False)
                ax2.grid(False)
                
                self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                    self.ax_list[1, 0]: (self.line_pos, self.line_pos_const),
                                    self.ax_list[0, 1]: self.line_fft_ang,
                                    ax1: self.line_fft_pos,
                                    self.ax_list[1, 1]: self.line_phase,
                                    ax2: self.line_amp}
                    
                self.ax_list[0, 0].set_xlabel('Time/s')
                self.ax_list[0, 0].set_ylabel('Angle/rad')
                self.ax_list[1, 0].set_xlabel('Time/s')
                self.ax_list[1, 0].set_ylabel('Position/steps')
                self.ax_list[0, 1].set_xlabel('Frequency/Hz')
                self.ax_list[0, 1].set_xlim(0, self.omega)
                self.ax_list[0, 1].set_ylabel('Arbitrary Unit')
                self.ax_list[1, 1].set_xlabel('Time/s')
                self.ax_list[1, 1].set_ylabel('Phase/pi')
            
            elif(module_name == "pid"):
                if(self.flag_subplot_init):
                    self.figure, self.ax_list = plt.subplots(2, 2, figsize=(8, 5))
                    self.figure.suptitle('PID')
                    self.flag_subplot_init = False
                self.line_angle, = self.ax_list[0, 0].plot([], [], 'b-')
                self.line_pos, = self.ax_list[1, 0].plot([], [], 'r-')
                self.line_angle_vel, = self.ax_list[0, 1].plot([], [], 'b-')
                self.line_pos_vel, = self.ax_list[1, 1].plot([], [], 'r-')
                
                self.ax_new_list = {self.ax_list[0, 0]: self.line_angle,
                                    self.ax_list[1, 0]: self.line_pos,
                                    self.ax_list[0, 1]: self.line_angle_vel,
                                    self.ax_list[1, 1]: self.line_pos_vel}
                
                self.ax_list[0, 0].set_xlabel('Time/s')
                self.ax_list[0, 0].set_ylabel('Angle/rad')
                self.ax_list[1, 0].set_xlabel('Time/s')
                self.ax_list[1, 0].set_ylabel('Position/steps')
                self.ax_list[0, 1].set_xlabel('Time/s')
                self.ax_list[0, 1].set_ylabel('Angular Velocity/rad/s')
                self.ax_list[1, 1].set_xlabel('Time/s')
                self.ax_list[1, 1].set_ylabel('Cart Velocity/steps/s')
                
            # Configure the events
            self.figure.canvas.mpl_connect('close_event', self.handle_close)
            self.figure.canvas.mpl_connect('motion_notify_event', self.handle_move)
            self.figure.canvas.manager.set_window_title(module_name)
            self.figure.canvas.draw_idle()
            plt.tight_layout()
            plt.show(block = False)

    def real_time_plot(self, module_name):
        self.module_name = module_name
        if(module_name == "measure"):
            self.fft()
            if(self.index < self.plot_length):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind], 
                                        self.angle[low_ind:high_ind])
                    self.line_fft.set_data(self.fft_freq, 
                                         abs(self.fft_angle))
                    try:
                        txt1 = self.ax_list[1].text(0.5, 1.1, 'sampling rate: ' + str(1 / self.avg_spacing)[:4] + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                        txt2 = self.ax_list[1].text(0.5, 1.2, 'resolution: ' + str(1 / len(self.index_list) / self.avg_spacing)[:5] + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(45)
                            
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
                    low_ind = self.temp_index + 1 + self.buffer_length - self.plot_length
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind], 
                                        self.angle[low_ind:high_ind])
                    self.line_fft.set_data(self.fft_freq, 
                                         abs(self.fft_angle))
                    try: 
                        txt1 = self.ax_list[1].text(0.5, 1.1, 'sampling rate: ' + str(1 / self.avg_spacing)[:4] + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                        txt2 = self.ax_list[1].text(0.5, 1.2, 'resolution: ' + str(1 / len(self.index_list) / self.avg_spacing)[:5] + 'Hz',
                                            transform = self.ax_list[1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(45)
                            
                    self.ax_list[1].set_xlim(0, 2 * self.omega)
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                        txt2.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1
                
        elif(module_name == "NR"):
            self.fft()
            if(self.index < self.plot_length):
                if(self.counter % MAX_COUNT == 0):
                    low_ind = self.buffer_length + 1
                    high_ind = self.temp_index + self.buffer_length + 1
                    
                    self.line_angle.set_data(self.time[low_ind:high_ind],
                                        self.angle[low_ind:high_ind])
                    self.line_pos.set_data(self.time[low_ind:high_ind],
                                           self.position[low_ind:high_ind])
                    self.line_pos_const.set_data(self.time[low_ind:high_ind],
                                                 self.amp_0 * np.sin(2 * np.pi * \
                                                     self.omega * (self.time[low_ind:high_ind] + self.start_time)))
                    self.line_fft_ang.set_data(self.fft_freq, 
                                                abs(self.fft_angle))
                    self.line_fft_pos.set_data(self.fft_freq,
                                               abs(self.fft_pos))
                    self.line_phase.set_data(*zip(*self.phase_list))
                    self.line_amp.set_data(*zip(*self.amp_list))
                    
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.1, 'sampling rate: ' + str(1 / self.avg_spacing)[:4] + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        txt2 = self.ax_list[0, 1].text(0.5, 1.2, 'resolution: ' + str(1 / len(self.index_list) / self.avg_spacing)[:5] + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    self.figure.suptitle(module_name + ' Driven Freq: ' + str(self.omega) + 'Hz')
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(45)
                        
                    self.ax_list[0, 1].set_xlim(0, self.omega * 2)
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
                    self.line_pos_const.set_data(self.time[low_ind:high_ind],
                                                 self.amp_0 * np.sin(2 * np.pi * \
                                                     self.omega * (self.time[low_ind:high_ind] + self.start_time)))
                    self.line_fft_ang.set_data(self.fft_freq, 
                                                abs(self.fft_angle))
                    self.line_fft_pos.set_data(self.fft_freq,
                                               abs(self.fft_pos))
                    self.line_phase.set_data(*zip(*self.phase_list))
                    self.line_amp.set_data(*zip(*self.amp_list))
                    
                    try:
                        txt1 = self.ax_list[0, 1].text(0.5, 1.1, 'sampling rate: ' + str(1 / self.avg_spacing)[:4] + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                        txt2 = self.ax_list[0, 1].text(0.5, 1.2, 'resolution: ' + str(1 / len(self.index_list) / self.avg_spacing)[:5] + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    self.figure.suptitle(module_name + ' Driven Freq: ' + str(self.omega) + 'Hz')
                    
                    for ax in self.ax_new_list:
                        ax.relim()
                        ax.autoscale_view()
                        for label in ax.get_yticklabels():
                            label.set_rotation(45)
                
                    self.ax_list[0, 1].set_xlim(0, self.omega * 2)
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
                        txt1 = self.ax_list[0, 1].text(0.5, 1.1, 'sampling rate: ' + str(1 / self.avg_spacing)[:4] + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    if(self.pid_param == 'r'):
                        self.figure.suptitle('PID parameters (resumed previous values)')
                    else:
                        self.figure.suptitle('PID parameters(' + self.pid_param + ')')
                    
                    for axes in self.ax_list:
                        for ax in axes:
                            ax.relim()
                            ax.autoscale_view()
                            for label in ax.get_yticklabels():
                                label.set_rotation(45)
                        
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
                        txt1 = self.ax_list[0, 1].text(0.5, 1.1, 'sampling rate: ' + str(1 / self.avg_spacing)[:4] + 'Hz',
                                                transform = self.ax_list[0, 1].transAxes)
                    except ZeroDivisionError:
                        pass
                    
                    if(self.pid_param == 'r'):
                        self.figure.suptitle('PID parameters (resumed previous values)')
                    else:
                        self.figure.suptitle('PID parameters(' + self.pid_param + ')')
                    
                    for axes in self.ax_list:
                        for ax in axes:
                            ax.relim()
                            ax.autoscale_view()
                            for label in ax.get_yticklabels():
                                label.set_rotation(45)
                        
                    self.figure.canvas.draw()
                    self.figure.canvas.flush_events()
                    try:
                        txt1.remove()
                    except UnboundLocalError:
                        pass
                    
                self.counter += 1            
        
    def handle_close(self, event):
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
    
    def handle_move(self, event):
        # TODO: Add annotation really this is secondary
        # if event.inaxes:
        #     self.ann = plt.gca().annotate(str(event.xdata) + ',' + str(event.ydata), \
        #         xy = (event.xdata, event.ydata))
        pass

    def export_csv(
        self, 
        module_name, 
        relative_path = r"Normalised_Resonance_Control\NR_csv"
        ):
        try:
            dirc = self.path + '\\' + datetime.now().strftime("%d-%m-csv")
            os.makedirs(dirc)
        except OSError:
            pass
        filename = dirc + '\\' + module_name + \
            datetime.now().strftime("-%H-%M-%S") + ".csv"
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            special_info = input("Any special info to add to the csv file?\n\n")
            writer.writerow(["special_info: " + special_info])
            if(module_name == "pid"):
                try:
                    writer.writerow(["Kp", "Ki", "Kd", "Kp_pos", "Ki_pos", "Kd_pos"])
                    writer.writerow([self.pid_param.split(',')[i] for i in range(6)])
                except (AttributeError, IndexError):
                    pass
            writer.writerow(["temp_index: " + str(self.temp_index)])
            writer.writerow(["omega: " + str(self.omega)])
            try:
                writer.writerow(["amplitude: " + str(self.amp_list[-1][1])])
                writer.writerow(["phase/pi: " + str(self.phase_list[-1][1])])
            except (AttributeError, IndexError):
                pass
            writer.writerow(["time", "angle", "position", "angular_velocity", "cart_velocity"])
            for i in range(len(self.time)):
                writer.writerow([self.time[i], self.angle[i], self.position[i],\
                    self.angular_velocity[i], self.position_velocity[i]])
            csvfile.close()

        print("\nExported to " + filename + "\n")
        
    def fft_index_list(self):
        '''return the list and average time spacing'''
        current_time = self.time[self.temp_index + self.buffer_length]
        time_stamp = current_time
        index = self.fft_length - 2
        index_list = np.zeros(self.fft_length, dtype = int)
        index_list[self.fft_length - 1] = current_time
        
        for i in range(self.temp_index + self.buffer_length, self.temp_index + 1, -1):
            if index < 0: 
                self.index_list = index_list
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
        if(self.time[self.temp_index] > 5 * self.sampling_div):
            index_list, avg_spacing = self.fft_index_list()
            self.avg_spacing = avg_spacing
            
            fft_ang = fft(self.angle[index_list])
            fft_pos = fft(self.position[index_list])
            fft_freq = fftfreq(len(index_list), avg_spacing)
            # print("Length of fft_freq: ", len(index_list))
            self.fft_angle = fft_ang[1:int(len(fft_freq) / 2)]
            self.fft_pos = fft_pos[1:int(len(fft_freq) / 2)]
            self.fft_freq = fft_freq[1:int(len(fft_freq) / 2)]
            return True
        else:
            return False
    
    def NR_phase_calc(self, interpolation = True):
        if (self.fft()):
            close_ind = np.argmin(np.abs(self.fft_freq - self.omega))
            if interpolation:
                if self.fft_freq[close_ind] < self.omega:
                    delta_phase_1 = self.phase_rectify(np.angle(self.fft_angle[close_ind\
                        + 1]) - np.angle(self.fft_pos[close_ind + 1]) + np.pi)
                    delta_phase_0 = self.phase_rectify(np.angle(self.fft_angle[close_ind])\
                        - np.angle(self.fft_pos[close_ind]) + np.pi)
                    
                    self.phase = delta_phase_0 + (self.omega - \
                        self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                            + 1] - self.fft_freq[close_ind]) * \
                                (delta_phase_1 - delta_phase_0)

                elif self.fft_freq[close_ind] > self.omega:
                    delta_phase_1 =  self.phase_rectify(np.angle(self.fft_angle[close_ind\
                        - 1]) - np.angle(self.fft_pos[close_ind - 1]) + np.pi)
                    delta_phase_0 = self.phase_rectify(np.angle(self.fft_angle[close_ind])\
                        - np.angle(self.fft_pos[close_ind]) + np.pi)
                    
                    self.phase = delta_phase_0 + (self.omega - \
                        self.fft_freq[close_ind]) / (self.fft_freq[close_ind\
                            - 1] - self.fft_freq[close_ind]) * \
                                (delta_phase_1 - delta_phase_0)

                else:
                    self.phase = self.phase_rectify(np.angle(self.fft_angle[close_ind]) \
                        - np.angle(self.fft_pos[close_ind]) + np.pi)
            else:
                self.phase = self.phase_rectify(np.angle(self.fft_angle[close_ind]) \
                    - np.angle(self.fft_pos[close_ind]) + np.pi)
            
            self.phase_list.pop(0)
            self.phase_list.append((self.time[self.temp_index], self.phase / np.pi))
            return True
        else:
            return False
        
    def NR_update(self, scan = False, interpolation = True):
        '''Needs to be called to update the plot 
        for the phase and amplitude'''
        if (self.NR_phase_calc(interpolation)):
            if scan:
                # This is the set up for scanning
                return 0, 0
            else:
                delta_amp_Kp = self.NR_Kp * ((self.phase + np.pi / 2))/ (2 * np.pi)
                try:
                    delta_amp_Kd = self.NR_Kd * (self.phase_list[-1][1] - self.phase_list[-2][1]) / \
                        (self.phase_list[-1][0] - self.phase_list[-2][0])
                except RuntimeError:
                    pass
                self.amp *= (1 - delta_amp_Kp) * (1 - delta_amp_Kd)
                # Need a way to transmit this to a thread...
                self.amp_list.pop(0)
                self.amp_list.append((self.time[self.temp_index], self.amp))
                return self.amp, self.phase
        else:
            return 0, 0
    
    def phase_rectify(self, phase):
        '''Shifts the phase to be between 0.5 * pi and -1.5*pi, which is symmetric abour -0.5*pi'''
        phase = phase - 2 * np.pi * int(phase / (2 * np.pi))
        if phase > 0.5 * np.pi:
            return phase - 2 * np.pi
        else:
            return phase
    
    # TODO: Add a function to analyse the frequency spectrum (secondary, let students do these)
    # TODO: fix plot_length with updated index_list (secondary)
    # TODO: add a sampling rate selection in arduino (secondary)
    # TODO: to make the step function in the NR stage continuous
    # TODO: rethink about the integral control,, integrate from the start (secondary)
    # TODO: NR amplitude tuning? --> this is related to the resonance driving frequency???
    # TODO: check the NR_update function
    # TODO: rethink about the derivative control
    # TODO: add a different title for downward and upward control
    # TODO: add a different title for scanning for response
    # TODO: label in the csv file of different setup !!! Useful for later analysis
    # TODO: much clever way to update the amplitude use previous amplitude because 
    # overall the motion is sinusoidal, so maybe use this property to connect the two waves?
    
class live_data(data):
    
    '''This is the class for live graph plotting without blocking. Inherit from the data class.'''
    
    def __init__(
        self,
        fft_length,
        sampling_div,
        wait_to_stable,
        ):
        super().__init__(fft_length, sampling_div, wait_to_stable)
        
    def copy(self, data):
        self.time = data.time
        self.angle = data.angle
        self.angular_velocity = data.angular_velocity
        self.position = data.position
        self.position_velocity = data.position_velocity
        self.index = data.index
        self.temp_index = data.temp_index
        self.counter = data.counter
        self.fft_angle = data.fft_angle
        self.fft_pos = data.fft_pos
        self.fft_freq = data.fft_freq
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
    
class data_frame():
    
    descritpion = "This class is a single data_frame to store data from arduino"
    
    def __init__(
        self,
        time = 0.,
        angle = 0.,
        position = 0.,
        angular_velocity = 0.,
        position_velocity = 0.
    ):
        self.time = time
        self.angle = angle
        self.position = position
        self.angular_velocity = angular_velocity
        self.position_velocity = position_velocity
    
    def update_data(self, 
                    data,
                    appendPos = True,
                    appendVel = False):
        try:
            self.time = float(data[0])
            self.angle = float(data[1])
            if(appendPos):
                self.position = float(data[2])
            if(appendVel):
                self.angular_velocity = float(data[3])
                self.position_velocity = float(data[4])
        except TypeError:
            pass

class arduino():
    
    '''Arduino class'''
    
    def __init__(
        self,
        port,
        baudrate,
        timeout = 10,
        dsrdtr = None
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dsrdtr = dsrdtr
        self.message = ""
        self.receive = ""
        self.command = ""
        self.omega = ""
    
    def clear(self):
        self.message = ""
        self.receive = ""
        self.command = ""
        self.omega = ""
        try:
            self.board.reset_input_buffer()
            self.board.reset_output_buffer()
        except IOError:
            pass

    def find_port(self):
        arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'USB Serial Device' in p.description  # Adjust this condition based on your Arduino's description
    ]
        if not arduino_ports:
            raise IOError("No Arduino found. Please make sure it's connected.")
        elif len(arduino_ports) > 1:
            print("Multiple Arduinos found. Selecting the first one.")
        
        self.port = arduino_ports[0]
        # TODO: select the port from the list of ports (secondary)
    
    def initiate(self):
        self.find_port()
        self.board = serial.Serial(
            self.port,
            self.baudrate,
            timeout = self.timeout,
            dsrdtr = self.dsrdtr
        )
        self.board.write('connection\n'.encode('ASCII'))
        self.read_single()
        
    def send_command(self):
        # possible commands: reboot, center, pid, measure, NR 
        self.command = input() + "\n"
        print("")
        self.board.write(self.command.encode('ASCII'))
        
    def send_message(self, message):
        self.message = message
        self.board.write(message.encode('ASCII'))
        
    def send_input_message(self, save_to_omega = True):
        self.message = input() + "\n"
        print("")
        self.board.write(self.message.encode('ASCII'))
        if(save_to_omega):
            self.omega = self.message
    
    def send_list_omega(self):
        temp_flag = True
        while(temp_flag):
            try:
                num = int(input("Number of frequencies to scan simultaneously (maximum 10): "))
            except ValueError:
                print("Please enter a valid number")
            if(num == 1):
                print("Please enter the driven frequency: ")
                self.send_input_message()
                temp_flag = False
            elif(num >= 2 and num <= 10): 
                start_point, end_point = int(input("Please enter the start and end frequency in this format (a,b): ").split(','))
                if(end_point <= start_point):
                    print("Please enter a valid range")
                    continue
                else:
                    msg = ""
                    omega_list = np.linspace(start_point, end_point, num)
                    for i in range(num - 1):
                        msg += str(omega_list[i]) + ","
                    msg += str(omega_list[-1]) + "\n"
                    temp_flag_check = True
                    while(temp_flag_check):
                        print("The frequency list is: ", msg, "\n")
                        temp = input("Is this correct? (y/n): ")
                        if(temp == "y"):
                            self.send_message(msg)
                            temp_flag = False
                            temp_flag_check = False
                        elif(temp == "n"):
                            temp_flag_check = False
                        else:
                            print("Please enter a valid input")
            else:
                print("Invalid input, please enter an integer between 1 and 10")
        
    def read_single(self, prt = True, in_waiting = True):
        if(in_waiting):
            while(self.board.in_waiting == 0):
                pass
        self.receive = self.board.readline().decode('ASCII')
        if(prt):
            print(self.receive)
        
    def read_all(self):
        while(self.board.in_waiting == 0):
            pass
        while(self.board.in_waiting):
            self.receive = self.board.readline().decode('utf-8')
            print(self.receive)
               
class cart_pendulum():
    
    '''Cart pendulum class'''
    
    def __init__(
        self,
        arduino,
        data):
        self.arduino = arduino
        self.data = data
        self.module_name = r"\Defaut_Cart_Pendulum"
        self.center_count = 0
        self.distance = 0
        self.NR_counter = 0
        # A dictionary of flags to control the system
        self.flag_list = {
            "command": True, # whether a command is sent to the arduino
            "reset": False, # reset command
            "center": False, # center command
            "pid": False, # pid command
            "measure": False, # measure command
            "NR": False, # Normalised Resonance command
            "omega": True, # Input driven frequency command
            "swing_request": True, # whether the swing is requested
            "pid_input": True, # whether the pid input is requested
            "thread_init": True, # whether the thread is initiated
            "flag_scan": True # whether run scanning mode
        }
        self.init_true_flag_list = ["command", 
                                    "omega",
                                    "swing_request",
                                    "pid_input",
                                    "thread_init",
                                    "flag_scan"]
        self.init_false_flag_list = ["reset", 
                                     "center", 
                                     "pid", 
                                     "measure", 
                                     "NR"]
        self.reset_dict = { # To renew the flag_command
            "Resetting...",
            "No command detected.",
            "Unidentified command. Please try again.",
            "More than one command detected. Resetting the values."
        }
        self.command_dict = {
            "Begin centering.": "center",
            "Begin the PID control.": "pid",
            "Begin the natural frequency and quality factor measuring.": "measure",
            "Begin the normalised resonance.": "NR"
        }
        
    def reset_flag_list(self, swing_request = False):  
        for flag in self.init_true_flag_list:
            self.flag_list[flag] = True
        for flag in self.init_false_flag_list:
            self.flag_list[flag] = False
        if(swing_request):
            self.flag_list["swing_request"] = False
        time.sleep(0.5)
            
    def reconnect(self, 
                  exp = False, 
                  swing_request = False, 
                  send_terminate = False
                  ):
        '''This function stops the serial connection and waits for ENTER to reconnect'''
        if(send_terminate):
            time.sleep(0.1)
            self.arduino.send_message("Terminate\n")
        try:
            plt.close("all")
            self.arduino.read_single()
            self.arduino.clear()
            self.arduino.board.close()
            time.sleep(0.1)
            if(exp):
                temp_datum.export_csv(self.module_name, relative_path = self.path)
            input("\nPress ENTER to reconnect.\n\nOr press CTRL+C then ENTER to exit the program.\n")
            self.arduino.initiate()
        except KeyboardInterrupt:
            self.arduino.board.close()

        self.reset(reset_data = False, swing_request = swing_request)
         
    def reset(self, reset_data = True, swing_request = False):
        self.arduino.clear()
        self.reset_flag_list(swing_request = swing_request)
        self.data.clear_data()
        self.data.clear_figure()
        temp_datum.clear_data()
        temp_datum.clear_figure()
        if(reset_data):
            self.clear_data()
    
    def clear_data(self):
        self.center_count = 0
        self.distance = 0
    
    def command_flag(self): # command flag controlled by the arduino output
        if(self.arduino.receive.rstrip() not in self.reset_dict \
            and self.arduino.receive.rstrip() in self.command_dict):
            self.flag_list[self.command_dict[self.arduino.receive.rstrip()]] = True
            self.flag_list["command"] = False
            # print("Press Ctrl+C to stop the program.\n")
        elif (self.arduino.receive.rstrip() in self.reset_dict):
            self.flag_list["command"] = False
            self.flag_list["reset"] = True
        else:
            self.flag_list["command"] = True
            
    def command(self):
        self.arduino.read_all()
        self.arduino.send_command()
        self.arduino.read_single()
        self.command_flag()
    
    def thread_reader(self, appendPos = False, appendVel = False):
        while(not temp_datum.flag_close_event):
            self.arduino.read_single(prt = False, in_waiting = True)
            if(self.arduino.receive.rstrip() == "Kill switch hit."):
                temp_datum.flag_close_event = True
                break
            try:
                df.update_data(self.arduino.receive.rstrip().split(','), \
                    appendPos = appendPos, appendVel = appendVel)
                self.data.append_data(df, appendPos = appendPos, appendVel = appendVel)
            except ValueError:
                self.arduino.board.reset_input_buffer()
                pass
    
    def thread_writer(self):
        while(not temp_datum.flag_close_event):
            msg = input("Send the new amplitude/steps\n") + "\n"
            try:
                a = float(msg.split(',')[0])
                if(self.data.omega * abs(a) > 2000):
                    print("The amplitude is too large. Please try again.\n")
                else:
                    msg = str(abs(a)) + "," + str(temp_datum.phase) + "\n"
                    self.arduino.send_message(msg)
                    print(msg)
            except ValueError:
                print("Invalid input. Please try again.\n")
            time.sleep(0.01)
    
    def center(self):
        self.module_name = r"center"
        self.arduino.read_single(prt = False)
        self.center_count, self.distance = int(self.arduino.receive.rstrip().split(',')[0]),\
            int(self.arduino.receive.rstrip().split(',')[1])
        print("Centering done: ", self.center_count, "\tRail Distance: ", self.distance, "\n")
        self.reset_flag_list()
        
    def pid(self):
        self.module_name = r"pid"
        try:
            self.data.path = self.path + r"\pid"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["swing_request"]):
            self.arduino.read_single()
            # self.arduino.send_input_message(save_to_omega = False)
            self.arduino.send_message('n' + '\n')
            self.arduino.read_single()
            if(self.arduino.receive.rstrip() == "Continue with swing up strategy." or \
                self.arduino.receive.rstrip() == "Continue without swing up strategy."):
                self.flag_list["swing_request"] = False
        else: 
            if(self.flag_list["pid_input"]):
                self.arduino.read_all()
                self.arduino.send_input_message()
                self.arduino.read_all()
                if(self.arduino.receive.rstrip() == "Start inversion control."):
                    self.data.pid_param = self.arduino.message.rstrip()
                    self.flag_list["pid_input"] = False
            else:
                if(self.arduino.receive.rstrip() == "Kill switch hit."):
                    print("Kill switch hit. Resetting the system...\n")
                    self.reconnect(exp = True)
                else:
                    if(self.flag_list["thread_init"]):
                        reader = threading.Thread(target = self.thread_reader, 
                                                args = (True, True))
                        reader.start()
                        self.flag_list["thread_init"] = False
                    
                    if(not temp_datum.flag_close_event):
                        temp_datum.copy(self.data)
                        temp_datum.init_plot(self.module_name)
                        temp_datum.real_time_plot(self.module_name)
                    else:
                        self.reconnect(exp = True)
                        
    def measure(self):
        self.module_name = r"measure"
        try:
            self.data.path = self.path + r"\measure"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["thread_init"]):
            reader = threading.Thread(target = self.thread_reader, 
                                      args = (False, False))
            reader.start()
            self.flag_list["thread_init"] = False
        # plot the graph in the main thread
        if(not temp_datum.flag_close_event):
            temp_datum.copy(self.data)
            temp_datum.init_plot(self.module_name)
            temp_datum.real_time_plot(self.module_name)
        else:
            self.reconnect(exp = True)
    
    def NR(self, NR_scan = False, interpolation = True):
        '''Be careful of the step function -- pi phase difference'''
        self.module_name = r"NR"
        try:
            self.data.path = self.path + r"\NR"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["omega"]):
            self.flag_list["omega"] = False
            self.arduino.read_single(prt = False)
            # self.arduino.send_input_message()
            self.arduino.send_list_omega()
            self.arduino.read_single()
            if(self.arduino.receive.rstrip() == "Invalid input, please try again."):
                self.flag_list["omega"] = True
            if(self.flag_list["omega"] == False):
                self.data.omega = float(self.arduino.omega.rstrip())
        else:
            if(self.arduino.receive.rstrip() == "Kill switch hit."):
                print("Kill switch hit. Resetting the system...\n")
                self.reconnect(exp = True)
            else:
                if(self.flag_list["thread_init"]):
                    reader = threading.Thread(target = self.thread_reader, 
                                            args = (True, False))
                    reader.start()
                    if (not NR_scan):
                        writer = threading.Thread(target = self.thread_writer, args = ())
                        writer.start()
                    self.flag_list["thread_init"] = False
                
                if(not temp_datum.flag_close_event):
                    temp_datum.copy(self.data)
                    temp_datum.init_plot(self.module_name)
                    temp_datum.real_time_plot(self.module_name)
                else:
                    reader.join()
                    if(not NR_scan):
                        writer.join()
                    self.reconnect(exp = True)
                
                # BUG: currently disabled because the PID of the 
                # NR stage has not been fine tuned
                if(self.NR_counter >= temp_datum.wait_to_stable):
                    _, _ = temp_datum.NR_update(NR_scan, interpolation) 
                    # print("Amplitude: ", amp, " Phase: ", phase / np.pi, "\n")
                    # self.arduino.send_message(str(amp) + "," + str(phase) + "\n")
                    self.NR_counter = 0
                else:
                    self.NR_counter += 1

    def create_folder(self):
        self.cwd = os.getcwd()
        self.path = self.cwd + r"\cart_pendulum_data"
        try: 
            os.mkdir(self.path)
        except OSError:
            pass

    def main(self):
        input("\nPress ENTER to begin connection...\n")
        self.create_folder()
        self.arduino.initiate()
        while(self.arduino.board.is_open):
            try:
                if(self.flag_list["command"]):
                    try:
                        self.command()
                    except KeyboardInterrupt:
                        self.reconnect()
                elif(self.flag_list["reset"]):
                    self.reset(reset_data = True)
                elif(self.flag_list["center"]):
                    try:
                        self.center()
                    except KeyboardInterrupt:
                        self.reconnect()
                elif(self.flag_list["pid"]):
                    try:
                        self.pid()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True)
                elif(self.flag_list["measure"]):
                    try:
                        self.measure()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True)
                elif(self.flag_list["NR"]):
                    try:
                        while(self.flag_list["flag_scan"]):
                            msg = input("Scan for response? (y/n): ")
                            if(msg.rstrip() == 'y'):
                                self.flag_list["flag_scan"] = False
                                NR_scan = True
                            elif(msg.rstrip() == 'n'):
                                self.flag_list["flag_scan"] = False
                                NR_scan = False
                            else:
                                print("\nInvalid input, please try again.")
                            print("")
                        self.NR(NR_scan = NR_scan, interpolation = True)
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True)
                    
            except KeyboardInterrupt:
                self.arduino.board.close() # Triggers reset() in the arduino
                self.reset_flag_list()
                break


if __name__ == "__main__":
    
    # Start up routine of the test
    fft_lengths = 1024
    sampling_divs = 0.1
    wait_to_stables = 5
    # fft_length = int(input("fft_length: "))
    # sampling_div = float(input("sampling_div: "))
    # wait_to_stable = int(input("wait_to_stable: "))
        
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

    cartER.main()
    print("\nProgram ends.")

# TODO: ask whether to enter data analysis mode
# TODO: check whether the platformio can do the arduino code upload