import numpy as np
import matplotlib as mpl, matplotlib.pyplot as plt
import time, os, threading
# import modules from other python files
from data_process import data, live_data
from arduino_manager import arduino
from moment_data_process import data_frame
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


# This is simply a class to manage the cart pendulum system, nothing physically interesting
class cart_pendulum():
    
    '''Cart pendulum manager class'''
    
    def __init__(
        self,
        arduino,
        data,
        temp_data,
        data_frame,):
        self.arduino = arduino
        self.data = data
        self.temp_datum = temp_data
        self.df = data_frame
        self.module_name = r"\Defaut_Cart_Pendulum"
        self.center_count = 0
        self.distance = 0
        self.NR_counter = 0
        self.thread_counter = 0
        # A dictionary of flags to control the system
        self.flag_list = {
            "command": True, # whether a command is sent to the arduino
            "reset": False, # reset command
            "center": False, # center command
            "pid": False, # pid command
            "measure": False, # measure command
            "NR": False, # Normalised Resonance command
            "setSpeed": False, # set speed and acceleration command
            "freq_scan": False, # frequency scan command
            "multi_freq": False, # whether multiple frequencies are sent
            "omega": True, # Input driven frequency command
            "amp": True, # Input varying amplitude command
            "amp_0": True, # Input constant amplitude command
            "swing_request": True, # whether the swing is requested
            "pid_input": True, # whether the pid input is requested
            "thread_init": True, # whether the thread is initiated
            "flag_scan": True, # whether run scanning mode
            "setSpeed_request": True, # whether the setSpeed is requested
        }
        self.init_true_flag_list = ["command", 
                                    "omega",
                                    "amp",
                                    "amp_0",
                                    "swing_request",
                                    "pid_input",
                                    "thread_init",
                                    "flag_scan",
                                    "setSpeed_request"]
        self.init_false_flag_list = ["reset", 
                                     "center", 
                                     "pid", 
                                     "measure", 
                                     "NR",
                                     "setSpeed",
                                     "freq_scan",
                                     "multi_freq",]
        self.reset_dict = { # To renew the flag_command
            "Resetting...",
            "No command detected.",
            "Unidentified command. Please try again.",
            "More than one command detected. Resetting the values.",
            "Hasn't been centered. Please center the cart first.",
        }
        self.command_dict = {
            "Begin centering.": "center",
            "Begin the PID control.": "pid",
            "Begin the natural frequency and quality factor measuring.": "measure",
            "Begin the normalised resonance.": "NR",
            "Begin the speed and acceleration setting.": "setSpeed",
            "Begin the frequency scan.": "freq_scan",
        }
        
    def reset_flag_list(self, swing_request = False):  
        for flag in self.init_true_flag_list:
            self.flag_list[flag] = True
        for flag in self.init_false_flag_list:
            self.flag_list[flag] = False
        if(swing_request):
            self.flag_list["swing_request"] = False
            
    def reconnect(self, 
                  exp = False, 
                  swing_request = False, 
                  send_terminate = False,
                  NR_phase_amp = False,
                  manual_continue = True,
                  input_spec_info = True,
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
                self.temp_datum.export_csv(self.module_name, 
                                      NR_phase_amp = NR_phase_amp,
                                      input_spec_info = input_spec_info,)
            if(manual_continue):
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
        self.temp_datum.clear_data()
        self.temp_datum.clear_figure()
        if(reset_data):
            self.clear_data()
    
    def clear_data(self):
        self.center_count = 0
        self.distance = 0
        self.phase = 0.
        self.NR_counter = 0
        self.thread_counter = 0
    
    def command_flag(self): 
        # command flag controlled by the arduino output
        if(self.arduino.receive.rstrip() not in self.reset_dict \
            and self.arduino.receive.rstrip() in self.command_dict):
            self.flag_list[self.command_dict[self.arduino.receive.rstrip()]] = True
            self.flag_list["command"] = False
        elif (self.arduino.receive.rstrip() in self.reset_dict):
            self.flag_list["command"] = False
            self.flag_list["reset"] = True
        else:
            self.flag_list["command"] = True
            
    def command(self):
        self.arduino.read_all()
        self.arduino.send_command()
        # self.arduino.read_single() # previously used
        self.arduino.read_all()
        self.command_flag()
    
    def thread_reader(self, 
                      appendPos = False, 
                      appendVel = False, 
                      thread_check = False):
        while(not self.temp_datum.flag_close_event):
            self.arduino.read_single(prt = False, in_waiting = True)
            if(self.arduino.receive.rstrip() == "Kill switch hit."):
                self.temp_datum.flag_close_event = True
                break
            try:
                self.df.update_data(self.arduino.receive.rstrip().split(','), \
                    appendPos = appendPos, appendVel = appendVel)
                self.data.append_data(self.df, appendPos = appendPos, appendVel = appendVel)
                if(thread_check):
                    print("time_sys: %.3f time_read: %.3f thread_counter: %d" % \
                        ((time.time() - self.data.sys_start_time), (self.df.time - self.data.start_time), self.thread_counter))
                    self.thread_counter += 1
            except ValueError:
                self.arduino.board.reset_input_buffer()
                pass
    
    def thread_writer(self):
        while(not self.temp_datum.flag_close_event):
            msg = input("Send the new amplitude/steps (Press Ctrl+C to exit!!!)\n") + "\n"
            try:
                a = float(msg.split(',')[0])
                if(self.data.omega * abs(a) > 2000):
                    print("The amplitude is too large. Please try again.\n")
                else:
                    msg = str(abs(a)) + "," + str(self.phase) + "\n"
                    self.arduino.send_message(msg)
                    self.temp_datum.amp = abs(a)
                    self.data.amp = abs(a)
                    print("sent amp, phase: " + msg)
                    # BUG: not sending the phase at the same time!
            except ValueError:
                print("Invalid input, please try again.\n")
            time.sleep(2) # wait 2 seconds for the transient behaviour to fade away a bit
    
    def center(self):
        self.module_name = r"center"
        self.arduino.read_single(prt = False)
        self.center_count, self.distance = int(self.arduino.receive.rstrip().split(',')[0]),\
            int(self.arduino.receive.rstrip().split(',')[1])
        print("Centering done: ", self.center_count, "\tRail Distance: ", self.distance, "\n")
        self.reset_flag_list()
                  
    def measure(self):
        self.module_name = r"measure"
        try:
            self.data.path = self.path + r"\measure"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["thread_init"]):
            reader = threading.Thread(target = self.thread_reader, 
                                      args = (False, False, False))
            reader.start()
            self.flag_list["thread_init"] = False
        # plot the graph in the main thread
        if(not self.temp_datum.flag_close_event):
            self.temp_datum.copy(self.data)
            self.temp_datum.init_plot(self.module_name)
            self.temp_datum.real_time_plot(self.module_name)
        else:
            self.reconnect(exp = True)

    def setSpeed(self):
        '''Set the speed and acceleration of the cart'''
        self.module_name = r"setSpeed"
        try:
            self.data.path = self.path + r"\setSpeed"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["setSpeed_request"]):
            self.arduino.read_all()
            self.arduino.send_input_message(save_to_omega = False)
            self.arduino.read_single()
            if(self.arduino.receive.rstrip().startswith("Start sinusoidal motion with")):
                self.flag_list["setSpeed_request"] = False
                self.data.setSpeed_param = self.arduino.receive.rstrip().replace("Start sinusoidal motion with ", "")
        else:
            if(self.flag_list["amp_0"]):
                self.arduino.read_all()
                self.arduino.send_input_message(save_to_omega = False)
                msg_amp = self.arduino.message.rstrip()
                self.arduino.read_single()
                if(self.arduino.receive.rstrip().startswith("Start with amplitude:")):
                    self.flag_list["amp_0"] = False
                    self.data.amp_0 = float(msg_amp)
                    self.temp_datum.amp_0 = float(msg_amp)
            else:
                if(self.arduino.receive.rstrip() == "Kill switch hit."):
                    print("Kill switch hit. Resetting the system...\n")
                    self.reconnect(exp = True)
                else:
                    if(self.flag_list["thread_init"]):
                        reader = threading.Thread(target = self.thread_reader, 
                                                args = (True, True, False))
                        reader.start()
                        self.flag_list["thread_init"] = False
                        
                    if(not self.temp_datum.flag_close_event):
                        self.temp_datum.copy(self.data)
                        self.temp_datum.init_plot(self.module_name)
                        self.temp_datum.real_time_plot(self.module_name)
                    else:
                            self.reconnect(exp = True)      
        
    def freq_scan(self):
        self.module_name = r"freq_scan"
        try:
            self.data.path = self.path + r"\freq_scan"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["omega"]):
            self.flag_list["omega"] = False
            self.arduino.read_single(prt = False)
            if (self.arduino.send_list_omega()):
                self.flag_list["multi_freq"] = True
                self.data.omega_num = len(self.arduino.omega_list)
            else:
                self.data.omega_num = 1
            self.arduino.read_single()
            if(self.arduino.receive.rstrip() == "Invalid input, please try again."):
                self.flag_list["omega"] = True
            if(self.flag_list["omega"] == False and self.flag_list["multi_freq"] == False):
                self.data.omega = float(self.arduino.omega.rstrip())
            elif(self.flag_list["omega"] == False and self.flag_list["multi_freq"] == True):
                self.data.omega_list = self.arduino.omega_list
                self.data.omega = self.arduino.omega_list[-1] # default to take the largest value in the list
        else:
            if(self.flag_list["amp_0"]):
                self.arduino.read_all()
                self.arduino.send_input_message(save_to_omega = False)
                msg_amp = self.arduino.message.rstrip()
                self.arduino.read_single()
                if(self.arduino.receive.rstrip().startswith("Start with amplitude:")):
                    self.flag_list["amp_0"] = False
                    self.data.amp_0 = float(msg_amp)
                    self.temp_datum.amp_0 = float(msg_amp)
            else:
                if(self.arduino.receive.rstrip() == "Kill switch hit."):
                    print("Kill switch hit. Resetting the system...\n")
                    self.reconnect(exp = True, NR_phase_amp = False)
                else:
                    if(self.flag_list["thread_init"]):
                        reader = threading.Thread(target = self.thread_reader, 
                                                args = (True, False, False))
                        reader.start()
                        self.flag_list["thread_init"] = False
                    
                    if(not self.temp_datum.flag_close_event):
                        self.temp_datum.copy(self.data, True)
                        self.temp_datum.init_plot(self.module_name)
                        self.temp_datum.real_time_plot(self.module_name, scan = True)
                    else:
                        self.reconnect(exp = True)
                    
                    if(self.data.omega_list is None):
                        self.temp_datum.NR_phase_calc(self.data.omega, scan = True, interpolation = True)
                    else:
                        self.temp_datum.NR_update(scan = True, interpolation = True)
                    self.NR_counter += 1
        
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
            time.sleep(0.1)
            self.arduino.send_message('n' + '\n')
            self.arduino.read_single()
            if(self.arduino.receive.rstrip() == "Continue with swing up strategy." or \
                self.arduino.receive.rstrip() == "Continue without swing up strategy."):
                self.flag_list["swing_request"] = False
        else: 
            if(self.flag_list["pid_input"]):
                self.arduino.read_all()
                self.arduino.send_input_message(save_to_omega = False)
                # Time waited for Arduino to printout the message through serial port
                time.sleep(0.01)
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
                                                args = (True, True, False))
                        reader.start()
                        self.flag_list["thread_init"] = False
                    
                    if(not self.temp_datum.flag_close_event):
                        self.temp_datum.copy(self.data)
                        self.temp_datum.init_plot(self.module_name)
                        self.temp_datum.real_time_plot(self.module_name)
                    else:
                        self.reconnect(exp = True)
          
    def NR(self, NR_scan = False, interpolation = True):
        self.module_name = r"NR"
        try:
            self.data.path = self.path + r"\NR"
            os.makedirs(self.data.path)
        except OSError:
            pass
        if(self.flag_list["omega"]):
            self.flag_list["omega"] = False
            if(NR_scan):
                self.arduino.read_single(prt = False)
                if (self.arduino.send_list_omega()):
                    self.flag_list["multi_freq"] = True
                    self.data.omega_num = len(self.arduino.omega_list)
                else:
                    self.data.omega_num = 1
            else:
                self.arduino.read_single()
                self.arduino.send_input_message(save_to_omega = True)
            self.arduino.read_single()
            if(self.arduino.receive.rstrip() == "Invalid input, please try again."):
                self.flag_list["omega"] = True
            if(self.flag_list["omega"] == False and self.flag_list["multi_freq"] == False):
                self.data.omega = float(self.arduino.omega.rstrip())
            elif(self.flag_list["omega"] == False and self.flag_list["multi_freq"] == True):
                self.data.omega_list = self.arduino.omega_list
                self.data.omega = self.arduino.omega_list[-1] # default to take the largest value in the list
        else:
            while(self.flag_list["amp"]):
                print("\n\nCurrent initial active amplitude is %.2f\n"%(self.data.amp))
                temp_active_amp = input("Please enter the initial active amplitude in steps: ")
                print("")
                try:
                    temp_active_amp = float(temp_active_amp)
                    self.data.amp = temp_active_amp
                    self.temp_datum.amp = temp_active_amp
                    self.flag_list["amp"] = False
                except ValueError:
                    print("\nPlease enter a valid number.")
            if(self.flag_list["amp_0"]):
                self.arduino.read_all()
                self.arduino.send_input_message(save_to_omega = False)
                msg_amp = self.arduino.message.rstrip()
                self.arduino.read_single()
                if(self.arduino.receive.rstrip().startswith("Start with amplitude:")):
                    self.flag_list["amp_0"] = False
                    self.data.amp_0 = float(msg_amp)
                    self.temp_datum.amp_0 = float(msg_amp)
            else:
                if(self.arduino.receive.rstrip() == "Kill switch hit."):
                    print("Kill switch hit. Resetting the system...\n")
                    self.reconnect(exp = True, NR_phase_amp = not NR_scan)
                else:
                    manual = False # turn up manual control of the amplitude
                    if(self.flag_list["thread_init"]):
                        reader = threading.Thread(target = self.thread_reader, 
                                                args = (True, False, False))
                        reader.start()
                        if (not NR_scan and manual):
                            writer = threading.Thread(target = self.thread_writer, args = ())
                            writer.start()
                        self.flag_list["thread_init"] = False
                    
                    if(not self.temp_datum.flag_close_event):
                        self.temp_datum.copy(self.data, True)
                        self.temp_datum.init_plot(self.module_name, NR_scan)
                        self.temp_datum.real_time_plot(self.module_name, NR_scan)
                    else:
                        if(not NR_scan and manual):
                            writer.join()
                        self.reconnect(exp = True, NR_phase_amp = not NR_scan)
                    
                    if(self.NR_counter >= self.temp_datum.wait_to_stable):
                        amp, self.phase = self.temp_datum.NR_update(NR_scan, interpolation, manual) 
                        if(not manual):
                            # After attempting many times, this is the correct way to update the phase
                            self.arduino.send_message(str(amp) + "," + str(self.phase + np.pi) + "\n")
                            # self.arduino.send_message(str(amp) + "," + str(self.phase) + "\n")
                        elif(manual and not NR_scan):
                            pass
                        self.NR_counter = 0
                    else:
                        if(self.data.omega_list is None):
                            self.temp_datum.NR_phase_calc(self.data.omega, NR_scan, interpolation)
                        else:
                            pass
                        self.NR_counter += 1

    def main_auto_freq_scan(self,
                            auto_freq,
                            auto_amp,
                            duration,
                            trans_fade_time = 40,
                            ):
        self.module_name = r"auto_freq_scan"
        self.path = os.getcwd()
        try:
            self.data.path = self.path + r"\auto_freq_scan"
            os.makedirs(self.data.path)
        except OSError:
            pass
        self.arduino.initiate()
        self.arduino.read_all()
        time.sleep(1)
        self.arduino.send_message("0\n") # to reset the arduino board
        time.sleep(1)
        self.arduino.read_all()
        self.arduino.send_message("1\n") # to center the cart as a routine
        self.arduino.read_single()
        time.sleep(trans_fade_time)
        self.arduino.read_all()
        self.arduino.send_message("4\n") # to turn on the frequency scan mode
        time.sleep(1)
        self.arduino.read_single()
        self.arduino.send_message(str(auto_freq) + "\n") # to send the automated frequency
        self.data.omega = auto_freq
        self.temp_datum.omega = auto_freq
        time.sleep(1)
        self.arduino.read_all()
        self.arduino.send_message(str(auto_amp) + "\n") # to send the automated amplitude
        self.data.amp_0 = auto_amp
        self.temp_datum.amp_0 = auto_amp
        self.arduino.read_single()
        self.auto_start_time = time.time()
        while(not self.temp_datum.flag_close_event):
            if(self.arduino.receive.rstrip() == "Kill switch hit."):
                print("Kill switch hit. Resetting the system...\n")
                self.reconnect(exp = True, 
                               send_terminate = True, 
                               NR_phase_amp = True,
                               manual_continue = False,
                               input_spec_info = False,
                               )
            else:
                
                if(time.time() - self.auto_start_time > duration):
                    self.reconnect(exp = True, 
                                   send_terminate = True, 
                                   NR_phase_amp = True,
                                   manual_continue = False,
                                   input_spec_info = False,
                                  )
                    reader.join()
                    break
                
                if(self.flag_list["thread_init"]):
                    reader = threading.Thread(target = self.thread_reader, 
                                            args = (True, False, False))
                    reader.start()
                    self.flag_list["thread_init"] = False
                
                if(not self.temp_datum.flag_close_event):
                    self.temp_datum.copy(self.data, True)
                    self.temp_datum.init_plot(self.module_name)
                    self.temp_datum.real_time_plot(self.module_name, scan = True)
                else:
                    self.reconnect(exp = True, 
                                   send_terminate = True, 
                                   NR_phase_amp = True, 
                                   manual_continue = False,
                                   input_spec_info = False,
                                   )
                    reader.join()
                    break
                
                self.temp_datum.NR_phase_calc(self.data.omega, scan = True, interpolation = True)
    
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
                        self.NR(NR_scan = False, interpolation = True)
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True, NR_phase_amp = False)
                elif(self.flag_list["setSpeed"]):
                    try:
                        self.setSpeed()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True)
                elif(self.flag_list["freq_scan"]):
                    try:
                        self.freq_scan()
                    except KeyboardInterrupt:
                        self.reconnect(exp = True, send_terminate = True)
            except KeyboardInterrupt:
                self.arduino.board.close() # Triggers reset() in the arduino
                self.reset_flag_list()
                break

if(__name__ == "__main__"):
    # Initiation parameters
    fft_lengths = 512 # Good values are 2**n (same as 2^n), possible to choose other numbers
    sampling_divs = 0.04 # The minimum sampling division set in Arduino is 50 ms
    wait_to_stables = 1 # NR stage parameter, but also controls the updating rate of phase plot
        
    #  Initialisation of the arduino board and the data class
    arduino_board = arduino(port, baudrate) # initiate the arduino class
    df = data_frame() # a moment data frame class
    datum = data(fft_length = fft_lengths, 
                sampling_div = sampling_divs, 
                wait_to_stable = wait_to_stables) # a data class for storing data
    temp_datum = live_data(fft_length = fft_lengths, 
                sampling_div = sampling_divs, 
                wait_to_stable = wait_to_stables) # variable for non-blocking plot
    cartER = cart_pendulum(arduino_board, datum, temp_datum, df)

    cartER.main()
    print("\nProgram ends.")