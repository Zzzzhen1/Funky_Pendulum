import numpy as np
import serial
import serial.tools.list_ports

class arduino():
    
    '''arduino class for I2C communication and Arduino initialisation'''
    
    def __init__(
        self,
        port,
        baudrate,
        timeout = None, 
        dsrdtr = None, # TODO: this is supposedly referring to the reset pin, however not working properly
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
        '''Clear the message, receive and command variables'''
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
        '''Automatically find the port of the arduino. 
        Adjust 'USB Serial Device' to your arduino's description'''
        arduino_ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if 'USB Serial Device' in p.description  # Adjust this condition based on your Arduino's description
        ]
        num = 0 # default set to the first port
        if not arduino_ports:
            raise IOError("No Arduino found. Please make sure it's connected.")
        elif len(arduino_ports) > 1:
            print("Multiple Arduinos found:")
            for index, ard in enumerate(arduino_ports):
                print(str(index) + ": " + ard)
            temp_flag = True
            while temp_flag:
                try:
                    num = int(input("Please enter the number of the port you want to use: "))
                except ValueError:
                    print("Please enter a valid number")
                    continue
                if(num > len(arduino_ports) or num <= 0):
                    print("Please enter a valid number")
                    continue
                else:
                    temp_flag = False
        self.port = arduino_ports[num]
    
    def initiate(self):
        '''Start up routine of the arduino'''
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
        '''possible commands: reboot, center, pid, measure, NR, setSpeed, freq_scan'''
        self.command = input() + "\n"
        print("")
        self.board.write(self.command.encode('ASCII'))
        
    def send_message(self, message):
        '''Send a message to the arduino'''
        self.message = message
        self.board.write(message.encode('ASCII'))
        
    def send_input_message(self, save_to_omega = True):
        '''Send a message to the arduino, the message is input by the user'''
        self.message = input() + "\n"
        print("")
        self.board.write(self.message.encode('ASCII'))
        if(save_to_omega):
            self.omega = self.message
    
    def send_list_omega(self):
        '''Send a list of frequencies / a single frequency to the arduino'''
        temp_flag = True
        while(temp_flag):
            try:
                num = int(input("Number of frequencies to scan simultaneously (maximum 10): "))
            except ValueError:
                print("\nPlease enter a valid number")
                continue
            if(num == 1):
                print("\nPlease enter the driven frequency: \n")
                self.send_input_message()
                temp_flag = False
                return False
            elif(num >= 2 and num <= 10): 
                try:
                    start_point, end_point = (input("\nPlease enter the start and end frequency in this format (a,b): ").split(','))
                except ValueError:
                    print("\nPlease enter a valid range")
                    continue
                try:
                    start_point = float(start_point)
                    end_point = float(end_point)
                except ValueError:
                    print("\nPlease enter valid numbers")
                    continue
                
                if(end_point <= start_point):
                    print("\nPlease enter a valid range")
                    continue
                else:
                    msg = ""
                    omega_list = np.linspace(start_point, end_point, num, dtype = float)
                    for i in range(num - 1):
                        msg += "%.3f" % omega_list[i] + ","
                    msg += "%.3f" % omega_list[num - 1] + "\n"
                    temp_flag_check = True
                    while(temp_flag_check):
                        print("\nThe frequency list is: ", msg)
                        print("\nThe minimum spacing between frequencies is: %.3f" % ((end_point - start_point) / (num - 1)))
                        print("\nTo obtain nice phase calculation results, 1 / (fft_length * sampling_div) should be smaller than this")
                        temp = input("\nIs this what you want? (y/n): ")
                        if(temp == "y"):
                            self.send_message(msg)
                            print("")
                            self.omega_list = omega_list
                            temp_flag = False
                            temp_flag_check = False
                            return True
                        elif(temp == "n"):
                            temp_flag_check = False
                        else:
                            print("\nPlease enter a valid input")
            else:
                print("\nInvalid input, please enter an integer between 1 and 10")
        
    def read_single(self, prt = True, in_waiting = True):
        '''Read a single line from the arduino, in_waiting for blocking the program 
        until a line is received'''
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
      