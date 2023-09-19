# Initialisation of some constants and variables
port = 'COM6' 
baudrate = 230400 # TODO: extract all constants from a larger project file?
MAX_COUNT = 10 # Number of points waited to plot a frame TODO: change this to manipulate the fps
ANGLE_ROTATION = 55 # Rotation of the y-label

class data_frame():
    
    '''This class is a single data_frame to store data from arduino'''
    
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
         