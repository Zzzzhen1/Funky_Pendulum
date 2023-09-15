// General description:


// Include all the necessary head files
#include "Arduino.h"
#include "AS5600.h"
#include "Wire.h"
#include "math.h"
#include "AccelStepper.h"
#include "ezButton.h"

// Initialization of Arduino Board
const int baudrate = 230400;

// General intialization parameters
#define MAX_SIZE 20  // Maximum size of the input array

// This motor model (NEMA 17) is a typical bipolar 1.8deg (200steps/rev) motor.
// Therefore the corresponding microstep is 200 / STEPS, which needs to be taken into account
// Also, there's a maximum amount of steps each loop can take to avoid jittering! Will be tested later
#define STEPS 400 // Number of steps per revolution [Currently microsteps have not been implemented, so the position would be integer steps]
const int step_fac = int(STEPS / 200);  // Think of the configuration as 200 steps per revolution, but with 200/STEPS as minimum division.
const int max_steps = 200;              // Maximum number of steps per loop, which also determines the precision of cart position per loop.

// Define the stepper using accelstep
#define step_pin 8      // Connect to the PUL pin on the driver
#define step_vcc_pin 9  // Connect to constant 5V output PIN
#define dir_pin 10      // Connect to the DIR pin on the driver
#define dir_vcc_pin 11  // Connect to constant 5V output PIN
AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);

// Stepper related variables
int distance = 0;                // Total Rail distance
int pos_L = 0;                   // Position of the left switch
int pos_R = 0;                   // Position of the right switch
float pos_cart = 0;              // Current position of the cart. Changing in every cycle
float pos_cart_prev = 0;         // Previous position of the cart. Used to update the new move function
float pos_cart_target = 0;       // Target position of the cart. Won't change until the target has been reached.
float pos_integ = 0;             // Integral control bit for position
float vel = 0;                   // Cart velocity calculated using buffer arrays.
const float speed_lim = 5000.0;  // Maximum speed, also the speed for the run() method
float run_speed = 1000.0;        // Speed that the stepper normally runs at
const float accel = 120000.;    // Pre-set acceleration
long int safe_steps = 50;        // Safe distance to both switches
const float safe_speed = 500.0;  // Avoid crushing into the swtich too hard
float steps = 0.;                // Target position or relative movement, depends on the situation
float temp_speed = 5000.;        // Temporal speed limit
float temp_accel = 120000.;     // Temporal acceleration

// NR stage variable
double omega = 0.;             // Driven frequency of the cart
const int freq_size = 10;      // Maximum number of frequencies this cart can run at
double omega_list[freq_size];  // Multiple driven frequencies of the cart
float amp = 0.;                // Initial sinosuidal amplitude (steps) used for NR amplitude scan
float amp_0 = 50.;             // Initial sinosuidal amplitude (steps) used for constant oscillation
const float amp_swing = 100.;  // Constant swing up amplitude (steps) [Currently disabled]
float phase = 0.;              // Initial angle phase (in terms of the cart position)

// Buffer related variables and arrays
int buf_ind = 0;                                 // Buffer index of the loop. Keeps increasing over the time
int current_ind = 0;                             // Current index of the buffer array. current_ind = buf_ind % buf_len
int temp_ind = 0;                                // Temporary index variable
const int buf_len = 100;                         // Length of the buffer arrays
const int circ_buf_len = 2 * buf_len;            // Length of the circular buffer arrays
const int pos_vel_len = 2;                       // Number of points to calculate velocity (see get_velocity())
double circ_buffer_time[circ_buf_len] = { 0 };   // Circular buffer of the time to recover the sample history.
double circ_buffer_angle[circ_buf_len] = { 0 };  // Circular buffer of the angle to recover angular history
int circ_buffer_position[circ_buf_len] = { 0 };  // Circular buffer of the cart position to recover the position history.

// PID parameters [Needs another fine tuning because the setup for the motor has changed]
float Kp = 1000;       //Reasonable: 300-2000
float Ki = 0;          //Reasonable: 300-1700
float Kd = 4;          //Reasonable: 2-10
float Kp_pos = -0.05;  //Reasonable: 0.001-0.1
float Ki_pos = 0.04;   //Reasonable: 0.001-0.1
float Kd_pos = -0.01;  //Reasonable: 0.0001-0.005

// Define the button
#define Lbtn_pin 5        // Left switch pin
#define Rbtn_pin 4        // Right switch pin
ezButton Lbtn(Lbtn_pin);  // Left switch initiation
ezButton Rbtn(Rbtn_pin);  // Right switch initiation

//Equilibrium measuring and time-related variables
double current_time = 0.;            // Current time in seconds
double previous_time = 0.;           // Previous time in seconds
const long sample_div = 50;          // Sampling division. Default: 50 ms --> sampling rate ~= 20 Hz
unsigned long sample_time = 0;       // Sampling time variable to enable sampling rate.
unsigned long sample_time_prev = 0;  // Sampling time variable to enable sampling rate.

// Button state change detect
bool state_Lbtn = 1;       // Instant state of the left button
bool state_Rbtn = 1;       // Instant state of the right button
bool state_Lbtn_prev = 1;  // State of the left button of the previous loop
bool state_Rbtn_prev = 1;  // State of the right button of the previous loop
bool state_L = 1;          // Proper state variable that detects the change of state of the left button
bool state_R = 1;          // Proper state variable that detects the change of state of the right button

// Initialization of communication
String message = "";  // Used for receiving general messages
String junk = "";     // Junk message string
String cmd = "";      // Command string

// Useful counting variables
int center_count = 0;

// Flag indication of different stages
// During reset, these flags need to be set to original values
bool flag_command = 1;  // Receiving command from PC
int int_cmd;
bool flag_reset = 0;          // Reset command flag
bool flag_center = 0;         // Center command flag
bool flag_measure = 0;        // Angle Measure command flag
bool flag_setSpeed = 0;       // Test out the max speed and then set the speed_lim for the stepper motor -- command flag
bool flag_freq_scan = 0;      // Frequency Scan command flag
bool flag_pid = 0;            // PID Control command flag
bool flag_NR = 0;             // Normalised resonance command flag
bool flag_print_command = 1;  // Print the command once
bool flag_exit = 0;           // Exit current stage. Go back to function selection menu
bool flag_init_ang_cul = 1;   // Detect the initial cumulative angle --> used to the zero the angle measurement
// Center stage
bool flag_L = 1;  // Center stage flag for left switch
bool flag_R = 1;  // Center stage flag for right switch
// NR stage
bool flag_omega = 1;  // Receive a input command
bool flag_amp = 1;    // Receive a input command

// PID stage
bool flag_swing_request = 1;  // Default: ask for swing up and ask for parameter?
bool flag_swing = 0;          // Whether execute swing up strategy. Default: no swing up strategy
bool flag_pid_input = 1;      // Default: ask for new parameters
bool flag_eq_measure = 1;     // Measure the equilibrium angle
bool flag_switch_regime = 1;  // Switch to inversion control (with swing up strategy)
// SetSpeed stage
bool flag_setSpeed_request = 1;

// Initialize the angle sensor
void init_angle_sensor() {
  //any AS5600 specific initiations to be called here
}
AS5600 angle_sensor;
float ang = 0;            // Angle measurement
double ang_cul = 0;       // Cumulative angle measurement
double init_ang_cul = 0;  // Initial cumulative angle measurement
double ang_vel = 0;       // Angular velcity measurement
double ang_dev = 0;       // Angle deviation
double ang_eq = 0;        // Equilibrium angle
double ang_integ = 0;     // Integral control bit for angle

// Initialize the stepper
void init_stepper(float spd_lim, float accl) {
  pinMode(step_vcc_pin, OUTPUT);
  pinMode(dir_vcc_pin, OUTPUT);
  digitalWrite(step_vcc_pin, HIGH);
  digitalWrite(dir_vcc_pin, HIGH);
  stepper.setSpeed(safe_speed);
  stepper.setMaxSpeed(spd_lim);
  stepper.setAcceleration(accl);
  stepper.setCurrentPosition(0);
}

// Initialize the buttons
void init_button() {
  // Initialize the debounce time of the buttons
  Lbtn.setDebounceTime(10);
  Rbtn.setDebounceTime(10);
}

// Print the menu through the serial
void menu_print() {
  Serial.println("Cart pendulum functions: ");
  Serial.println("Enter 0 to reset the arduino board.");
  Serial.println("Enter 1 to begin the cart position centering.");
  Serial.println("Enter 2 to begin the natural frequency and quality factor measuring.");
  Serial.println("Enter 3 to test max running speed and max acceleration.");
  Serial.println("Enter 4 to begin the frequency scan.");
  Serial.println("Enter 5 to begin the PID control of inverted pendulum.");
  Serial.println("Enter 6 to begin the normalised resonance.");
}

// Begin execution of command print
void command_print(int num) {
  if (flag_print_command) {
    flag_print_command = 0;
    switch (num) {
      case 0:
        Serial.println("Resetting...");
        break;
      case 1:
        Serial.println("Begin centering.");
        break;
      case 2:
        Serial.println("Begin the natural frequency and quality factor measuring.");
        break;
      case 3:
        Serial.println("Begin the speed and acceleration setting.");
        break;
      case 4:
        Serial.println("Begin the frequency scan.");
        break;
      case 5:
        Serial.println("Begin the PID control.");
        break;
      case 6:
        Serial.println("Begin the normalised resonance.");
        break;
    }
    delay(500);
  }
}

// Reset the flags to original state
void reset(bool center = true) {
  flag_command = 1;
  flag_reset = 0;
  flag_center = 0;
  flag_measure = 0;
  flag_setSpeed = 0;
  flag_freq_scan = 0;
  flag_pid = 0;
  flag_NR = 0;
  flag_print_command = 1;
  flag_exit = 0;
  flag_init_ang_cul = 1;
  flag_L = 1;
  flag_R = 1;
  flag_omega = 1;
  flag_amp = 1;
  flag_swing_request = 1;
  flag_swing = 0;
  flag_pid_input = 1;
  flag_eq_measure = 1;
  flag_switch_regime = 1;
  flag_setSpeed_request = 1;
  if (center) {
    center_count = 0;
  }
  amp = 0.;
  amp_0 = 50.;
  buf_ind = 0;
  temp_ind = 0;
  temp_speed = speed_lim;
  temp_accel = accel;
  // init_stepper(speed_lim, accel); // TODO: check if this is necessary
  memset(circ_buffer_angle, 0., sizeof(circ_buffer_angle));
  memset(circ_buffer_time, 0., sizeof(circ_buffer_time));
  memset(circ_buffer_position, 0., sizeof(circ_buffer_position));
  memset(omega_list, 0., sizeof(omega_list));
  ang_eq = 0.;
  init_ang_cul = 0.;
  sample_time = 0;
  sample_time_prev = 0;
  Serial.flush();
}

// Make sure there is only one command being executed each loop
void single_command_check() {
  int sum = flag_reset + flag_center + flag_pid + flag_measure + flag_NR + flag_setSpeed;
  switch (sum) {
    case 0:
      Serial.println("No command detected.");
      delay(500);
      flag_command = 1;
      break;
    case 1:
      break;
    default:
      Serial.println("More than one command detected. Resetting the values.");
      delay(500);
      reset();
      break;
  }
}

// Reset the cart. This is a safety measure.
void cart_reset(bool kill = true) {
  stepper.stop();
  delay(500);
  stepper.setSpeed(safe_speed);
  stepper.runToNewPosition(0);
  // Final moment print to trigger stage changes in laptop
  if (kill) {
    Serial.println("Kill switch hit.");
  }
}

// Check whether a input string is fully made of numbers
bool isNumber(String str) {
  for (int i = 0; i < str.length(); i++) {
    if (!isDigit(str.charAt(i))) return false;
  }
  return true;
}

// Check whether a input string is a float number
bool isFloat(String str, bool neg = false) {
  int dot_count = 0;
  int neg_count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == '.') {
      dot_count += 1;
    } else if (str.charAt(i) == '-') {
      neg_count += 1;
    } else if (!isDigit(str.charAt(i))) {
      return false;
    }
    if (dot_count > 1) return false;
    if (neg_count > 1) return false;
    if (neg_count == 1 && !neg) return false;
  }
  return true;
}

// Used in the setSpeed stage, appoint the input two values 
bool isTwoFloat(String str){
  String temp_str = "";
  int comma_count = 0;
  for(int i = 0; i < str.length(); i++){
    if(comma_count > 1){
      temp_speed = speed_lim;
      temp_accel = accel;
      return false;
    }
    if(str.charAt(i) == ','){
      comma_count += 1;
      if(isFloat(temp_str)){
        temp_speed = temp_str.toFloat();
      }else{
        temp_speed = speed_lim;
        temp_accel = accel;
        return false;
      }
      temp_str = "";
    }else{
      temp_str += str.charAt(i);
    }
  } 
  if(isFloat(temp_str)){
    temp_accel = temp_str.toFloat();
  }else{
    temp_speed = speed_lim;
    temp_accel = accel;
    return false;
  }
  return true;
}

// Check whether the input is made of multiple frequencies
bool isMultiFreq(String str) {
  String temp_str = "";
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == ',') {
      if (count >= freq_size) {
        memset(omega_list, 0., sizeof(omega_list));
        return false;
      }
      if (isFloat(temp_str)) {
        omega_list[count] = 2 * M_PI * temp_str.toDouble();
        count += 1;
      } else {
        memset(omega_list, 0., sizeof(omega_list));
        return false;
      }
      temp_str = "";
    } else {
      temp_str += str.charAt(i);
    }
  }
  // Final conversion
  omega_list[count] = 2 * M_PI * temp_str.toDouble();
  return true;
}

// Receive the input made up of multiple float numbers. Return the pointer to the float array
// [First time using the pointer in C++. It would make the code much clearer if other part 
// of the code can be converted to pointer as well.]
float* isMultiFloat(String str){
  String temp_str = "";
  static float temp_float[MAX_SIZE] = {0};
  int count = 0;
  for(int i = 0; i < str.length(); i++){
    if(str.charAt(i) == ','){
      if(count >= MAX_SIZE){
        return nullptr;
      }
      if(isFloat(temp_str)){
        temp_float[count] = temp_str.toFloat();
        count += 1;
      }else{
        return nullptr;
      }
      temp_str = "";
    }else{
      temp_str += str.charAt(i);
    }
  }
  return temp_float;
}

// Read message from Serial port, wait infinitely
String read_msg() {
  while (Serial.available() == 0) {}
  message = Serial.readStringUntil('\n');
  message.trim();
  return message;
}

// Read message without waiting, has timeout feature
String read_ready_msg() {
  message = Serial.readStringUntil('\n');
  message.trim();
  return message;
}

// Read command and turns up command flags
String read_cmd() {
  Serial.flush();
  while (Serial.available() == 0) {}
  message = Serial.readStringUntil('\n');
  message.trim();
  flag_command = 0;
  if (isNumber(message)) {
    int_cmd = message.toInt();
    switch (int_cmd) {
      case 0:
        flag_reset = 1;
        return "Reset";
      case 1:
        flag_center = 1;
        return "Center";
      case 2:
        flag_measure = 1;
        return "Measure";
      case 3:
        if (center_count >= 1) {
          flag_setSpeed = 1;
          return "setSpeed";
        } else {
          flag_command = 1;
          delay(500);
          Serial.println("Hasn't been centered. Please Center the cart first.");
          delay(500);
          return "";
        }
      case 4:
        flag_freq_scan = 1;
        return "freqScan";
      case 5:
        flag_pid = 1;
        return "PID";
      case 6:
        flag_NR = 1;
        return "NR";
      default:
        flag_command = 1;
        delay(500);
        Serial.println("Unidentified command. Please try again.");
        delay(500);
        return "";
    }
  } else {
    if (message == "connection") {
      flag_command = 1;
      delay(500);
      Serial.println("Successfully Connected");
      delay(500);
      return message;
    } else if (message == "Terminate") {
      Serial.println("Terminating...");
      reset();
    }
    flag_command = 1;
    delay(500);
    Serial.println("Unidentified command. Please try again.");
    delay(500);
    return "";
  }
}

// Recycle variable in case there is any useful information
String collect_junk_serial() {
  if (Serial.available()) {
    junk = Serial.readStringUntil('\n');
  }
  Serial.flush();
}

// Principal set up of the arduino board and serial connection
void setup() {
  Serial.begin(baudrate);
  Serial.setTimeout(5000);
  // Communication begin routine //
  while (Serial.available() == 0) {}
  Serial.println("Successfully Connected");
  delay(500);
  junk = collect_junk_serial();
  // Communication begin routine //
  Wire.begin();
  init_angle_sensor();
  init_stepper(speed_lim, accel);
  init_button();
}

// Principal loop
void loop() {
  if (flag_command) {
    if (Serial) {
      menu_print();
      cmd = read_cmd();
      if (flag_command == 0) {
        single_command_check();
        command_print(int_cmd);
      }
    } else {
      message = read_msg();
      if (message == "connection") {
        Serial.println("Successfully Connected");
        delay(500);
      }
    }
  } else {
    // Initiate the button in the loop
    Lbtn.loop();
    Rbtn.loop();
    // 1 for released and 0 for pressed
    state_Lbtn = Lbtn.getState();
    state_Rbtn = Rbtn.getState();
    state_L = state_change_L();
    state_R = state_change_R();

    if (flag_reset) {
      reset();
    } else if (flag_center) {
      center();
    } else if (flag_pid) {
      pid();
    } else if (flag_measure) {
      measure();
    } else if (flag_NR) {
      NR();
    } else if (flag_setSpeed) {
      setSpeed();
    } else if (flag_freq_scan){
      freq_scan();
    }
    state_Lbtn_prev = state_Lbtn;
    state_Rbtn_prev = state_Rbtn;
  }
}

// Some control functions
// Runs the cart centering
void center() {
  if (Serial) {
    if (state_L && state_R) {
      if (flag_L && flag_R) {
        stepper.setSpeed(safe_speed);
        stepper.move(-1);
        stepper.runSpeedToPosition();
      } else if (!flag_L && flag_R) {
        stepper.setSpeed(safe_speed);
        stepper.move(1);
        stepper.runSpeedToPosition();
      } else if (!flag_L && !flag_R) {
        // Calculate and move to the central position. Then, set this position as zero.
        distance = pos_R - pos_L;
        stepper.setSpeed(run_speed);
        stepper.runToNewPosition(pos_L + int(distance / 2));
        flag_center = 0;
        center_count += 1;
        flag_command = 1;
        Serial.print(center_count);
        Serial.print(",");
        Serial.print(distance);
        Serial.println("");
        stepper.setCurrentPosition(0);
        delay(1000);
        reset(false);
        init_stepper(speed_lim, accel);
      }
    } else if (!state_L) {
      pos_L = stepper.currentPosition();
      stepper.move(1);
      stepper.runSpeedToPosition();
      stepper.stop();
      stepper.setSpeed(safe_speed);
      stepper.runToNewPosition(pos_L + safe_steps);
      flag_L = 0;
    } else if (!state_R) {
      pos_R = stepper.currentPosition();
      stepper.move(-1);
      stepper.runSpeedToPosition();
      stepper.stop();
      stepper.setSpeed(safe_speed);
      stepper.runToNewPosition(pos_R - safe_steps);
      flag_R = 0;
    } else {
      stepper.setSpeed(run_speed);
    }
  } else {
    reset();
  }
}

// Runs of the PID control of the pendulum
void pid() {
  if (Serial) {
    if (flag_swing_request) {
      Serial.println("Do you want to turn up swing up strategy? type in (y/n)");
      message = read_msg();
      if (message == "y") {
        Serial.println("Continue with swing up strategy.");
        flag_swing = 1;
        flag_swing_request = 0;
        stepper.move(200);
        delay(500);
      } else if (message == "n") {
        Serial.println("Continue without swing up strategy.");
        flag_swing = 0;
        flag_swing_request = 0;
        delay(500);
      } else if (message == "Terminate") {
        Serial.println("Terminate the process.");
        cart_reset();
        reset();
      } else {
        delay(500);
        Serial.println("Invalid command, please try again.");
        delay(500);
      }
    } else {
      if (flag_pid_input) {
        pid_print();
        Serial.println("");
        Serial.println("Resume (ENTER r) or ENTER six numbers split by commas without spaces");
        Serial.println("For example: 1000,0,4,-0.05,0.04,-0.01");
        Serial.println("In this order:Kp_ang,Ki_ang,Kd_ang,Kp_pos,Ki_pos,Kd_pos");
        Serial.println("[Scroll up to see previous values]");
        Serial.println("Before press ENTER, make sure either the pendulum is stable at downright or upright position!");
        message = read_msg();
        if (pid_receive()) {
          pid_print();
          Serial.println("Start inversion control.");
          flag_pid_input = 0;
        } else if (message == "Terminate") {
          Serial.println("Terminate the process.");
          cart_reset();
          reset();
        } else {
          delay(500);
          Serial.println("Invalid input, please Try Again");
          delay(500);
        }
      } else {
        if (flag_eq_measure) {
          if (flag_swing) {
            ang_eq = get_cumulative_angle();
          } else {
            ang_eq = get_cumulative_angle();
          }
          ang_cul = get_cumulative_angle();
          ang_eq = rectify_angle();
          flag_eq_measure = 0.;
        } else {
          pid_control_run(flag_swing);
        }
      }
    }
  } else {
    cart_reset();
    reset();
  }
}

// PID stage stepper control, print through serial: time, angle and position (all with velocity)
void pid_control_run(bool flag) {
  if (state_L && state_R) {
    current_ind = buf_ind % buf_len;
    current_time = millis() / 1000.;
    sample_time = millis();
    ang_cul = get_cumulative_angle();
    ang_eq = rectify_angle();
    ang_dev = ang_cul - ang_eq;
    ang_vel = get_angular_velocity();

    cart_run_max();

    pos_cart = stepper.currentPosition();
    pos_cart_target = stepper.targetPosition();
    vel = get_velocity();

    cart_run_max();

    circ_buffer_position[current_ind] = pos_cart;
    circ_buffer_angle[current_ind] = ang_cul;
    circ_buffer_time[current_ind] = current_time;
    circ_buffer_position[current_ind + buf_len] = pos_cart;
    circ_buffer_angle[current_ind + buf_len] = ang_cul;
    circ_buffer_time[current_ind + buf_len] = current_time;

    if (sample_time - sample_time_prev >= sample_div) {
      sample_time_prev = sample_time;
      Serial.print(current_time, 5);
      Serial.print(",");
      Serial.print(ang_dev, 4);
      Serial.print(",");
      Serial.print(pos_cart, 1);
      Serial.print(",");
      Serial.print(ang_vel, 4);
      Serial.print(",");
      Serial.print(vel, 4);
      Serial.println("");
    }

    if (flag) {
      // with swing up strategy
      if (flag_switch_regime) {
        // Do the swing up
        steps = int(amp_swing * (ang_dev));
        stepper.move(steps);
        cart_run_max();
        if (abs(abs(ang_dev) - M_PI) < 0.05 * M_PI && abs(ang_vel) < 0.5) {
          ang_eq = ang_eq + M_PI;
          ang_eq = rectify_angle();
          flag_switch_regime = 0;
        } else if (abs(abs(ang_dev) - M_PI) < 0.05) {
          cart_reset(false);
          delay(5000);  // Wait for manual stablisation
        }
      } else {
        // Do the control
        if (Ki != 0.) {
          ang_integ = ang_integ_calc();
        }
        if (Ki_pos != 0.) {
          pos_integ = pos_integ_calc();
        }
        steps = int(Kp * ang_dev + Ki * ang_integ + Kd * ang_vel - Kp_pos * pos_cart - Ki_pos * pos_integ - Kd_pos * vel);
        stepper.move(steps);
        cart_run_max();
      }
    } else {
      // without swing up strategy
      // Do the control
      if (Ki != 0.) {
        ang_integ = ang_integ_calc();
      }
      if (Ki_pos != 0.) {
        pos_integ = pos_integ_calc();
      }
      steps = int(Kp * ang_dev + Ki * ang_integ + Kd * ang_vel - Kp_pos * pos_cart - Ki_pos * pos_integ - Kd_pos * vel);
      stepper.move(steps);
      cart_run_max();
    }
    pos_cart_target = stepper.targetPosition();
    if (pos_cart_target <= -int(distance / 2) + safe_steps || pos_cart_target >= int(distance / 2) - safe_steps) {
      cart_reset();
      reset();
    }
    buf_ind += 1;
    previous_time = current_time;  // Record the begin time of this loop. For test Only.
  } else {
    cart_reset();
    reset();
  }
}

// Runs the angle measurement
void measure() {
  if (Serial) {
    ang_cul = get_cumulative_angle();
    previous_time = current_time;
    current_time = micros() / 1000000.;
    sample_time = millis();
    if (sample_time - sample_time_prev >= sample_div) {
      sample_time_prev = sample_time;
      Serial.print(current_time, 6);
      Serial.print(",");
      Serial.println(ang_cul, 4);
    }
  } else {
    reset();
  }
}

// Performs the init_stepper function, and then do the sinusoidal motion
void setSpeed() {
  if (Serial) {
    if (flag_setSpeed_request) {
      Serial.print("Current speed: ");
      Serial.print(temp_speed, 1);
      Serial.print(" step/s Current acceleration: ");
      Serial.print(temp_accel, 1);
      Serial.println(" step/s^2");
      Serial.println("Type in the values for speed and acceleration separated by a comma without spaces:");
      Serial.println("[Note: in this stage, the cart will be driven at 1 Hz.]");
      message = read_msg();
      if (message == "Terminate") {
        Serial.println("Terminating...");
        cart_reset();
        reset();
      } else if (isTwoFloat(message)) {
        flag_setSpeed_request = 0;
        init_stepper(temp_speed, temp_accel);
        Serial.print("Start sinusoidal motion with speed ");
        Serial.print(temp_speed, 1);
        Serial.print(" step/s and acceleration ");
        Serial.print(temp_accel, 1);
        Serial.print(" step/s^2");
        Serial.println("");
        delay(500);
      } else {
        delay(500);
        Serial.println("Invalid input, please try again");
        delay(500);
      }
    } else {
      if(flag_amp){
        Serial.print("Current amplitude: ");
        Serial.print(amp_0, 1);
        Serial.println(" steps.");
        Serial.println("Type in the amplitude for the following sinusoidal oscillation in steps:");
        message = read_msg();
        if(isFloat(message)){
          amp_0 = message.toFloat();
          Serial.print("Start with amplitude: ");
          Serial.print(amp_0, 1);
          Serial.println(" steps.");
          flag_amp = 0;
          }else if(message == "Terminate"){
            Serial.println("Terminate the process.");
            cart_reset();
            reset();
          }else{
            Serial.println("Invalid input, please try again.");
          }
      }else{
        // Do the sinusoidal motion based on the distance value!!
        // Read out the position and the velocity of the cart.
        if(state_L && state_R){
          current_ind = buf_ind % buf_len;
          current_time = millis() / 1000.;
          sample_time = millis();
          pos_cart = stepper.currentPosition();
          pos_cart_target = stepper.targetPosition();
          vel = get_velocity();
          
          circ_buffer_position[current_ind] = pos_cart;
          circ_buffer_time[current_ind] = current_time;
          circ_buffer_position[current_ind + buf_len] = pos_cart;
          circ_buffer_time[current_ind + buf_len] = current_time;

          cart_run_max();
          if(sample_time - sample_time_prev >= sample_div){
            sample_time_prev = sample_time;
            Serial.print(current_time, 3);
            Serial.print(",");
            Serial.print("0.0");
            Serial.print(",");
            Serial.print(pos_cart, 1);
            Serial.print(",");
            Serial.print("0.0");
            Serial.print(",");
            Serial.print(vel, 1);
            Serial.println("");
          }
          steps = amp_0 * sin(2 * M_PI * current_time);
          cart_run_max();
          stepper.moveTo(steps);
          cart_run_max();
          cart_run_max();
          buf_ind += 1;
        }else{
          cart_reset();
          reset();
        }
      }
    }
  } else {
    cart_reset();
    reset(false);
  }
}

// Runs the scanning for response stage, pretty much the duplicate of the NR stage
void freq_scan() {
  if (Serial) {
    if (state_L && state_R) {
      if (flag_omega) {
        // receive a frequency value
        Serial.println("Input a frequency value for driving the cart: (Hz)");
        message = read_msg();
        if (isFloat(message)) {
          omega = message.toFloat() * 2 * M_PI;
          memset(omega_list, 0., sizeof(omega_list));
          omega_list[0] = omega;
          Serial.print("Start with driven frequency: ");
          Serial.print(message);
          Serial.println(" Hz");
          flag_omega = 0;
        } else if (message == "Terminate") {
          Serial.println("Terminate the process.");
          cart_reset();
          reset();
        } else if (isMultiFreq(message)) {
          Serial.print("Starting with these frequencies (in Hz): ");
          for (int i = 0; i < freq_size; i++) {
            if (omega_list[i] == 0) {
              break;
            } else {
              Serial.print(omega_list[i] / 2 / M_PI, 4);
              Serial.print(" ");
            }
          }
          Serial.println("");
          flag_omega = 0;
        } else {
          Serial.println("Invalid input, please try again.");
        }
      } else {
        if(flag_amp){
          Serial.print("Current amplitude: ");
          Serial.print(amp_0, 1);
          Serial.println(" steps.");
          Serial.println("Type in the amplitude for the following sinusoidal oscillation in steps:");
          message = read_msg();
          if(isFloat(message)){
            amp_0 = message.toFloat();
            Serial.print("Start with amplitude: ");
            Serial.print(amp_0, 1);
            Serial.println(" steps.");
            flag_amp = 0;
            }else if(message == "Terminate"){
              Serial.println("Terminate the process.");
              cart_reset();
              reset();
            }else{
              Serial.println("Invalid input, please try again.");
            }
        } else {
          ang_cul = get_cumulative_angle();
          previous_time = current_time;
          sample_time = millis();
          current_time = sample_time / 1000.;
          pos_cart = stepper.currentPosition();
          pos_cart_target = stepper.targetPosition();

          if (sample_time - sample_time_prev >= sample_div) {
            sample_time_prev = sample_time;
            Serial.print(current_time, 3);
            Serial.print(",");
            Serial.print(ang_cul, 4);
            Serial.print(",");
            Serial.print(pos_cart);
            Serial.println("");
          }

          if (Serial.available() == 0) {
            // do the running
            // steps = (amp * sin(omega * current_time + phase) + amp_0 * sin(omega * current_time));
            steps = 0;
            for (int i = 0; i < freq_size; i++) {
              if (omega_list[i] != 0) {
                steps += amp_0 * sin(omega_list[i] * current_time);
              }
            }
            steps += amp * sin(omega * current_time + phase);
            cart_run_max();
            stepper.moveTo(steps);
            cart_run_max();
          } else {
            // receive data from laptop
            message = read_ready_msg();
            cart_run_max();
            NR_receive();
            cart_run_max();
          }
        }
      }
    } else {
      cart_reset();
      reset();
    }
  } else {
    reset();
    cart_reset();
  }
}

// The function that performs Normalised Resonance experiment
void NR() {
  if (Serial) {
    if (state_L && state_R) {
      if (flag_omega) {
        // receive a frequency value
        Serial.println("Input a frequency value for driving the cart: (Hz)");
        message = read_msg();
        if (isFloat(message)) {
          omega = message.toFloat() * 2 * M_PI;
          memset(omega_list, 0., sizeof(omega_list));
          omega_list[0] = omega;
          Serial.print("Start with driven frequency: ");
          Serial.print(message);
          Serial.println(" Hz");
          flag_omega = 0;
        } else if (message == "Terminate") {
          Serial.println("Terminate the process.");
          cart_reset();
          reset();
        } else if (isMultiFreq(message)) {
          Serial.print("Starting with these frequencies (in Hz): ");
          for (int i = 0; i < freq_size; i++) {
            if (omega_list[i] == 0) {
              break;
            } else {
              Serial.print(omega_list[i] / 2 / M_PI, 4);
              Serial.print(" ");
            }
          }
          Serial.println("");
          flag_omega = 0;
        } else {
          Serial.println("Invalid input, please try again.");
        }
      } else {
        if(flag_amp){
          Serial.print("Current amplitude: ");
          Serial.print(amp_0, 1);
          Serial.println(" steps.");
          Serial.println("Type in the amplitude for the following sinusoidal oscillation in steps:");
          message = read_msg();
          if(isFloat(message)){
            amp_0 = message.toFloat();
            Serial.print("Start with amplitude: ");
            Serial.print(amp_0, 1);
            Serial.println(" steps.");
            flag_amp = 0;
            }else if(message == "Terminate"){
              Serial.println("Terminate the process.");
              cart_reset();
              reset();
            }else{
              Serial.println("Invalid input, please try again.");
            }
        } else {
          ang_cul = get_cumulative_angle();
          previous_time = current_time;
          sample_time = millis();
          current_time = sample_time / 1000.;
          pos_cart = stepper.currentPosition();
          pos_cart_target = stepper.targetPosition();

          if (sample_time - sample_time_prev >= sample_div) {
            sample_time_prev = sample_time;
            Serial.print(current_time, 3);
            Serial.print(",");
            Serial.print(ang_cul, 4);
            Serial.print(",");
            Serial.print(pos_cart);
            Serial.println("");
          }

          if (Serial.available() == 0) {
            // do the running
            // steps = (amp * sin(omega * current_time + phase) + amp_0 * sin(omega * current_time));
            steps = 0;
            for (int i = 0; i < freq_size; i++) {
              if (omega_list[i] != 0) {
                steps += amp_0 * sin(omega_list[i] * current_time);
              }
            }
            steps += amp * sin(omega * current_time + phase);
            cart_run_max();
            stepper.moveTo(steps);
            cart_run_max();
          } else {
            // receive data from laptop
            message = read_ready_msg();
            cart_run_max();
            NR_receive();
            cart_run_max();
          }
        }
      }
    } else {
      cart_reset();
      reset();
    }
  } else {
    reset();
    cart_reset();
  }
}

// Calculate the cart velocity using the circular buffer
float get_velocity() {
  double sum = 0;
  int temp_ind = current_ind + buf_len - 2 * pos_vel_len;
  if (buf_ind > pos_vel_len + 1) {
    for (int i = 1; i <= pos_vel_len; i++) {
      double temp_vel = (circ_buffer_position[temp_ind + pos_vel_len + i] - circ_buffer_position[temp_ind + i])
                        / (circ_buffer_time[temp_ind + pos_vel_len + i] - circ_buffer_time[temp_ind + i]);
      if (isnan(temp_vel)) {
        sum += 0;
      } else {
        sum += temp_vel;
      }
    }
    return sum / pos_vel_len;
  } else {
    return 0.;  // Have not had enough data to calculate the velocity
  }
}

// Angle measurement in rad, in the range of 0 to 2*pi
double get_angle() {
  double temp_ang = angle_sensor.readAngle() * AS5600_RAW_TO_RADIANS;
  return temp_ang;
}

// Cumulative angle measurement
double get_cumulative_angle() {
  double temp_ang = angle_sensor.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
  if (flag_init_ang_cul) {
    init_ang_cul = temp_ang;
    flag_init_ang_cul = 0;
    return 0;
  } else {
    return temp_ang - init_ang_cul;
  }
}

// Angular speed measurement in rad/s
double get_angular_velocity() {
  double temp_ang_vel = angle_sensor.getAngularSpeed(1);
  if (buf_ind < pos_vel_len + 1) {
    return 0;
  } else {
    return temp_ang_vel;
  }
}

// Integral control calculation for angle
double ang_integ_calc() {
  if (buf_ind >= buf_len) {
    double sum = 0;
    double time = 0;
    for (int i = 2; i <= buf_len; i++) {
      sum += (circ_buffer_angle[current_ind + i] - ang_eq) * circ_buffer_time[current_ind + i]
             - (circ_buffer_angle[current_ind + i] - ang_eq) * circ_buffer_time[current_ind + i - 1];
    }
    time = circ_buffer_time[current_ind + buf_len] - circ_buffer_time[current_ind + 1];
    return sum / time;
  }
  return 0;
}

// Integral control calculation for position
double pos_integ_calc() {
  if (buf_ind >= buf_len) {
    double sum = 0;
    double time = 0;
    for (int i = 2; i <= buf_len; i++) {
      sum += double(circ_buffer_position[current_ind + i]) * circ_buffer_time[current_ind + i]
             - double(circ_buffer_position[current_ind + i]) * circ_buffer_time[current_ind + i - 1];
    }
    time = circ_buffer_time[current_ind + buf_len] - circ_buffer_time[current_ind + 1];
    return sum / time;
  } else {
    return 0;
  }
}

// Rectify the equilibrium angle in terms of the culmulative angle
double rectify_angle() {
  if (ang_eq - ang_cul >= 2 * M_PI) {
    ang_eq = ang_eq - 2 * M_PI;
  } else if (ang_eq - ang_cul < -2 * M_PI) {
    ang_eq = ang_eq + 2 * M_PI;
  }
  return ang_eq;
}

// Detect change of state of the left switch
bool state_change_L() {
  if (state_Lbtn == 0 && state_Lbtn_prev == 1) {
    return 0;
  } else {
    return 1;
  }
}

// Detect change of state of the right switch
bool state_change_R() {
  if (state_Rbtn == 0 && state_Rbtn_prev == 1) {
    return 0;
  } else {
    return 1;
  }
}

// Runs the cart one step and update the pos_cart
bool cart_run() {
  if (stepper.run()) {
    if (pos_cart > pos_cart_target) {
      pos_cart -= 1;
    } else {
      pos_cart += 1;
    }
    return 1;
  } else {
    return 0;
  }
}

// TODO: need to fix the run_speed() (secondary)
// Runs the cart one step at constant speed and update the pos_cart
bool cart_run_speed() {
  if (stepper.runSpeedToPosition()) {
    if (pos_cart > pos_cart_target) {
      pos_cart -= 1;
    } else {
      pos_cart += 1;
    }
    return 1;
  } else {
    return 0;
  }
}

// Runs the cart the max_steps and update the pos_cart
void cart_run_max() {
  if (flag_exit) {
    return;
  }
  for (int i = 0; i < max_steps; i++) {
    state_Lbtn = Lbtn.getState();
    state_Rbtn = Rbtn.getState();
    state_L = state_change_L();
    state_R = state_change_R();
    if (!state_L) {
      stepper.stop();
      flag_exit = 1;
      return;
    } else if (!state_R) {
      stepper.stop();
      flag_exit = 1;
      return;
    }
    if (!cart_run()) {
      return;
    }
    state_Lbtn_prev = state_Lbtn;
    state_Rbtn_prev = state_Rbtn;
  }
}

// Receive from laptop the comma separated data
void NR_receive() {
  int comma_index = message.indexOf(',');
  if (comma_index == -1) {
    return;
  } else {
    if (message.indexOf(',', comma_index + 1) != -1) {
      return;
    }
  }
  String s_amp = "";
  String s_phase = "";
  for (int i = 0; i < message.length(); i++) {
    if (i < comma_index) {
      s_amp += message[i];
    } else if (i > comma_index) {
      s_phase += message[i];
    }
  }
  amp = s_amp.toFloat();
  phase = s_phase.toFloat();
}

// Receive the comma separated data, 5 commas. Returns False if the input message is invalid
bool pid_receive() {
  if (message == "r") {
    return 1;
  }
  int comma_index[6] = { 0 };
  int len = message.length();
  int comma_num = 0;
  String s_Kp = "";
  String s_Ki = "";
  String s_Kd = "";
  String s_Kp_pos = "";
  String s_Ki_pos = "";
  String s_Kd_pos = "";

  for (int i = 1; i < 6; i++) {
    // comma_index[0] as a buffer
    // Make sure there are only five commas
    comma_index[i] = message.indexOf(',', comma_index[i - 1] + 1);
    if (comma_index[i] == -1) {
      return 0;
    }
  }
  if (message.indexOf(',', comma_index[5] + 1) != -1) {
    return 0;
  }
  for (int i = 0; i < len; i++) {
    if (i < comma_index[1]) {
      s_Kp += message[i];
    } else if (i > comma_index[1] && i < comma_index[2]) {
      s_Ki += message[i];
    } else if (i > comma_index[2] && i < comma_index[3]) {
      s_Kd += message[i];
    } else if (i > comma_index[3] && i < comma_index[4]) {
      s_Kp_pos += message[i];
    } else if (i > comma_index[4] && i < comma_index[5]) {
      s_Ki_pos += message[i];
    } else if (i > comma_index[5]) {
      s_Kd_pos += message[i];
    }
  }

  if (isFloat(s_Kp, true)) {
    Kp = s_Kp.toFloat();
  } else {
    return 0;
  }
  if (isFloat(s_Ki, true)) {
    Ki = s_Ki.toFloat();
  } else {
    return 0;
  }
  if (isFloat(s_Kd, true)) {
    Kd = s_Kd.toFloat();
  } else {
    return 0;
  }
  if (isFloat(s_Kp_pos, true)) {
    Kp_pos = s_Kp_pos.toFloat();
  } else {
    return 0;
  }
  if (isFloat(s_Ki_pos, true)) {
    Ki_pos = s_Ki_pos.toFloat();
  } else {
    return 0;
  }
  if (isFloat(s_Kd_pos, true)) {
    Kd_pos = s_Kd_pos.toFloat();
  } else {
    return 0;
  }

  return 1;
}

// Print current PID parameters
void pid_print() {
  Serial.println("Current parameters are:");
  Serial.print("Angle PID control: Kp = ");
  Serial.print(Kp, 4);
  Serial.print(" Ki = ");
  Serial.print(Ki, 4);
  Serial.print(" Kd = ");
  Serial.print(Kd, 4);
  Serial.println("");
  Serial.print("Cart PID control: Kp_pos = ");
  Serial.print(Kp_pos, 4);
  Serial.print(" Ki_pos = ");
  Serial.print(Ki_pos, 4);
  Serial.print(" Kd_pos = ");
  Serial.print(Kd_pos, 4);
  Serial.println("");
}

// TODO: add a jolt