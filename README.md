# Funky Pendulum

This is the continuation of the CartER projects undertaken by [Jeppe](https://github.com/JeppeKlitgaard/CartER/) and [Will](https://github.com/will-hd/CartER).

Based on the project accomplished by Zhen and Owen during 2023 summer. This repository mainly contains the control code and all the necessary components to construct one's own copy. Any questions about the code please feel free to contact [Zhen's cam email](mailto:zy336@cam.ac.uk) or [Zhen's gmail](mailto:yz20030303@gmail.com) with a proper subject or leave in dicussion session in this GitHub repository.

One advantage about this setup it is highly reproducible by using cheap microcontroller chip and PLA plastics. A list of components and purchase links will be included at the end of this README file.

## Getting Started

Download the zip file by clicking the **code** icon on the upper right corner. Download the following Python modules before running the code in the /src folder.

Necessary Python modules:
1. numpy
2. scipy
3. matplotlib
4. pyserial
5. pandas

use `pip install module_name` to configure the corresponding module for your python

Necessary Arduino Libraries:
1. AccelStepper
2. AS5600
3. EasyButton

For uploading the Arduino code to the Arduino Board, you need to either use Arduino IDE or PlatformIO (not compatible yet) to download the above libraries and then upload the code to the microcontroller.
 
## Useful Tips for Practical

It would require an admin right for any external device connection to classes laptop, so changing parts in the Arduino code would be impossible because you cannot upload any changes to the Arduino. You can still look at the Arduino however. Most parts are with a short description. 

Two useful shortcuts in VS Code are:
1. "Ctrl+K then Ctrl+0" to collapse all the codes
2. "Ctrl+k then Ctrl+J" to unfold all the collapsed part
This can help you to navigate certain part of the code

### Parts you can play with in the Python code are:

(inside the `if __name__ == "__main__` block of the Pendulum_Control_Console.py)
1. FFT parameters (fft_lengths and sampling_divs, as explained in the handout)
2. wait_to_stables (the Python code will go in a loop until you call an exit, this parameter decides the number of loops waited to update the amplitude and phase in **NR** stage)
3. NR_Kp and NR_Kd values and signs (NR_Ki has not been implemented yet) in the `data_phy()` class `__init__()` method

(of the methods in the `data_analysis()` class of the csv_process.py)
1. `measure_fit()` parameters (before handling with the paramters, you need to check out how `damp_sin()` function is defined)
2. `scan_fit()` parameters (check out how `sinusoid()` function is defined first)

### Some Interesting Results

Check out the [plots](https://github.com/Zzzzhen1/Funky_Pendulum/tree/main/processed_data/plots). They are produced using the data in the /processed_data folder and the csv_process.py. These plots characterise the non-linearity and the change of natural frequency with response amplitude. This is something you can try measuring during the practical.

## Experiment Setup

See [this branch](https://github.com/Zzzzhen1/Funky_Pendulum/blob/3D_file_branch(protected)/3D_Printing/README.md) about how to make use of the 3D printing files

Here are some pictures about the wiring and an example setup:

<figure>
    <figcaption>Full setup of the Funky Pendulum</figcaption>
    <img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/CartER_1.jpg" alt = "Full setup of the Funky Pendulum" width = "640" height = "360">
</figure>

---

<figure>
    <figcaption>Emergency stop button to cut off Arduino</figcaption>
    <img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/Emergency_button___motor_1.jpg" alt = "Emergency stop button to cut off Arduino" width = "640" height = "360">
</figure>

---

<figure>
    <figcaption>Power supply with 20-50V and variable current</figcaption>
    <img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/Power_supply_2.jpg" alt = "Power supply with 20-50V and variable current" width = "640" height = "360">
</figure>

---

<figure>
    <figcaption>AS5600 angle sensor fixed by a tape</figcaption>
    <img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/Rotary_encoder_1.jpg" alt = "AS5600 angle sensor fixed by a tape" width = "640" height = "360">
</figure>

---

<figure>
    <figcaption>DM542T stepper driver, specifications online</figcaption>
    <img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/Stepper_driver_3.jpg" alt = "DM542T stepper driver, specifications online" width = "640" height = "360">
</figure>

---

<figure>
    <figcaption>Overall wiring</figcaption>
    <img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/Wiring_1.jpg" alt = "Overall wiring" width = "640" height = "360">
</figure>

- TODO: attach a copy of practical handout here

## Proof of PID Inversion Control

Click the picture to see the inversion control video.

### First Inversion Control Video 

[<img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/CartER_1.jpg" width = "640" height = "360">](https://youtu.be/JiOYI30tvMM)

### Second Inversion Control Video (Full 3D construction, see Jeppe's docs)

[<img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/3D_file_branch(protected)/3D_Printing/Image/Full_Construction.PNG" width = "640" height = "360">](https://youtu.be/XKwGB6jRk7I)

## Potential Improvement for the Code
General:
1. not fully understood yet, but to keep things in consistent with the 
phase feedback(with an extra pi), phase_active is also added pi

From arduino_manager.py:
1. this is supposedly referring to the reset pin, however not working properly (secondary)

From data_process.py:
1. add a sampling rate selection stage in arduino (secondary)
2. add a different title for downward and upward control (secondary)
3. avoid pdf and csv saved with different names within the same run (secondary)

From Pendulum_Control_Console.py:
1. at the end of the program, ask whether to enter data analysis mode (secondary)
2. check whether the platformio can do the arduino code upload because the Arduino IDE would be inefficient and faulty (secondary)
4. **all the parameters in the code should have a reasonable range**
5. separate the different classes in different python files (secondary)
6. find delay time (a day of investigation) and make a plot (secondary)
7. the csv file saved from the csv_process.py needs a datetime in name (secondary)
9. pid data analysis --> stable time stop time (with threshold)... then plot stop_time vs. iteration graph (secondary)
10. add a uniform jolt to test the stability of this set of PID coefficients (secondary)
11. add a selection for the PID of normalised resonance (secondary)
12. decrease/increase plotting fps (secondary)
