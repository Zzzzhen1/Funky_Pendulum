# Funky_Pendulum

This is the continuation of the projects undertaken by [Jeppe](https://github.com/JeppeKlitgaard/CartER/) and [Will](https://github.com/will-hd/CartER).

Based on the project accomplished by Owen and Zhen during 2023 summer. Code developed by @Zzzzhen1. 
Any questions about the code please feel free to contact zy336@cam.ac.uk or leave in
dicussion session in this GitHub repository.

Download the zip file from the code icon. 
 
## General Description of the project. Should include the following features:

1. Schematics of the physical setup. (Possibly pictures and diagrams)
2. Complete safety measures.
3. What should students do to run the program. (Undecided yet: possibly use vscode or other editor space)
4. Possible changes that can be made in the program and how can students revert their changes. (In both python and Arduino)

## Getting Started

Download the zip file by clicking the **code** icon. Downloading the following Python modules before running the program.

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
4. math

For uploading the Arduino code to the Arduino Board, 

## Experiment Setup

- TODO: Explain how to use the 3D print file
- TODO: attach the pictures in the images folders here

## Proof of PID Inversion Control

### First Inversion Control Video

[<img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/CartER_1.jpg" width = "640" height = "360">](https://youtu.be/JiOYI30tvMM)

### Second Inversion Control Video

[<img src = "https://github.com/Zzzzhen1/Funky_Pendulum/blob/image_branch(protected)/image/CartER_2.jpg" width = "640" height = "360">](https://youtu.be/XKwGB6jRk7I)

## TODO List
General:
1. not fully understood, but to keep things in consistent with the 
phase feedback(with an extra pi), phase_active is also added pi

From arduino_manager.py:
1. this is supposedly referring to the reset pin, however not working properly

From data_process.py:
1. add a sampling rate selection in arduino (secondary)
2. add a different title for downward and upward control (secondary)
3. unify the datetime as a single variable in the data class

From Pendulum_Control_Console.py:
1. ask whether to enter data analysis mode (secondary)
2. check whether the platformio can do the arduino code upload because the Arduino IDE would be inefficient and faulty (secondary)
3. what to do if there are two peaks in the measure FFT? Worth mentioning in the handout(Non_linear behaviour)
4. all the parameters in the code should have a reasonable range
5. separate the different classes in different python files (secondary)
6. find delay time (a day of investigation) and make a plot (secondary)
7. the csv file saved after the scan experiment needs a date and time (secondary)
8. decide the amplitude of the NR scan drive stage (secondary)
9. pid data analysis --> stable time stop time (with threshold)... then plot stop_time vs. iteration graph (secondary)
10. jolt or hold up horizontal (secondary)
11. add a selection for the PID of normalised resonance (secondary)
12. decrease plotting fps (secondary)
