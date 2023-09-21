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
# plt.rcParams.update({'text.usetex': True,
#                      'font.family': 'serif',})

while True:
    filename = input("Enter the file name: ")
    if os.path.isfile(filename):
        break
    else:
        print("File does not exist. Please try again.")

gamma = []
gamma_err = []
omega = []
omega_err = []
freq = []
freq_err = []

avg_gamma = 0
avg_gamma_err = 0
avg_omega = 0
avg_omega_err = 0
avg_freq = 0
avg_freq_err = 0

with open(filename, "r", newline = '') as file:
    reader = csv.reader(file)
    for row in reader:
        if row[0] == "file_name":
            continue
        else:
            gamma.append(float(row[4]))
            gamma_err.append(float(row[5]))
            omega.append(float(row[2]))
            omega_err.append(float(row[3]))
            freq.append(float(row[0]))
            freq_err.append(float(row[1].rstrip()))
            
figure, axes = plt.subplots(3, 1, figsize = (8, 6), sharex = True)
for i in range(len(gamma)):
    avg_gamma += gamma[i] / gamma_err[i] ** 2
    avg_gamma_err += 1 / gamma_err[i] ** 2
    avg_omega += omega[i] / omega_err[i] ** 2
    avg_omega_err += 1 / omega_err[i] ** 2
    avg_freq += freq[i] / freq_err[i] ** 2
    avg_freq_err += 1 / freq_err[i] ** 2

x = np.arange(0, len(gamma), 1)

axes[0].text(0.1, 1.04, r'$\gamma$ = ' + str(round(avg_gamma / avg_gamma_err, 3)) + r'$\pm$' + str(round(1 / np.sqrt(avg_gamma_err), 3)) + r' rads$^{-1}$', transform = axes[0].transAxes)
axes[1].text(0.1, 1.04, r'$\omega$ = ' + str(round(avg_omega / avg_omega_err, 4)) + r'$\pm$' + str(round(1 / np.sqrt(avg_omega_err), 4)) + r' Hz', transform = axes[1].transAxes)
axes[2].text(0.1, 1.04, r'$f$ = ' + str(round(avg_freq / avg_freq_err, 3)) + r'$\pm$' + str(round(1 / np.sqrt(avg_freq_err), 2)) + r' Hz', transform = axes[2].transAxes)

axes[0].plot(x, gamma, 'r-')
axes[1].plot(x, omega, 'r-')
axes[2].plot(x, freq, 'r-')
axes[0].errorbar(x, gamma, yerr = gamma_err, fmt = 'bx', capsize = 3, label = r'$\gamma$ by best fit line')
axes[0].set_ylabel(r'$\gamma$/rads$^{-1}$')
axes[1].errorbar(x, omega, yerr = omega_err, fmt = 'bx', capsize = 3, label = r'$\omega$ by best fit line')
axes[1].set_ylabel(r'$\omega$/Hz')
axes[2].errorbar(x, freq, yerr = freq_err, fmt = 'bx', capsize = 3, label = r'$f$')
axes[2].set_ylabel(r'$f$/Hz')
axes[0].legend()
axes[1].legend()
axes[2].legend()
figure.suptitle("Natural Frequency and Damping Coefficient of the Cart", fontsize = 16)
plt.show()