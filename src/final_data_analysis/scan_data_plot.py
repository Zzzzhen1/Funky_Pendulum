import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.optimize import curve_fit

FIG_WIDTH = 12
FIG_HEIGHT = 8
TRANSPARENCY = 0.4

def fit_driving_amp(data_amp_array, ref_amp_list):
    # Fit the driving amplitude
    temp_array = []
    for driving_amp in data_amp_array:
        temp_amp = ref_amp_list[0]
        for ref_amp in ref_amp_list:
            if abs(driving_amp - ref_amp) < abs(driving_amp - temp_amp):
                temp_amp = ref_amp
        temp_array.append(temp_amp)
    return pd.Series(temp_array)

def parabolic_func(x, a, b, c):
    return a * x**2 + b * x + c

if (__name__ == '__main__'):
    # Turn on the saving mode
    flag_save = False
    
    # Load data from CSV
    while True:
        try:
            file_path = input('Enter file path: ')
            parent_dir = os.path.dirname(file_path)
            data = pd.read_csv(file_path)
            # This is for automated scan data analysis:
            # ref_file_path = input('Enter reference file path: ')
            # ref_data = pd.read_csv(ref_file_path)
            break
        except OSError:
            print('File not found. Please try again.')

    try:
        os.mkdir(parent_dir + '/plots')
    except FileExistsError:
        pass
    
    data_array = data.to_numpy()
    # This is for automated scan data analysis:
    # ref_data_array = ref_data.to_numpy()
    _, _, _, _, _, data_amp_array, _, _, _, = zip(*data_array)
    # This is for singly scan data:
    driving_amps = data['amp_0'].unique()
    
    # This is for automated scan data analysis:
    # driving_amps = ref_data['amp_0'].unique()
    
    # This is for singly scan data:
    driving_freqs = data['driving_freq'].unique()
    # This is for automated scan data analysis:
    # driving_freqs = ref_data['freq'].unique()
    
    colors = cm.rainbow(np.linspace(0, 1, len(driving_amps)))
    
    # Calculate response amplitude to driving amplitude ratio
    data['response_amp'] = abs(data['response_amp'])
    data['amp_ratio'] = data['response_amp'] / data['driving_amp']
    data['rectified_driving_amps'] = fit_driving_amp(data_amp_array, driving_amps)

    data.sort_values(by='driving_freq', inplace=True)

    # Plot the amplitude ratio against driving frequency with different colors
    plt.figure('Response Amplitude to Driving Amplitude Ratio vs Driving Frequency', figsize = (FIG_WIDTH, FIG_HEIGHT))
    peak_freqs = [] # will be appended with (driving_amp, response_amp, peak_freq) tuples
    for index, driving_amp in enumerate(driving_amps):
        subset = data[data['rectified_driving_amps'] == driving_amp]
        plt.plot(subset['driving_freq'], 
                 abs(subset['amp_ratio']), 
                 '--', 
                 color = colors[index])
        plt.errorbar(subset['driving_freq'], 
                     abs(subset['amp_ratio']), 
                     yerr = abs(subset['response_amp_err'] / subset['driving_amp']),
                     fmt = 'x', 
                     label = str(driving_amp) + ' step', 
                     color = colors[index],
                     capsize = 1)
        try:
            top_indices = np.array(np.argmax(abs(subset['amp_ratio'])))
            if top_indices == 0 or top_indices == len(subset['amp_ratio']) - 1:
                # If the peak is at the edge of the data, then quit the curve fitting
                raise ValueError
            min_index = np.min(top_indices) - 1
            max_index = np.max(top_indices) + 1
            top_indices = np.append(top_indices, min_index)
            top_indices = np.append(top_indices, max_index)
            x_data = subset['driving_freq'].iloc[top_indices]
            y_data = abs(subset['amp_ratio']).iloc[top_indices]
            popt,pcov = curve_fit(parabolic_func, x_data, y_data, maxfev = 2000000)
            x_fit = np.linspace(min(x_data), max(x_data), 10000)
            y_fit = parabolic_func(x_fit, *popt)
            plt.plot(x_fit, y_fit, color = colors[index])
            peak_freqs.append((driving_amp, driving_amp * np.max(y_fit), x_fit[np.argmax(y_fit)]))
            print('driving_amp: %d, peak freq: '%(int(driving_amp)) + str(x_fit[np.argmax(y_fit)])[:6] + ' Hz')
        except (ValueError, IndexError):
            pass
    plt.xlabel('Driving Frequency / Hz')
    plt.ylabel('Response Amp / Driving Amp')
    plt.title('Response Amplitude to Driving Amplitude Ratio vs Driving Frequency')
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    plt.grid(True)
    if flag_save:
        plt.savefig(parent_dir + '/plots/amp_ratio_vs_driving_freq.png', dpi = 600)

    # Plot the peak frequency against response amplitude
    plt.figure('Peak Frequency vs Response Amplitude', figsize = (FIG_WIDTH, FIG_HEIGHT))
    peak_freqs = np.array(peak_freqs)
    plt.plot(peak_freqs[:, 1], peak_freqs[:, 2], 'bx', markersize = 5)
    plt.plot(peak_freqs[:, 1], peak_freqs[:, 2], 'b-')
    plt.xlabel('Response Amplitude / rad')
    plt.ylabel('Peak Frequency / Hz')
    if len(peak_freqs[:, 1])>2:
        popt, pcov = curve_fit(parabolic_func, peak_freqs[:, 1], peak_freqs[:, 2], maxfev = 2000000)
    else:
        popt = np.array([1,0,0])
        pcov = np.array([0,0,0])
    x_fit = np.linspace(min(peak_freqs[:, 1]), max(peak_freqs[:, 1]), 10000)
    y_fit = parabolic_func(x_fit, *popt)
    plt.plot(x_fit, y_fit, 'r--')
    plt.text(0.5, 
             0.1, 
             'y = %.2f * (x^2 + %.2f * x + %.2f)'%(popt[0], popt[1] / popt[0], popt[2] / popt[0]), 
             transform = plt.gca().transAxes)
    if flag_save:
        plt.savefig(parent_dir + '/plots/peak_freq_vs_response_amp.png', dpi = 600)
    
    # Plot phase against driving frequency with different colors
    plt.figure('Phase vs Driving Frequency', figsize = (FIG_WIDTH, FIG_HEIGHT))
    for index, driving_amp in enumerate(driving_amps):
        subset = data[data['rectified_driving_amps'] == driving_amp]
        plt.plot(subset['driving_freq'], 
                 subset['phase'], 
                 '--', 
                 color = colors[index])
        plt.errorbar(subset['driving_freq'], 
                    subset['phase'], 
                    yerr = abs(subset['phase_err']), 
                    fmt = 'x',
                    label = str(driving_amp) + ' step', 
                    color = colors[index],
                    capsize = 1)
    plt.xlabel('Driving Frequency / Hz')
    plt.ylabel('Phase / pi')
    plt.title('Phase vs Driving Frequency')
    plt.grid(True)
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    if flag_save:
        plt.savefig(parent_dir + '/plots/phase_vs_driving_freq.png', dpi = 600)
    
    data.sort_values(by='rectified_driving_amps', inplace=True)
    plt.figure('Response Amplitude to Driving Amplitude Ratio vs Driving Amplitude', figsize = (FIG_WIDTH, FIG_HEIGHT))
    colors = cm.rainbow(np.linspace(0, 1, len(driving_freqs)))
    for index, driving_freq in enumerate(driving_freqs):
        subset = data[data['driving_freq'] == driving_freq]
        plt.plot(subset['rectified_driving_amps'], 
                 abs(subset['amp_ratio']), 
                 '--', 
                 color = colors[index])
        plt.errorbar(subset['rectified_driving_amps'], 
                     abs(subset['amp_ratio']), 
                     yerr = abs(subset['response_amp_err'] / subset['driving_amp']), 
                     fmt = 'x', 
                     label = str(driving_freq)[:4] + ' Hz', 
                     color = colors[index],
                     capsize = 1)
    plt.xlabel('Driving Amplitude / step')
    plt.ylabel('Response Amp / Driving Amp')
    plt.title('Response Amplitude to Driving Amplitude Ratio vs Driving Amplitude')
    plt.grid(True)
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    if flag_save:
        plt.savefig(parent_dir + '/plots/amp_ratio_vs_driving_amp.png', dpi = 600)
    
    data.sort_values(by='response_amp', inplace=True)
    plt.figure('Response Amplitude to Driving Amplitude Ratio vs Response Amplitude', figsize = (FIG_WIDTH, FIG_HEIGHT))
    for index, driving_freq in enumerate(driving_freqs):
        subset = data[data['driving_freq'] == driving_freq]
        plt.plot(abs(subset['response_amp']), 
                 abs(subset['amp_ratio']), 
                 '--', 
                 color = colors[index])
        plt.errorbar(abs(subset['response_amp']), 
                     abs(subset['amp_ratio']), 
                     yerr = abs(subset['response_amp_err'] / subset['driving_amp']), 
                     fmt = 'x', 
                     label = str(driving_freq)[:4] + ' Hz', 
                     color = colors[index],
                     capsize = 1)
    plt.xlabel('Response Amplitude / rad')
    plt.ylabel('Response Amp / Driving Amp')
    plt.title('Response Amplitude to Driving Amplitude Ratio vs Response Amplitude')
    plt.grid(True)
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    if flag_save:
        plt.savefig(parent_dir + '/plots/amp_ratio_vs_response_amp.png', dpi = 600)
    
    data.sort_values(by='rectified_driving_amps', inplace=True)
    plt.figure('Phase vs Driving Amplitude', figsize = (FIG_WIDTH, FIG_HEIGHT))
    for index, driving_freq in enumerate(driving_freqs):
        subset = data[data['driving_freq'] == driving_freq]
        plt.plot(subset['rectified_driving_amps'], 
                 subset['phase'], 
                 '--', 
                 color = colors[index])
        plt.errorbar(subset['rectified_driving_amps'], 
                     subset['phase'], 
                     yerr = abs(subset['phase_err']), 
                     fmt = 'x', 
                     label = str(driving_freq)[:4] + ' Hz', 
                     color = colors[index],
                     capsize = 1)
    plt.xlabel('Driving Amplitude / step')
    plt.ylabel('Phase / pi')
    plt.title('Phase vs Driving Amplitude')
    plt.grid(True)
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    if flag_save:
        plt.savefig(parent_dir + '/plots/phase_vs_driving_amp.png', dpi = 600)
    
    data.sort_values(by='response_amp', inplace=True)
    plt.figure('Phase vs Response Amplitude', figsize = (FIG_WIDTH, FIG_HEIGHT))
    for index, driving_freq in enumerate(driving_freqs):
        subset = data[data['driving_freq'] == driving_freq]
        plt.plot(abs(subset['response_amp']), 
                 subset['phase'], 
                 '--', 
                 color = colors[index])
        plt.errorbar(abs(subset['response_amp']), 
                     subset['phase'], 
                     yerr = abs(subset['phase_err']), 
                     fmt = 'x', 
                     label = str(driving_freq)[:4] + ' Hz', 
                     color = colors[index],
                     capsize = 1)
    plt.xlabel('Response Amplitude / rad')
    plt.ylabel('Phase / pi')
    plt.title('Phase vs Response Amplitude')
    plt.grid(True)
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    plt.show()
    if flag_save:
        plt.savefig(parent_dir + '/plots/phase_vs_response_amp.png', dpi = 600)
        plt.close('all')
    
    # Plot the 3D plot
    plt.figure('Response Ratio vs. Driving Amplitude vs. Driving Frequency', figsize = (FIG_WIDTH, FIG_HEIGHT))
    ax = plt.axes(projection='3d')
    for index, driving_freq in enumerate(driving_freqs):
        subset = data[data['driving_freq'] == driving_freq]
        ax.plot(subset['rectified_driving_amps'], 
                  subset['response_amp'], 
                  subset['phase'], 
                  '--', 
                  color = colors[index])
        ax.scatter(subset['rectified_driving_amps'], 
                     subset['response_amp'], 
                     subset['phase'], 
                     color = colors[index],
                     label = str(driving_freq)[:4] + ' Hz')
        ax.set_xlabel('Driving Amplitude / step')
        ax.set_ylabel('Response Amplitude / rad')
        ax.set_zlabel('Phase / pi')
    plt.show()
