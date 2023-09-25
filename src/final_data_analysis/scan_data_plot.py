import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.optimize import curve_fit

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
    # Load data from CSV
    while True:
        try:
            file_path = input('Enter file path: ')
            # file_path = r'C:\Programming\Python\CartER_III\scan_data-19-09.csv'
            data = pd.read_csv(file_path)
            ref_file_path = input('Enter reference file path: ')
            # ref_file_path = r'C:\Programming\Python\CartER_III\reference_parameters-init-19-09.csv'
            ref_data = pd.read_csv(ref_file_path)
            break
        except FileNotFoundError:
            print('File not found. Please try again.')

    data_array = data.to_numpy()
    ref_data_array = ref_data.to_numpy()
    _, _, _, _, _, data_amp_array, _, _, _, = zip(*data_array)
    driving_amps = ref_data['amp_0'].unique()
    driving_freqs = ref_data['freq'].unique()
    colors = cm.rainbow(np.linspace(0, 1, len(driving_amps)))
    # Calculate response amplitude to driving amplitude ratio
    data['response_amp'] = abs(data['response_amp'])
    data['amp_ratio'] = data['response_amp'] / data['driving_amp']
    data['rectified_driving_amps'] = fit_driving_amp(data_amp_array, driving_amps)

    data.sort_values(by='driving_freq', inplace=True)

    # Plot the amplitude ratio against driving frequency with different colors
    plt.figure(figsize = (10, 5))
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
        top_indices = np.array(np.argmax(abs(subset['amp_ratio'])))
        min_index = np.min(top_indices) - 1
        max_index = np.max(top_indices) + 1
        top_indices = np.append(top_indices, min_index)
        top_indices = np.append(top_indices, max_index)
        x_data = subset['driving_freq'].iloc[top_indices]
        y_data = abs(subset['amp_ratio']).iloc[top_indices]
        popt,pcov = curve_fit(parabolic_func, x_data, y_data, maxfev = 2000000)
        x_fit = np.linspace(min(x_data), max(x_data), 1000)
        y_fit = parabolic_func(x_fit, *popt)
        plt.plot(x_fit, y_fit, color = colors[index])
        print('driving_amp: %d, peak freq: '%(int(driving_amp)) + str(x_fit[np.argmax(y_fit)])[:6] + ' Hz')
    plt.xlabel('Driving Frequency / Hz')
    plt.ylabel('Response Amp / Driving Amp')
    plt.title('Response Amplitude to Driving Amplitude Ratio vs Driving Frequency')
    plt.legend(loc = 'upper left', bbox_to_anchor=(1.005, 1.05))
    plt.grid(True)

    # Plot phase against driving frequency with different colors
    plt.figure(figsize = (10, 5))
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
    
    data.sort_values(by='rectified_driving_amps', inplace=True)
    plt.figure(figsize = (10, 5))
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
    
    data.sort_values(by='response_amp', inplace=True)
    plt.figure(figsize = (10, 5))
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
    
    data.sort_values(by='rectified_driving_amps', inplace=True)
    plt.figure(figsize = (10, 5))
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
    
    data.sort_values(by='response_amp', inplace=True)
    plt.figure(figsize = (10, 5))
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
    
    # Plot the 3D plot
    plt.figure(figsize = (10, 5))
    ax = plt.axes(projection='3d')
    for index, driving_freq in enumerate(driving_freqs):
        subset = data[data['driving_freq'] == driving_freq]
        ax.plot3D(subset['rectified_driving_amps'], 
                  subset['response_amp'], 
                  subset['phase'], 
                  '--', 
                  color = colors[index])
        ax.scatter3D(subset['rectified_driving_amps'], 
                     subset['response_amp'], 
                     subset['phase'], 
                     color = colors[index],
                     label = str(driving_freq)[:4] + ' Hz')
    
    plt.show()
