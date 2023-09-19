import pandas as pd
import matplotlib.pyplot as plt
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']

# Load data from CSV

while True:
    try:
        file_path = input('Enter file path: ')
        data = pd.read_csv(file_path)
        break
    except FileNotFoundError:
        print('File not found. Please try again.')

# Calculate response amplitude to driving amplitude ratio
data['amp_ratio'] = data['response_amp'] / data['driving_amp']

# Create a list of unique parent_dir values for coloring
parent_dirs = data['parent_dir'].unique()
# TODO: sorted by the driving amplitude
data.sort_values(by='driving_freq', inplace=True)

# Plot the ratio against driving frequency with different colors
plt.figure(figsize = (10, 5))
for index, parent_dir in enumerate(parent_dirs):
    subset = data[data['parent_dir'] == parent_dir]
    plt.plot(subset['driving_freq'], abs(subset['amp_ratio']), '--', label = parent_dir, color = colors[index])
    plt.errorbar(subset['driving_freq'], abs(subset['amp_ratio']), 
                 yerr = abs(subset['response_amp_err'] / subset['driving_amp']), 
                 fmt = 'o', label = parent_dir, color = colors[index])
plt.xlabel('Driving Frequency / Hz')
plt.ylabel('Response Amp / Driving Amp')
plt.title('Response Amplitude to Driving Amplitude Ratio vs Driving Frequency')
plt.legend()
plt.grid(True)

# Plot phase against driving frequency with different colors
plt.figure(figsize = (10, 5))
for index, parent_dir in enumerate(parent_dirs):
    subset = data[data['parent_dir'] == parent_dir]
    plt.plot(subset['driving_freq'], subset['phase'], '--', label = parent_dir, color = colors[index])
    plt.errorbar(subset['driving_freq'], 
                 subset['phase'], 
                 yerr = abs(subset['phase_err']), 
                 fmt = 'o', 
                 label = parent_dir, 
                 color = colors[index])
plt.xlabel('Driving Frequency / Hz')
plt.ylabel('Phase / pi')
plt.title('Phase vs Driving Frequency')
plt.legend()
plt.grid(True)
plt.show()
