import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV
data = pd.read_csv('C:\Programming\Python\Funky Pendulum\scan_for_response_test_data\scan_data.csv')

# Calculate response amplitude to driving amplitude ratio
data['amp_ratio'] = data['response_amp'] / data['driving_amp']

# Create a list of unique parent_dir values for coloring
parent_dirs = data['parent_dir'].unique()

data.sort_values(by='driving_freq', inplace=True)

# Plot the ratio against driving frequency with different colors
plt.figure(figsize = (10, 5))
for parent_dir in parent_dirs:
    if(parent_dir == '11-08-csv' or parent_dir == '10-08-csv'):
        continue
    subset = data[data['parent_dir'] == parent_dir]
    # plt.plot(subset['driving_freq'], abs(subset['amp_ratio']), '--', label = parent_dir)
    plt.errorbar(subset['driving_freq'], abs(subset['amp_ratio']), 
                 yerr = abs(subset['response_amp_err'] / subset['driving_amp']), 
                 fmt = 'o', label = parent_dir)
plt.xlabel('Driving Frequency / Hz')
plt.ylabel('Response Amp / Driving Amp')
plt.title('Response Amplitude to Driving Amplitude Ratio vs Driving Frequency')
plt.legend()
plt.grid(True)

# Plot phase against driving frequency with different colors
plt.figure(figsize = (10, 5))
for parent_dir in parent_dirs:
    if(parent_dir == '11-08-csv' or parent_dir == '10-08-csv'):
        continue
    subset = data[data['parent_dir'] == parent_dir]
    # plt.plot(subset['driving_freq'], subset['phase'], '--', label = parent_dir)
    plt.errorbar(subset['driving_freq'], subset['phase'], yerr = abs(subset['phase_err']), fmt = 'o', label = parent_dir)
plt.xlabel('Driving Frequency / Hz')
plt.ylabel('Phase / pi')
plt.title('Phase vs Driving Frequency')
plt.legend()
plt.grid(True)
plt.show()
