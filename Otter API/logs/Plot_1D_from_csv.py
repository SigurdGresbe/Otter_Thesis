import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

# Load the data with correct headers
data = pd.read_csv('2024-03-22_12-45-31.csv', sep=';')

# Assuming the first row is the header, manually skip the first three data rows
data_corrected = data.iloc[3:]

# Ensure the Timestamp column is parsed correctly as datetime
data_corrected['Timestamp'] = pd.to_datetime(data_corrected.iloc[:, 0], format='%Y-%m-%d_%H:%M:%S:%f', errors='coerce')

# Filter out rows where the timestamp could not be parsed
data_corrected = data_corrected.dropna(subset=['Timestamp'])

# Convert "distance_to_target" to numeric values in case they are read as strings
data_corrected['distance_to_target'] = pd.to_numeric(data_corrected['distance_to_target'], errors='coerce')

# Set a variable for plotting every nth data point
n = 10  # You can change this value as needed
data_thinned = data_corrected.iloc[::n, :]  # Select every nth row

# Plotting
plt.figure(figsize=(12, 6))

# Plotting the distance to target over time, using only time (no date) on the X-axis
plt.plot(data_thinned['Timestamp'], data_thinned['distance_to_target'], label='Distance to Target', marker='o', linestyle='-', markersize=4)

plt.title('Distance to Target Over Time')
plt.xlabel('Time')
plt.ylabel('Distance to Target')
plt.legend()

# Formatting the X-axis to display only the time (HH:MM:SS)
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
plt.gca().xaxis.set_major_locator(mdates.AutoDateLocator())
plt.gcf().autofmt_xdate() # Auto-rotate date labels to fit them better

# Check the range of 'distance_to_target' and adjust y-axis if necessary
plt.ylim(data_corrected['distance_to_target'].min(), data_corrected['distance_to_target'].max())

plt.grid(True)
plt.tight_layout()  # Adjust the layout to fit everything properly
plt.show()