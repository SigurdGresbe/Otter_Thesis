import pandas as pd
import matplotlib.pyplot as plt

date = "2024-04-23_12-27-203"
data = pd.read_csv(f'{date}.csv', sep=';')


data_corrected = data.iloc[3:]


data_corrected['north_from_observer'] = pd.to_numeric(data_corrected['north_from_observer'], errors='coerce')
data_corrected['east_from_observer'] = pd.to_numeric(data_corrected['east_from_observer'], errors='coerce')
data_corrected['target_north_from_observer'] = pd.to_numeric(data_corrected['target_north_from_observer'], errors='coerce')
data_corrected['target_east_from_observer'] = pd.to_numeric(data_corrected['target_east_from_observer'], errors='coerce')


plt.figure(figsize=(10, 6))


plt.plot(data_corrected['east_from_observer'], data_corrected['north_from_observer'], label='Ship Path', marker='o', linestyle='-', markersize=1)

plt.plot(data_corrected['target_east_from_observer'], data_corrected['target_north_from_observer'], label='Target Path', marker='x', linestyle='--', markersize=1)

plt.title('Ship and Target Paths')
plt.xlabel('East from Observer')
plt.ylabel('North from Observer')
plt.legend()
plt.grid(True)
#plt.savefig(f"plots/{date}.png")
plt.show()
