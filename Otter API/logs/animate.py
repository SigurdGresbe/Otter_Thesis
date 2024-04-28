import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

file_path = r'C:\Users\47900\Python prog 1\Otter_Thesis-main\Otter API\logs\2024-04-23_12-27-203.csv'
data = pd.read_csv(file_path, delimiter=';')

# Bad data to NaN
data['north_from_observer'] = pd.to_numeric(data['north_from_observer'], errors='coerce')
data['east_from_observer'] = pd.to_numeric(data['east_from_observer'], errors='coerce')

# Replace NaN values with 0 (or another appropriate value) in the coordinate columns
data['target_north_from_observer'] = data['target_north_from_observer'].fillna(0)
data['target_east_from_observer'] = data['target_east_from_observer'].fillna(0)
data['north_from_observer'] = data['north_from_observer'].fillna(0)
data['east_from_observer'] = data['east_from_observer'].fillna(0)

# coordinates from csv
x_target = data['target_east_from_observer']
y_target = data['target_north_from_observer']
x_observer = data['east_from_observer']
y_observer = data['north_from_observer']

# plot and axis parameters
fig, ax = plt.subplots()
ax.set_xlim(min(x_observer.min(), x_target.min()), max(x_observer.max(), x_target.max()))
ax.set_ylim(min(y_observer.min(), y_target.min()), max(y_observer.max(), y_target.max()))
ax.set_xlabel('East from Observer')
ax.set_ylabel('North from Observer')

# plot lines
line_target, = ax.plot([], [], linestyle='--', linewidth=1, label='Target Path')
line_observer, = ax.plot([], [], linestyle='--', linewidth=1, label='Target Path')
ax.legend()


def init():
    line_target.set_data([], [])
    line_observer.set_data([], [])
    return line_target, line_observer


def animate(i):
    line_target.set_data(x_target[:i], y_target[:i])
    line_observer.set_data(x_observer[:i], y_observer[:i])
    return line_target, line_observer


ani = FuncAnimation(fig, animate, frames=min(len(x_observer), len(x_target)), init_func=init, blit=True, interval=100)

#save file
ani.save('203.gif', writer='pillow', fps=30)