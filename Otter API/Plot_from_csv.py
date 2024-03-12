import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


def plot_ship_and_target_from_columns(file_path, ship_y_col, ship_x_col, target_y_col, target_x_col):
    # Load the CSV file, skip the first 3 rows, and use semicolons as the delimiter
    data = pd.read_csv(file_path, delimiter=';', skiprows=3)

    # Make sure the column indices are correct and within the bounds of the DataFrame's shape
    max_col_index = data.shape[1] - 1
    if ship_y_col > max_col_index or ship_x_col > max_col_index or target_y_col > max_col_index or target_x_col > max_col_index:
        raise ValueError("One or more specified column indices are out of bounds.")

    # Calculate the target's absolute coordinates based on the ship's position
    data['Target_Absolute_Y'] = data.iloc[:, ship_y_col] + data.iloc[:, target_y_col]
    data['Target_Absolute_X'] = data.iloc[:, ship_x_col] + data.iloc[:, target_x_col]

    # Plot the paths with the adjusted target coordinates
    plt.figure(figsize=(10, 6))
    plt.plot(data.iloc[:, ship_x_col], data.iloc[:, ship_y_col], label='Ship Path', marker='o', linestyle='-',
             color='blue')
    plt.plot(data['Target_Absolute_X'], data['Target_Absolute_Y'], label='Target Path (Adjusted)', marker='x',
             linestyle='--', color='red')
    plt.title('Otter target tracking')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.legend()
    plt.grid(True)
    plt.show()


# Adjust the column indices if necessary and ensure they are within the bounds of your DataFrame
file_path = 'logs/2024-03-07_15-17-09.csv'
ship_y_col = 20  # Adjust these indices as necessary, ensuring they're within the bounds of your data
ship_x_col = 21
target_y_col = 23
target_x_col = 24

# Call the function with updated arguments
plot_ship_and_target_from_columns(file_path, ship_y_col, ship_x_col, target_y_col, target_x_col)

