import pandas as pd
import numpy as np


def find_closest(df, input_value):
    # Split the input value into two separate numbers
    target_x, target_y = map(int, input_value.strip("()").split(';'))

    # Initialize variables to keep track of the closest distance and corresponding index
    closest_distance = float('inf')
    closest_indices = None

    # Iterate over the DataFrame
    for column in df.columns:
        for row_index, value in df[column].iteritems():
            if pd.notna(value):
                # Split the value in the cell into two numbers
                cell_x, cell_y = map(int, value.split(';'))
                # Calculate Euclidean distance
                distance = np.sqrt((cell_x - target_x) ** 2 + (cell_y - target_y) ** 2)

                if distance < closest_distance:
                    closest_distance = distance
                    closest_indices = (row_index, column)

    return closest_indices


df = pd.read_csv('lib/throttle_map_v2_noneg.csv', index_col=0, sep=";")
df = df.dropna(axis=1, how='all')

# Drop rows where all values are NaN
df = df.dropna(axis=0, how='all')

print(df)

# Input value for which you want to find the closest
input_value = "(700;350)"

# Find the closest value's indices
closest_indices = find_closest(df, input_value)

if closest_indices:
    print(f"The closest value is at row {closest_indices[0]} and column {closest_indices[1]}")
else:
    print("No closest value found.")

row_index = "0.6"
print(df[f"{float(row_index):.2f}"][0.05])