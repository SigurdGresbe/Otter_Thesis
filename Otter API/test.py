import pandas as pd
import numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import griddata


class RPMToForceInterpolator:
    def __init__(self, csv_file_path):
        self.rpm_left, self.rpm_right, self.force_x, self.force_z = self.load_and_prepare_data(csv_file_path)
        self.tree = cKDTree(np.vstack((self.rpm_left, self.rpm_right)).T)

    def load_and_prepare_data(self, csv_file_path):
        df = pd.read_csv(csv_file_path, delimiter=';', quotechar='"', index_col=0)
        force_x, force_z = np.meshgrid(df.index.astype(float), df.columns.astype(float), indexing='ij')

        rpm_left = []
        rpm_right = []
        for i, row in enumerate(df.itertuples(index=False)):
            for j, cell in enumerate(row):
                if pd.notna(cell):
                    l_rpm, r_rpm = map(float, cell.split(';'))
                    rpm_left.append(l_rpm)
                    rpm_right.append(r_rpm)

        return np.array(rpm_left), np.array(rpm_right), force_x.ravel(), force_z.ravel()

    def interpolate_force_values(self, rpm_left, rpm_right, k=3):
        distances, indices = self.tree.query([(rpm_left, rpm_right)], k=k)
        weights = 1 / (distances[0] + 1e-10)  # Prevent division by zero
        normalized_weights = weights / np.sum(weights)

        force_x_interp = np.sum(normalized_weights * self.force_x[indices[0]])
        force_z_interp = np.sum(normalized_weights * self.force_z[indices[0]])

        return force_x_interp, force_z_interp


# Usage example
csv_file_path = "lib/throttle_map_v2_noneg.csv"  # Adjust this path to where your CSV file is located
interpolator = RPMToForceInterpolator(csv_file_path)

# Input RPM values for an example
example_rpm = (250, 100)  # Example RPM pair for interpolation
force_z_interp, force_x_interp = interpolator.interpolate_force_values(*example_rpm)

print(f"Interpolated force values for RPM {example_rpm}: Force_x = {force_x_interp}, Force_z = {force_z_interp}")
