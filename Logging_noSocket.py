import csv
import argparse
import json
import requests
import sys
import time
import datetime 
import pandas as pd

stop_loop = 0
"""
def append_xyz_to_csv(x, y, z, filename):
    # Open the file in append mode with newline='' to avoid extra blank lines
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        # Append the new entry
        writer.writerow([x, y, z])
"""
def append_data_and_save(df, time, x, y, z, filepath):
    """
    Appends new data to the provided DataFrame and saves it to a CSV file.

    Parameters:
    - df (pd.DataFrame): The DataFrame to append to.
    - time (any): The time value to append.
    - x (float): The x-coordinate to append.
    - y (float): The y-coordinate to append.
    - z (float): The z-coordinate to append.
    - filepath (str): The file path where the DataFrame will be saved as a CSV.

    Returns:
    - pd.DataFrame: The updated DataFrame with the new data appended.
    """
    # Create a dictionary from the provided data
    data = {'Time': time, 'X': x, 'Y': y, 'Z': z}

    # Convert the dictionary to a DataFrame
    new_row = pd.DataFrame([data])

    # Append the new row to the DataFrame
    updated_df = df.concat(new_row, ignore_index=True)

    # Save the updated DataFrame to a CSV file
    updated_df.to_csv(filepath, index=False)
    
    return updated_df


def get_data(url):
    """
    if len(sys.argv) < 2:
        exit("usage: name ip")

    sys.argv[1]
    """
    try:
        r = requests.get(url)
    except requests.exceptions.RequestException as exc:
        print("Exception occurred: {}".format(exc))
        return None

    if r.status_code != requests.codes.ok:
        print("Got error {}: {}".format(r.status_code, r.text))
        return None

    return r.json()


def get_acoustic_position(base_url):

    return get_data("{}/api/v1/position/acoustic/filtered".format(base_url))


def main():
        # Send data to the server
        parser = argparse.ArgumentParser(description="Get acoustic position from Water Linked Underwater GPS")
        parser.add_argument(
            "-u",
            "--url",
            help="Base URL to use",
            type=str,
            default="http://192.168.7.1/")
        
        args = parser.parse_args()
        
        # Initialize an empty DataFrame with columns
        df = pd.DataFrame(columns=['Time', 'X', 'Y', 'Z'])

        # Specify the file path for the CSV
        csv_file_path = 'C:\\Users\\sgres\\Documents\\Otter_Thesis-main\\Otter API\\xyz_data.csv'


        base_url = args.url
        print("Using base_url: %s" % base_url)
        while not stop_loop:
            acoustic_position = get_acoustic_position(base_url)
            #append_xyz_to_csv(acoustic_position['x'], acoustic_position['y'], acoustic_position['z'], 'xyz_data.csv')
            df = append_data_and_save(df, datetime.datetime.now().strftime('%m - %d %H:%M:%S'), acoustic_position['x'], acoustic_position['y'], acoustic_position['z'], csv_file_path)
            time.sleep(0.2)  # Sleep for 0.2 seconds to make the loop run 5 times per second
            print(df)
        """
        X, Y, Z = "x", "y", "z"
        socket_com(X,Y,Z)
        """

if __name__ == "__main__":
    main()



