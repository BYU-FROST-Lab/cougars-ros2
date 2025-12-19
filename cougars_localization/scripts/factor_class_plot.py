#!/usr/bin/env python3

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class Series:
    def __init__(self, name, color='r', alpha=0.7, size=200):
        self.name = name
        self.color = color
        self.alpha = alpha
        self.size = size  # Size of the scatter points
        self.timestamps = []
        self.values = []
        self.pose_keys = []
        self.line = None
        self.scatter = None

    def add_delta_measurement(self, delta_value, timestamp, pose_key=None):
        """Add a new delta measurement with a specific timestamp and pose key."""

        self.timestamps.append(timestamp)
        if self.values:
            new_value = self.values[-1] + delta_value
        else:
            new_value = delta_value
        self.values.append(new_value)
        self.pose_keys.append(pose_key)

    def add_measurement(self, value, timestamp, pose_key=None):
        """Add a new measurement with a specific timestamp and optionally link to a pose key."""
        self.timestamps.append(timestamp)
        self.values.append(value)
        self.pose_keys.append(pose_key)


    def plot(self, ax):
        """Plot the series on the given axis."""
        if self.line is None:
            self.line, = ax.plot([], [], '-', color=self.color, alpha=self.alpha, label=self.name)
            self.scatter = ax.scatter([], [], color=self.color, alpha=0.8, s=self.size)

        # Plot the main data points
        self.line.set_data(self.timestamps, self.values)
        self.scatter.set_offsets(list(zip(self.timestamps, self.values)))

        ax.set_xlim(1724953702000000000, 1724953854000000000)
        ax.set_ylim(-2395, -2367)


        # if len(self.timestamps) > 1:
        #     ax.set_xlim(min(self.timestamps), max(self.timestamps))
        #     ax.set_ylim(min(self.values) - 1, max(self.values) + 1)
        # else:
        #     ax.set_xlim(self.timestamps[0] - 1, self.timestamps[0] + 1)
        #     ax.set_ylim(self.values[0] - 1, self.values[0] + 1)

        # Draw pose keys as text on the plot
        for i, (x, y, key) in enumerate(zip(self.timestamps, self.values, self.pose_keys)):
            if key is not None:
                ax.text(x, y, str(key), color='black', ha='center', va='center', fontsize=10, weight='bold')

class Plotter:
    def __init__(self, series_list):
        # Store the series objects
        self.series_list = series_list

        # Create figure and axis
        self.figure, self.ax = plt.subplots()

        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        self.ax.grid(True)

        # Display the initial plot
        plt.ion()  # Interactive mode on
        plt.show()

    def update_plot(self):
        """Update the plot with new measurements."""
        for series in self.series_list:
            print('here')
            series.plot(self.ax)

        self.plot_keyed_connections()

        plt.draw()
        plt.pause(0.1)

    def plot_keyed_connections(self):
        """Plot lines connecting points in different series that share the same pose key."""
        for i, series1 in enumerate(self.series_list):
            for j, series2 in enumerate(self.series_list):
                if i < j:  # Avoid repeating checks
                    for k, key1 in enumerate(series1.pose_keys):
                        if key1 is not None and key1 in series2.pose_keys:
                            idx2 = series2.pose_keys.index(key1)
                            self.ax.plot([series1.timestamps[k], series2.timestamps[idx2]],
                                         [series1.values[k], series2.values[idx2]], 'g--')

    def show(self):
        """Display the plot."""
        plt.show(block=True)

# # Create instances of the Series class for each plot
# delta_series_1 = Series(name='Delta 1', color='red', size=600, alpha=0.5)
# gps_series_1 = Series(name='GPS 1', color='blue')

# delta_series_2 = Series(name='Delta 2', color='green', size=600, alpha=0.5)
# imu_series_2 = Series(name='IMU 2', color='purple')

# # Create two Plotter objects with different series
# plotter1 = Plotter([delta_series_1, gps_series_1])
# plotter2 = Plotter([delta_series_2, imu_series_2])

# for i in range(20):
#     timestamp = i
#     pose_key = i + 100

#     # Add measurements to the first plot
#     delta_series_1.add_delta_measurement(i, timestamp, pose_key)
#     if (pose_key % 2 == 0):
#         gps_series_1.add_measurement(i + 1, timestamp, pose_key)
#     else:
#         gps_series_1.add_measurement(i+1, timestamp)

#     # Add measurements to the second plot
#     delta_series_2.add_delta_measurement(i, timestamp, pose_key)
#     imu_series_2.add_measurement(i + 3, timestamp, pose_key)

#     # Update both plots
#     plotter1.update_plot()
#     plotter2.update_plot()

#     time.sleep(1)

# # Display both plots
# plotter1.show()
# plotter2.show()
