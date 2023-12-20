import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')  # Change the backend to Qt5Agg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd


# Function to read CSV data
def csv_to_datadict(file_path: str, fps, time_key='time [s]'):
    df = pd.read_csv(file_path)
    datadict = {}
    for key in df:
        datadict[key] = df[key].to_numpy()

    # Subsample data
    dt = datadict[time_key][1] - datadict[time_key][0]
    samples = int((1 / fps) / dt)
    for key in datadict:
        datadict[key] = datadict[key][::samples]
    return datadict


def get_frames_data(datadict):
    frames_data = []
    for i, _ in enumerate(datadict['angle [rad]']):
        a = [(datadict['tvc.a.start_x [m]'][i], datadict['tvc.a.start_y [m]'][i]),
             (datadict['tvc.a.end_x [m]'][i], datadict['tvc.a.end_y [m]'][i])]
        b = [(datadict['tvc.b.start_x [m]'][i], datadict['tvc.b.start_y [m]'][i]),
             (datadict['tvc.b.end_x [m]'][i], datadict['tvc.b.end_y [m]'][i])]
        g = [(datadict['tvc.g.start_x [m]'][i], datadict['tvc.g.start_y [m]'][i]),
             (datadict['tvc.g.end_x [m]'][i], datadict['tvc.g.end_y [m]'][i])]
        l = [(datadict['tvc.l.start_x [m]'][i], datadict['tvc.l.start_y [m]'][i]),
             (datadict['tvc.l.end_x [m]'][i], datadict['tvc.l.end_y [m]'][i])]
        frames_data.append([a, b, g, l])
    return frames_data


# Set up the figure
def get_animation(fig, frames_data, interval):

    # Create the animation with the specified interval
    return FuncAnimation(
        fig,
        update,
        frames=len(frames_data),
        repeat=False,
        fargs=(frames_data,),
        interval=interval
    )


# Function to update the lines for each frame
def update(frame, frames_data):
    plt.clf()  # Clear the previous frame
    plt.axis('equal')  # Set equal aspect ratio
    plt.xlim(-5, 5)  # Set x-axis limits
    plt.ylim(-5, 5)  # Set y-axis limits

    # List to store Line2D objects for legend
    legend_lines = []

    # Custom names for each line
    custom_names = ['a', 'b', 'g', 'l']

    for i, line_data in enumerate(frames_data[frame]):
        start_point, end_point = line_data[0], line_data[1]
        label = custom_names[i] if i < len(custom_names) else f'Line {i + 1}'
        line, = plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], marker='o', label=label)
        legend_lines.append(line)

    # Display the legend
    plt.legend(handles=legend_lines, loc='upper right')

    plt.title(f'Frame {frame + 1}')



# Example usage
if __name__ == "__main__":
    # Define your points using a numpy array
    fps = 60
    datadict = csv_to_datadict('data/tvc_sinsweep.csv', fps=60, time_key='angle [rad]')

    fig, ax = plt.subplots()
    frames_data = get_frames_data(datadict)
    animation = get_animation(fig, frames_data, 1/fps)

    plt.show()
