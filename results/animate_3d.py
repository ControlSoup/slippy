import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from file_handling import csv_to_datadict

class Axis():
    def __init__(
        self,
        pos_data_m: np.array,
        red_data_m: np.array = None,
        green_data_m: np.array = None,
        blue_data_m: np.array = None,
        axis_scale = 1.0
    ):
        self.pos_data_m = pos_data_m.copy()

        if red_data_m is None:
            self.red_data_m = [
                np.ones_like(self.pos_data_m[0]),
                np.zeros_like(self.pos_data_m[0]),
                np.zeros_like(self.pos_data_m[0])
            ]
        else:
            self.red_data_m = red_data_m.copy() 

        if green_data_m is None:
            self.green_data_m = [
                np.zeros_like(self.pos_data_m[1]),
                np.ones_like(self.pos_data_m[1]),
                np.zeros_like(self.pos_data_m[1])
            ]
        else:
            self.green_data_m = green_data_m.copy()

        if blue_data_m is None:
            self.blue_data_m = [
                np.zeros_like(self.pos_data_m[2]),
                np.zeros_like(self.pos_data_m[2]),
                np.ones_like(self.pos_data_m[2])
            ]
        else:
            self.blue_data_m = blue_data_m.copy()

        self.pos_data_m = np.array(self.pos_data_m, dtype = float)
        self.red_data_m = np.array(self.red_data_m, dtype = float) * axis_scale
        self.green_data_m = np.array(self.green_data_m, dtype = float) * axis_scale
        self.blue_data_m = np.array(self.blue_data_m, dtype = float) * axis_scale

    def origin(sample_data: np.array, axis_scale = 1.0):
        return Axis(
            pos_data_m=[
                np.zeros_like(sample_data),
                np.zeros_like(sample_data),
                np.zeros_like(sample_data),
            ],
            axis_scale=axis_scale
        )

    def plot(self, ax, frame):
        x1 = self.pos_data_m[0][frame]
        y1 = self.pos_data_m[1][frame]
        z1 = self.pos_data_m[2][frame]

        red_x2 = self.pos_data_m[0][frame] + self.red_data_m[0][frame]
        red_y2 = self.pos_data_m[1][frame] + self.red_data_m[1][frame]
        red_z2 = self.pos_data_m[2][frame] + self.red_data_m[2][frame]

        green_x2 = self.pos_data_m[0][frame] + self.green_data_m[0][frame]
        green_y2 = self.pos_data_m[1][frame] + self.green_data_m[1][frame]
        green_z2 = self.pos_data_m[2][frame] + self.green_data_m[2][frame]

        blue_x2 = self.pos_data_m[0][frame] + self.blue_data_m[0][frame]
        blue_y2 = self.pos_data_m[1][frame] + self.blue_data_m[1][frame]
        blue_z2 = self.pos_data_m[2][frame] + self.blue_data_m[2][frame]

        ax.plot([x1,red_x2], [y1,red_y2], [z1,red_z2], color="red")
        ax.plot([x1,blue_x2], [y1,blue_y2], [z1,blue_z2], color="blue")
        ax.plot([x1,green_x2], [y1,green_y2], [z1,green_z2], color="green")

    def get_max_value(self):
        return max(
            self.pos_data_m[0].max() + self.red_data_m[0].max() + self.green_data_m[0].max() + self.blue_data_m[0].max(),
            self.pos_data_m[1].max() + self.red_data_m[1].max() + self.green_data_m[1].max() + self.blue_data_m[1].max(),
            self.pos_data_m[2].max() + self.red_data_m[2].max() + self.green_data_m[2].max() + self.blue_data_m[2].max(),
        )

    def get_min_value(self):
        return min(
            self.pos_data_m[0].min() + self.red_data_m[0].min() + self.green_data_m[0].min() + self.blue_data_m[0].min(),
            self.pos_data_m[1].min() + self.red_data_m[1].min() + self.green_data_m[1].min() + self.blue_data_m[1].min(),
            self.pos_data_m[2].min() + self.red_data_m[2].min() + self.green_data_m[2].min() + self.blue_data_m[2].min(),
        )

class Curve():
    def __init__(
        self,
        pos_data_m: np.array,
        name: str = ' ', 
        color: str = "black" ,
        offset_pos_data: np.array = None
    ):
        self.pos_data_m = np.array(pos_data_m.copy())
        self.name = name
        self.color = color

        if offset_pos_data is None:
            self.offset_pos_data = [
                np.zeros_like(self.pos_data_m[0]),
                np.zeros_like(self.pos_data_m[1]),
                np.zeros_like(self.pos_data_m[2])
            ]
        else:
            self.offset_pos_data = offset_pos_data

    def plot(self, ax, frame):
        ax.plot(
            self.offset_pos_data[0][:frame] + self.pos_data_m[0][:frame], 
            self.offset_pos_data[1][:frame] + self.pos_data_m[1][:frame], 
            self.offset_pos_data[2][:frame] + self.pos_data_m[2][:frame], 
            color=self.color
        )

    def get_max_value(self):
        return max(
            self.offset_pos_data[0].max() + self.pos_data_m[0].max(),
            self.offset_pos_data[1].max() + self.pos_data_m[1].max(),
            self.offset_pos_data[2].max() + self.pos_data_m[2].max(),
        )

    def get_min_value(self):
        return min(
            self.offset_pos_data[0].min() + self.pos_data_m[0].min(),
            self.offset_pos_data[1].min() + self.pos_data_m[1].min(),
            self.offset_pos_data[2].min() + self.pos_data_m[2].min(),
        )

class Line():
    def __init__(
        self, 
        start_m: np.array, 
        end_m: np.array, 
        name: str = ' ', 
        color: str = "black" 
    ) -> None:
        self.name = name
        self.color = color
        self.start_m = start_m.copy()
        self.end_m = end_m.copy()
    
    def plot(self, ax, frame):
        x1 = self.start_m[0][frame]
        y1 = self.start_m[1][frame]
        z1 = self.start_m[2][frame]

        x2 = self.end_m[0][frame]
        y2 = self.end_m[1][frame]
        z2 = self.end_m[2][frame]

        ax.plot([x1,x2], [y1,y2], [z1,z2], color=self.color)
        
    
    def get_max_value(self):
        return max(
            self.start_m[0].max(), 
            self.start_m[1].max(), 
            self.start_m[2].max(),
            self.end_m[0].max(),
            self.end_m[1].max(),
            self.end_m[2].max()
        )
    
    def get_min_value(self):
        return min(
            self.start_m[0].min(), 
            self.start_m[1].min(), 
            self.start_m[2].min(),
            self.end_m[0].min(),
            self.end_m[1].min(),
            self.end_m[2].min()
        )

class Vector3():
    def __init__(
        self, 
        pos_data_m: np.array, 
        components: np.array, 
        name: str = ' ', 
        color: str = "black" 
    ) -> None:
        self.name = name
        self.color = color
        self.pos_data_m = np.array(pos_data_m.copy())
        self.components = np.array(components.copy())
    
    def from_origin(
        components: np.array, 
        name: str = '', 
        color: str = 'black'
    ):
        return Vector3(
            np.array([
                np.zeros_like(components[0]),
                np.zeros_like(components[1]),
                np.zeros_like(components[2])
            ]),
            components,
            name,
            color
        )

    def plot(self, ax, frame):
        x = self.pos_data_m[0][frame]
        y = self.pos_data_m[1][frame]
        z = self.pos_data_m[2][frame]

        u = self.components[0][frame]
        v = self.components[1][frame]
        w = self.components[2][frame]

        ax.quiver(x,y,z,u,v,w, color = self.color)
    
    def get_max_value(self):
        return max(
            self.pos_data_m[0].max() + self.components[0].max(), 
            self.pos_data_m[1].max() + self.components[1].max(), 
            self.pos_data_m[2].max() + self.components[2].max(),
        )
    
    def get_min_value(self):
        return min(
            self.pos_data_m[0].min() + self.components[0].min(), 
            self.pos_data_m[1].min() + self.components[1].min(), 
            self.pos_data_m[2].min() + self.components[2].min(),
        )

x_key = 'time [s]'
datadict = csv_to_datadict("data/test.csv", fps=10, time_key=x_key) 
fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

object_list = [
    # Axis.origin(sample_data=datadict[x_key], axis_scale=0.125),
    Axis(
        # pos_data_m = [
        #     np.zeros_like(datadict['time [s]']),
        #     np.zeros_like(datadict['time [s]']),
        #     np.zeros_like(datadict['time [s]']),
        # ],
        pos_data_m = [
            datadict['hopper.inertial_pos.x [m]'],
            datadict['hopper.inertial_pos.y [m]'],
            datadict['hopper.inertial_pos.z [m]']
        ],
        red_data_m=[
            datadict['hopper.dcm.c11 [-]'],
            datadict['hopper.dcm.c12 [-]'],
            datadict['hopper.dcm.c13 [-]']
        ],
        green_data_m=[
            datadict['hopper.dcm.c21 [-]'],
            datadict['hopper.dcm.c22 [-]'],
            datadict['hopper.dcm.c23 [-]']
        ],
        blue_data_m=[
            datadict['hopper.dcm.c31 [-]'],
            datadict['hopper.dcm.c32 [-]'],
            datadict['hopper.dcm.c33 [-]']
        ],
        axis_scale=1.0
    ),
    Curve(
        pos_data_m = [
            datadict['hopper.inertial_pos.x [m]'],
            datadict['hopper.inertial_pos.y [m]'],
            datadict['hopper.inertial_pos.z [m]']
        ],
        color='blue'
    ),
]

# x_key = 'angle [rad]'
# datadict = csv_to_datadict("data/tvc_sinsweep.csv", fps=10, time_key=x_key) 
# fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))
# object_list = [
#     Vector3.from_origin(
#         components = [
#             datadict['tvc.thrust.x [N]'],
#             datadict['tvc.thrust.y [N]'],
#             datadict['tvc.thrust.z [N]']
#         ],
#         color="orange"
#     ),
#     Curve(
#         pos_data_m = [
#             datadict['tvc.thrust.x [N]'],
#             datadict['tvc.thrust.y [N]'],
#             datadict['tvc.thrust.z [N]']
#         ],
#         color='blue'
#     ),
# ]


max = max([q.get_max_value() + 2.0 for q in object_list])
min = min([q.get_min_value() - 2.0 for q in object_list])


def update(frame):
    global quivers

    ax.clear()

    for obj in object_list:
        obj.plot(ax, frame)
        
    ax.set_title(f"Time: [{np.round(datadict[x_key][frame], 1)}]")
    ax.set_xlim(min, max)
    ax.set_ylim(min, max)
    ax.set_zlim(min, max)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")


ani = FuncAnimation(fig, update, frames=len(datadict[x_key]) - 1, interval=50)
plt.show()
