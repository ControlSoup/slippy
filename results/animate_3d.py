import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from file_handling import csv_to_datadict

class Curve():
    def __init__(
        self,
        pos_data: np.array,
        name: str = ' ', 
        color: str = "black" 
    ):
        self.pos_data = pos_data
        self.name = name
        self.color = color

    def plot(self, ax, frame):
        ax.plot(self.pos_data[0][:frame], self.pos_data[1][:frame], self.pos_data[2][:frame], color=self.color)

    def get_max_value(self):
        return max(
            self.pos_data[0].max(),
            self.pos_data[1].max(),
            self.pos_data[2].max(),
        )

    def get_min_value(self):
        return min(
            self.pos_data[0].min(),
            self.pos_data[1].min(),
            self.pos_data[2].min(),
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
        pos_m: np.array, 
        components: np.array, 
        name: str = ' ', 
        color: str = "black" 
    ) -> None:
        self.name = name
        self.color = color
        self.pos_m = pos_m.copy()
        self.components = components.copy()
    
    def plot(self, ax, frame):
        x = self.pos_m[0][frame]
        y = self.pos_m[1][frame]
        z = self.pos_m[2][frame]

        u = self.components[0][frame]
        v = self.components[1][frame]
        w = self.components[2][frame]

        ax.quiver(x,y,z,u,v,w, color = self.color)
    
    def get_max_value(self):
        return max(
            self.pos_m[0].max() + self.components[0].max() + 2.0, 
            self.pos_m[1].max() + self.components[1].max() + 2.0, 
            self.pos_m[2].max() + self.components[2].max() + 2.0,
        )
    
    def get_min_value(self):
        return min(
            self.pos_m[0].min() + self.components[0].min() - 2.0, 
            self.pos_m[1].min() + self.components[1].min() - 2.0, 
            self.pos_m[2].min() + self.components[2].min() - 2.0,
        )

x_key = 'angle [rad]'
datadict = csv_to_datadict("data/tvc_sinsweep.csv", fps=15, time_key=x_key) 
fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

object_list = [
    Vector3(
        [
            datadict['tvc.pos_joint.x [m]'],
            datadict['tvc.pos_joint.y [m]'],
            datadict['tvc.pos_joint.z [m]'],
        ],
        [
            datadict['tvc.thrust.x [N]'],
            datadict['tvc.thrust.y [N]'],
            datadict['tvc.thrust.z [N]'],
        ],
    ),
    Curve(
        [
            datadict['tvc.thrust.x [N]'],
            datadict['tvc.thrust.y [N]'],
            datadict['tvc.thrust.z [N]'],
        ],
        color="red"
    )

]


max = max([q.get_max_value() for q in object_list])
min = min([q.get_min_value() for q in object_list])


def update(frame):
    global quivers

    ax.clear()

    for obj in object_list:
        obj.plot(ax, frame)
        

    ax.set_xlim(min, max)
    ax.set_ylim(min, max)
    ax.set_zlim(min, max)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")


ani = FuncAnimation(fig, update, frames=len(datadict[x_key]) - 1, interval=50)
plt.show()
