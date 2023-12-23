import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class Vector3():
    def __init__(self, start: np.array, end: np.array, name = ' ', color = 'black') -> None:
        self.name = name
        self.color = color
        self.start = start.copy()
        self.end = end.copy()
    
    def get_current_quiver(self, frame):
        x = self.start[0][frame]
        y = self.start[1][frame]
        z = self.start[2][frame]

        u = self.end[0][frame] - self.start[0][frame]
        v = self.end[1][frame] - self.start[1][frame]
        w = self.end[2][frame] - self.start[2][frame]

        return x,y,z,u,v,w
    
    def get_max_value(self):
        return max(
            self.start[0].max(), 
            self.start[1].max(), 
            self.start[2].max(),
            self.end[0].max(),
            self.end[1].max(),
            self.end[2].max()
        )
    
    def get_min_value(self):
        return min(
            self.start[0].max(), 
            self.start[1].max(), 
            self.start[2].max(),
            self.end[0].max(),
            self.end[1].max(),
            self.end[2].max()
        )


fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

range = np.arange(0,1,0.1)
quivers = [
    Vector3([range, range, range],[range + 1 , np.arange(0,1, 0.1) * 2 , range + 1]),
    Vector3([range, range, range],[range + 3 , np.arange(0,1, 0.1) * 2 , range + 1]),
    Vector3([range, range, range],[range - 1 , np.arange(0,1, 0.1) * 2 , range + 1])
],

max = -100
for quiver in quivers:
        
def update(frame):
    global quivers

    ax.clear()

    for quiver in quivers:
        ax.quiver(*quiver.get_current_quiver(frame))

    ax.set_xlim(max)
    ax.set_ylim(max)
    ax.set_zlim(max)

ani = FuncAnimation(fig, update, frames=len(range) - 1, interval=50)
plt.show()
