import bpy
import numpy as np
import sys
from pandas import read_csv

# Set the path to your CSV file
csv_file_path = 'data/test.csv'
object_name = 'body'

# Function to read CSV data
def csv_to_datadict(file_path: str, fps, time_key='time [s]'):
    df = read_csv(file_path)
    datadict ={}
    for key in df:
        datadict[key] = df[key].to_numpy()

    # Sub sample data
    dt = datadict[time_key][1] - datadict[time_key][0]
    samples =  int((1/fps) / dt)
    for key in datadict:
        datadict[key] = datadict[key][::samples]
    return datadict

# Function to set the location and rotation of the object at a specific frame
def set_object_transform(frame, obj, pos, quat):
    obj.show_axis = True
    obj.location = pos
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = quat
    obj.keyframe_insert(data_path="location", frame=frame)
    obj.keyframe_insert(data_path="rotation_quaternion", frame=frame)

# Calcualte fps to view in real time
fps = int(50)
bpy.context.scene.render.fps = fps

# Clear existing keyframes
bpy.ops.anim.keyframe_clear_v3d()

# Init datadict
datadict = csv_to_datadict(csv_file_path, fps)
obj = bpy.data.objects['Cube']

for i,_ in enumerate(datadict['time [s]']):
    frame = i
    pos = [
        datadict[f'{object_name}.pos.x [m]'][i],
        datadict[f'{object_name}.pos.y [m]'][i],
        datadict[f'{object_name}.pos.z [m]'][i]
    ]
    quat = [
        datadict[f'{object_name}.quat.a [-]'][i],
        datadict[f'{object_name}.quat.b [-]'][i],
        datadict[f'{object_name}.quat.c [-]'][i],
        datadict[f'{object_name}.quat.d [-]'][i]
    ]

    # Modify trajectory for the given frame
    set_object_transform(i, obj, pos, quat)


# Set the end frame to match the last frame in the animation
bpy.context.scene.frame_end = frame

# Set up rendering settings
# print(f"Total Frames :{frame}")
# bpy.context.scene.render.image_settings.file_format = 'FFMPEG'
# bpy.context.scene.render.ffmpeg.format = 'MPEG4'
# bpy.context.scene.render.ffmpeg.codec = 'H264'
# bpy.context.scene.render.filepath = 'output_animation.mp4'

# Render animation
# bpy.ops.render.render(animation=True)

bpy.ops.screen.animation_play()