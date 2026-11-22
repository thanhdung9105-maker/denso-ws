import os

path = '/home/dung/denso_ws/src/denso_gazebo/launch/denso_gazebo_moveit.launch.py'
with open(path, 'r') as f:
    text = f.read()

text = text.replace("default_value='false'", "default_value='true'")

with open(path, 'w') as f:
    f.write(text)

print('Updated launch_rviz default_value to true!')
