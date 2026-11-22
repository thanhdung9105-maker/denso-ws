import os

path = '/home/dung/launch_denso_gazebo_moveit.sh'
with open(path, 'r') as f:
    text = f.read()

# Make sure launch_rviz:=true is passed explicitly when launching
if 'launch_rviz:=true' not in text:
    text = text.replace('denso_gazebo_moveit.launch.py "$@"', 'denso_gazebo_moveit.launch.py launch_rviz:=true "$@"')
    with open(path, 'w') as f:
        f.write(text)
    print('Updated launch_denso_gazebo_moveit.sh with launch_rviz:=true!')
else:
    print('Already has launch_rviz:=true')
