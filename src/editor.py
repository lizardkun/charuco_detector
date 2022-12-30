#!/usr/bin/env python3
import os
import yaml


with open(r'data.yml') as file:
    pose = yaml.unsafe_load(file)


with open(r'virat_template.launch', 'r') as file:
  
    # Reading the content of the file
    # using the read() function and storing
    # them in a new variable
    data = file.read()
  
    # Searching and replacing the text
    # using the replace() function
    data = data.replace("$@camx$@",str(pose['cam_pos_x']))
    data = data.replace("$@camy$@", str(pose['cam_pos_y']))
    data = data.replace("$@camz$@", str(pose['cam_pos_z']))
    data = data.replace("$@camr$@", str(pose['cam_roll']))
    data = data.replace("$@camp$@", str(pose['cam_pitch']))
    data = data.replace("$@camw$@", str(pose['cam_yaw']))
with open(r'virat2.launch', 'w') as file:
  
    # Writing the replaced data in our
    # text file
    file.write(data)
