#!/bin/bash
# A simple script to launch Ros core
sudo gnome-terminal --tab --title="ROS core" --command="bash -i  -c 'cd ~/ws; source ~/.bashrc; roscore; $SHELL'"

