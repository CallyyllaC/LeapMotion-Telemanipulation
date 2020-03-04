#!/bin/bash
# A simple script to build catkin workspace /ws/
sudo gnome-terminal  --tab --title="Build WS" --command="bash -i -c 'cd ~/ws; source ~/.bashrc; sudo catkin build; $SHELL'"
