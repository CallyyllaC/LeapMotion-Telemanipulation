#!/bin/bash
# A simple script to launch Riz project and leap motion service
sudo gnome-terminal --tab --title="leapd service" --command="bash -i -c 'sudo service leapd stop; sleep 2; sudo leapd; $SHELL'" --tab --title="bash -i -c 'sleep 4; leap control panel; $SHELL'" --command="sudo LeapControlPanel" --tab --title="Leap Panda Tele" --command="bash -i -c 'cd ~/ws; source ~/.bashrc; sleep 14; roslaunch vm_leap node.launch; $SHELL'"
