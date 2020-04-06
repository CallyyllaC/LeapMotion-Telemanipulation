# LeapMotion-Telemanipulation

## About<br>
This project is used to control a PANDA arm using a Leap motion controller. This package includes the files for either one or two Leap Motion devices to be used, as well as example code for both changing how multiple devices work, as well as how to expand to any amount of Leap Motion devices, these are found in the Example directory. The instructions of how to set up the devices for both 1 or 2 Leap Motions is described below, as well as in different situations (physical or virtual machines).
<br>
This repo also contains a launch scripts directory the launch scripts that I used to run all the bash commands were included, these will need to be modified to the catkin workspace that you use (mine was called "ws"), I used these for conveniance and they are labeled based on what they do.
<br>
## Info<br>
This was done using ubuntu 18.04lts however should be recreatable on other linux versions with little or no modification.<br>
This repo contains two packages, if using a single Leap Motion, you do not require the VM package<br>
Using multiple Leap Motions through a virtual machine is optional, also a seperate computer may be used instead<br>

### Installing with one Leap Motion:
<ol><li>Install ROS</li>
  <li>Install <a href="https://developer.leapmotion.com/setup/desktop">Leap motion</a> sdk driver</li>
<li>Later versions of ubuntu* require <a href="https://d2beseu6pw5d2t.cloudfront.net/t/tip-ubuntu-systemd-and-leapd/2118">this fix </a>
</li>
  <li>Install <a href="https://wiki.ros.org/leap_motion">leap motion</a> for ros</li>
  <li>Install <a href="https://wiki.ros.org/moveit">moveit</a> for ros including panda_moveit_config</li>
  <li>Install tkinter for python `sudo apt-get install python-tk`</li>
  <li>Install 3dtransform for python 'pip install transforms3d'</li>
<li>Copy over leap_panda_telemanipulation into your catkin workspace src folder</li>
</ol>
### Launching with one Leap Motion:
<ol><li>Plug in the first Leap Motion</li>
<li>Launch the leapd service, ros leap motion, and the leap_to_panda packages on the host machine</li>
</ol>
<br>
<img src="https://i.imgur.com/ocGtBR9.png" width="350" title="Single leap motion example">
<br>

### Installing with two Leap Motions:
<ol><li>Install ROS</li>
<li>Install <a href="https://developer.leapmotion.com/setup/desktop">Leap motion</a> sdk driver</li>
<li>Later versions of ubuntu* require <a href="https://d2beseu6pw5d2t.cloudfront.net/t/tip-ubuntu-systemd-and-leapd/2118">this fix </a>
</li>
<li>Install <a href="https://wiki.ros.org/leap_motion">leap motion</a> for ros</li>
<li>Install <a href="https://wiki.ros.org/moveit">moveit</a> for ros including panda_moveit_config</li>
<li>Install tkinter for python `sudo apt-get install python-tk`</li>
<li>Install 3dtransform for python 'pip install transforms3d'</li>
<li>Copy over leap_panda_telemanipulation and vm_leap into your catkin workspace src folder</li>
<li>If you are using a virtual machine download your prefered virtual machine software and set up an os, I used vmware player running ubuntu 18lts</li>
<li>Complete steps 1 to 4 again on either your new virtual machine, or your secondary computer</li>
<li>Copy over vm_leap into your second/virtual machines catkin workspace src folder</li>
<li>Set up the ros network on both machines so that the host pc is the master</li>
</ol>

### Launching with two Leap Motions on a VM:
<ol><li>Plug the first Leap Motion into a "less dominant usb socket"**</li>
<li>Pass that usb device into the virtual machine</li>
<li>Start ros core on the host machine</li>
<li>Launch the leapd service, ros leap motion, and the vm_leap packages on the virtual machine</li>
<li>Plug in second Leap Motion</li>
<li>Launch the leapd service, ros leap motion, and the leap_to_panda packages on the host machine</li>
</ol>

### Launching with two Leap Motions on a seperate machine:
<ol><li>Plug in your Leap Motions to each machine</li>
<li>Start ros core on the host machine</li>
<li>Launch the leapd service, ros leap motion, and the vm leap packages on the second machine</li>
<li>Launch the leapd service, ros leap motion, moveit, and the the leap panda telemanipulation packages on the host machine</li>
</ol>
<br>
<img src="https://i.imgur.com/DHMNygB.jpg" width="350" title="VM Packages running on client machine">
<br>
<img src="https://i.imgur.com/UfBRGTK.jpg" width="350" title="Dual leap motion example">
<br>

### Configuring the leap panda telemanipulation package
<ul><li>To specify the usage of either one or two Leap Motion devices, goto `LeaptoPanda.cpp` and set the bool `DualLeaps` to true if not ensure that it is set to false</li>
<li>In order to change the distance between multiple leap devices, goto `LeaptoPanda.cpp` and set the double `CalibrationDistance` to the distance between the center of the two leap devices in meters. You also need to set self.Calibration` in LeapMoveGroup.py to the same value, if you are only using a single leap device, please ensure that `self.Calibration` is set to 0</li></ul>
  
### How To Use
<ul><li>The panda arm by default will follow the users right hand, as long as it is in reach</li>
<li>The panda arms gripper will open and close based off of the users right hand pinching (thumb and index finger)</li>
<li>The left hand is used to move the workspace around but only whist recognising a closed fist gesture</li></ul>
  
### Key Points
<ul><li>When using two Leap Motion devices, the host Leap device should be on the left hand side by default</li>
<li>The final position is calculated (offset + Leap position) * scale</li>
<li>Offset variables are based off of the raw Leap Motion positions and will not be affected by scale</li>
<li>When using two Leap Motion devices, if a hand is visible on both devices the one with the highest gesture confidence is taken, if these are the same then the left device is always prefered</li>
<li>When using two Leap Motion devices, if no hands are detected, the ui will show -calibration/2 for the x positions, this will not affect the data in any way, this is just what is returned when no hand is detected</li>
</ul>

### Example Code
<ul><li>Provided is example code that was uses the alternate method of working out where then hand is when seen by multiple devices, this uses averaging rather than confidence score</li>
<li>Provided is example code that demonstrates the scalability of the project by giving a theoretical example of how 4 leap motions in a square would work, and what code would needed to be changed in order to make that happen</li>
</ul>

*\*I believe versions after ubuntu 14, maybe, I know for a fact 16lts and 18lts require this*

*\*\*essentially the first leap motion must be pluged in to a usb socket that the host will ignore, basically we want the 2nd leap to be plugged into a socket that the host machine leap service will discover first so that it will connect to that one and not the one we are using for the VM. This will be an issue until vmware usb virtualisation is imporved* 
