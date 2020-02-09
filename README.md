# LeapMotion-Telemanipulation

## Info<br>
This was done using ubuntu 18.04lts however should be recreatable on other linux versions with little or no modification.<br>
This repo contains two packages, if using a single Leap Motion, you do not require the VM package<br>
Using multiple Leap Motions through a virtual machine is optional, and a seperate computer can be used instead<br>

### Installing with one Leap Motion:
<ol><li>Install ROS</li>
<li>Install Leap motion sdk driver</li>
<li>Later versions of ubuntu* require <a href="https://d2beseu6pw5d2t.cloudfront.net/t/tip-ubuntu-systemd-and-leapd/2118">this fix </a>
</li>
<li>Install leap motion for ros</li>
<li>Install moveit for ros including panda_moveit_config</li>
<li>Copy over leap_panda_telemanipulation into your catkin workspace src folder</li>
</ol>

### Launching with one Leap Motion:
<ol><li>Plug in the first Leap Motion</li>
<li>Launch the leapd service, ros leap motion, and the leap_to_panda node on the host machine</li>
<li>launch the LeapMoveGroup script</li>
</ol>

### Installing with two Leap Motions:
<ol><li>Install ROS</li>
<li>Install Leap motion sdk driver</li>
<li>Later versions of ubuntu* require <a href="https://d2beseu6pw5d2t.cloudfront.net/t/tip-ubuntu-systemd-and-leapd/2118">this fix </a>
</li>
<li>Install leap motion for ros</li>
<li>Install moveit for ros including panda_moveit_config</li>
<li>Copy over leap_panda_telemanipulation and vm_leap into your catkin workspace src folder</li>
<li>Download your prefered virtual machine software and set up an os, I used vmware player running ubuntu 18lts</li>
<li>Complete steps 1 to 4 again on your new virtual machine</li>
<li>Copy over vm_leap into your virtual machines catkin workspace src folder</li>
<li>Set up the ros network on both machines so that the host pc is the master</li>
<li></li>
</ol>

### Launching with two Leap Motions:
<ol><li>Plug the first Leap Motion into a "less dominant usb socket"**</li>
<li>Pass that usb device into the virtual machine</li>
<li>Start ros core on the host machine</li>
<li>Launch the leapd service, ros leap motion, and the vm_leap node on the virtual machine</li>
<li>Plug in second Leap Motion</li>
<li>Launch the leapd service, ros leap motion, and the leap_to_panda node on the host machine</li>
<li>launch the LeapMoveGroup script</li>
</ol>

*\*I believe versions after ubuntu 14, maybe, I know for a fact 16lts and 18lts require this*

*\*\*essentially the first leap motion must be pluged in to a usb socket that the host will ignore, basically we want the 2nd leap to be plugged into a socket that the host machine leap service will discover first so that it will connect to that one and not the one we are using for the VM. This will be an issue until vmware usb virtualisation is imporved* 
