# LeapMotion-Telemanipulation

## things to note<br>
This was done using ubuntu 18.04lts however should be recreatable on other linux versions with little or no modification.<br>
This project was build over the default panda demo rather than creating its own seperate package

### Steps to recreate:
<ol><li>install ROS</li>
<li>install Leap motion sdk driver</li>
<li>later versions of ubuntu* require <a href="https://d2beseu6pw5d2t.cloudfront.net/t/tip-ubuntu-systemd-and-leapd/2118">this fix </a>
</li>
<li>install leap motion for ros</li>
<li>install moveit for ros</li>
<li>copy over files in this repo  where ws/ is your catkin workspace</li>
</ol>

### Steps to start with two leap motions:
<ol><li>Plug the fisrt leap device into a "less dominant usb socket"**</li>
<li>Pass that usb device into the virtual machine</li>
<li>Start ros core on the host machine</li>
<li>Launch the leapd service, ros leap motion, and the vm_leap node on the virtual machine</li>
<li>Launch the leapd service, ros leap motion, and the leap_to_panda node on the virtual machine</li>
<li>launch the LeapMoveGroup script</li>
</ol>

*\*I believe versions after ubuntu 14, maybe, I know for a fact 16lts and 18lts require this*

*\*\*essentially the first leap motion must be pluged in to a usb socket that the host will ignore, basically we want the 2nd leap to be plugged into a socket that the host machine leap service will discover first so that it will connect to that one and not the one we are using for the VM. This will be an issue until vmware usb virtualisation is imporved* 
