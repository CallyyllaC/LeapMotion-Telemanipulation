//
// Depricated, no longer required due to using ros network to connect multiple PC's for dual leaps
//

#include "ros/ros.h"
#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "geometry_msgs/Point.h"
#include "leap_motion/Gesture.h"
#include "vm_leap/Modified_leap.h"

class VMLeap
{
private:
  ros::NodeHandle node; 
  ros::Publisher publisher;
  ros::Subscriber subscriber;

  void Leap_filteredCallback(const leap_motion::Human::ConstPtr& msg)
  {
    leap_motion::Hand handLeft = msg->left_hand;
    leap_motion::Hand handRight = msg->right_hand;
    
    //create output message
    vm_leap::Modified_leap output;

    //get the palm point objects
    geometry_msgs::Point leftPalm = handLeft.palm_center;
    geometry_msgs::Point rightPalm = handRight.palm_center;

    //Get center of mass of both hands
    output.left_location = {leftPalm.x, leftPalm.y, leftPalm.z};
    output.right_location = {rightPalm.x, rightPalm.y, rightPalm.z};

    //Get the roll, pitch and yaw of both hands in radians
    output.left_orientation = {handLeft.roll, handLeft.pitch, handLeft.yaw};
    output.right_orientation = {handRight.roll, handRight.pitch, handRight.yaw};

    output.grab = handLeft.grab_strength;
    output.pinch = handRight.pinch_strength;
    
    publisher.publish(output);
  }

public:
  VMLeap()
  {
    //Topic you want to publish
    publisher = node.advertise<vm_leap::Modified_leap>("vm_leap", 1);

    //Topic you want to subscribe
    subscriber = node.subscribe("/leap2/leap_motion/leap_filtered", 1, &VMLeap::Leap_filteredCallback, this);
  }

};//End of class VMLeap

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "vmleap");
  //Create an object of class VMLeap that will take care of everything
  VMLeap VMLeapObj;

  ros::spin();

  return 0;
}
