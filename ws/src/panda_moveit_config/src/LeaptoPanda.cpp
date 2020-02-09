#include "ros/ros.h"
#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "geometry_msgs/Point.h"
#include "leap_motion/Gesture.h"
#include "panda_moveit_config/Modified_leap.h"

class LeapToPanda
{
private:
  ros::NodeHandle node; 
  ros::Publisher publisher;
  ros::Subscriber subscriber1;
  ros::Subscriber subscriber2;
  ros::Timer timer;
  ros::Timer timer2;
  leap_motion::Hand handLeft1;
  leap_motion::Hand handRight1;
  leap_motion::Hand handLeft2;
  leap_motion::Hand handRight2;
  bool DualLeaps;
  int CalibrationDistance;

  double comparison(double x1, double x2, double c1, double c2)
  {
    if (x1 == 0)
    {
      return x2;
    }
    else if (x2 == 0 || c1 > c2)
    {
      return x1;
    }
    else
    {
      return x2;
    }    
  }

  double comparison(double x1, double x2, double c1, double c2, int d)
  {
    if (x1 == 0)
    {
      return x2 - (d/2);
    }
    else if (x2 == 0 || c1 > c2)
    {
      return x1 + (d/2);
    }
    else
    {
      return x2 - (d/2);
    }    
  }

  void dual_leap_publish(const ros::TimerEvent& event)
  {
    //create output message
    panda_moveit_config::Modified_leap output;

    //get the palm point objects
    geometry_msgs::Point leftPalm1 = handLeft1.palm_center;
    geometry_msgs::Point rightPalm1 = handRight1.palm_center;
    geometry_msgs::Point leftPalm2 = handLeft2.palm_center;
    geometry_msgs::Point rightPalm2 = handRight2.palm_center;
    
    //Get center of mass of both hands
    output.left_location = {comparison(leftPalm1.x, leftPalm2.x, handLeft1.confidence, handLeft2.confidence, CalibrationDistance), comparison(leftPalm1.y, leftPalm2.y, handLeft1.confidence, handLeft2.confidence), comparison(leftPalm1.z, leftPalm2.z, handLeft1.confidence, handLeft2.confidence)};
    output.right_location = {comparison(rightPalm1.x, rightPalm2.x, handRight1.confidence, handRight2.confidence, CalibrationDistance), comparison(rightPalm1.y, rightPalm2.y, handRight1.confidence, handRight2.confidence), comparison(rightPalm1.z, rightPalm2.z, handRight1.confidence, handRight2.confidence)};

    //Get the roll, pitch and yaw of both hands in radians
    output.left_orientation = {comparison(handLeft1.roll, handLeft2.roll, handLeft1.confidence, handLeft2.confidence), comparison(handLeft1.pitch, handLeft2.pitch, handLeft1.confidence, handLeft2.confidence), comparison(handLeft1.yaw, handLeft2.yaw, handLeft1.confidence, handLeft2.confidence)};
    output.right_orientation = {comparison(handRight1.roll, handRight2.roll, handRight1.confidence, handRight2.confidence), comparison(handRight1.pitch, handRight2.pitch, handRight1.confidence, handRight2.confidence), comparison(handRight1.yaw, handRight2.yaw, handRight1.confidence, handRight2.confidence)};

    publisher.publish(output);
  }

  void leap_publish(const ros::TimerEvent& event)
  {
    //create output message
    panda_moveit_config::Modified_leap output;

    //get the palm point objects
    geometry_msgs::Point leftPalm = handLeft1.palm_center;
    geometry_msgs::Point rightPalm = handRight1.palm_center;

    //Get center of mass of both hands
    output.left_location = {leftPalm.x, leftPalm.y, leftPalm.z};
    output.right_location = {rightPalm.x, rightPalm.y, rightPalm.z};

    //Get the roll, pitch and yaw of both hands in radians
    output.left_orientation = {handLeft1.roll, handLeft1.pitch, handLeft1.yaw};
    output.right_orientation = {handRight1.roll, handRight1.pitch, handRight1.yaw};

    publisher.publish(output);
  }

  void Leap_filteredCallback1(const leap_motion::Human::ConstPtr& msg)
  {
    handLeft1 = msg->left_hand;
    handRight1 = msg->right_hand;
  }

  void Leap_filteredCallback2(const leap_motion::Human::ConstPtr& msg)
  {
    handLeft2 = msg->left_hand;
    handRight2 = msg->right_hand;
  }

public:
  LeapToPanda()
  {    
    //are you using two leap motions?
    DualLeaps = true;
    //if so how far apart are they in milimeters centre to centre?
    CalibrationDistance = 388;

    //Topic you want to publish
    publisher = node.advertise<panda_moveit_config::Modified_leap>("leap_to_panda", 1);

    //Topic you want to subscribe
    subscriber1 = node.subscribe("/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback1, this);
    
    if (DualLeaps)
    {
      subscriber2 = node.subscribe("/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback2, this);
      timer2 = node.createTimer(ros::Duration(1), &LeapToPanda::dual_leap_publish, this);
    }
    else
    {
      timer = node.createTimer(ros::Duration(0.1), &LeapToPanda::leap_publish, this);
    }    
  }

};//End of class LeapToPanda

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "leapToPanda");
  //Create an object of class LeapToPanda that will take care of everything
  LeapToPanda LeapToPandaObj;

  ros::spin();

  return 0;
}
