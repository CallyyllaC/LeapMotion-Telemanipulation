//References
//Wiki.ros.org. (2019). ROS/Tutorials/Writingpublishersubscriber(C++) - ROS Wiki. [online] Available at: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B) [Accessed 9 February 2020].
//Wiki.ros.org. (2017). Roscpp/Overview/Timers - ROS Wiki. [online] Available at: https://wiki.ros.org/roscpp/Overview/Timers [Accessed 9 February 2020].
//Docs.ros.org. (2020). Geometry_Msgs/Pose Documentation. [online] Available at: https://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html [Accessed 9 February 2020].
//GitHub. (2013). ros-drivers/leap_motion. [online] Available at: https://github.com/ros-drivers/leap_motion/tree/hydro/ [Accessed 25 Jan. 2020].

#include "ros/ros.h"
#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "geometry_msgs/Point.h"
#include "leap_motion/Gesture.h"
#include "leap_panda_telemanipulation/Modified_leap.h"

class LeapToPanda
{
private:
  //class variables
  ros::NodeHandle node;         //ROS Node 
  ros::Publisher publisher;     //holds publisher
  ros::Subscriber subscriber1;  //holds host Leap Motion
  ros::Subscriber subscriber2;  //holds client Leap Motion
  ros::Timer timer;             //holds ROS timer
  leap_motion::Hand handLeft1;  //holds host leap left hand
  leap_motion::Hand handRight1; //holds host leap right hand
  leap_motion::Hand handLeft2;  //holds client leap left hand
  leap_motion::Hand handRight2; //holds client leap right hand
  bool DualLeaps;               //holds if multiple Leap Motions are to be used
  double CalibrationDistance;   //holds the distance between the two leap devices

  //Used to compare leap devices and return average value
  double comparison(double x1, double x2)
  {
    //if hand isnt detected on leap 1 return leap 2
    if (x1 == (double)0)
    {
      return x2;
    }
    //if hand isnt detected on leap 2 return leap 1
    else if (x2 == (double)0)
    {
      return x1;
    }
    //otherwise return average
    else
    {
      return (x1 + x2)/2;
    }    
  }

  //overload compare function that takes in the calibration distance to return the modified x value by either adding or subtracting half of the calibration distance
  double comparison(double x1, double x2, double d)
  {
    //if hand isnt detected on leap 1 return leap 2
    if (x1 == (double)0)
    {
      return x2 - (d/2);
    }
    //if hand isnt detected on leap 2 return leap 1
    else if (x2 == (double)0)
    {
      return x1 + (d/2);
    }
    //otherwise return average
    else
    {
      return ( (x2 - (d/2)) + (x1 + (d/2)) )/2;
    }    
  }

  //timer called publish function for when using two leap devices
  void dual_leap_publish(const ros::TimerEvent& event)
  {
    //create output message
    leap_panda_telemanipulation::Modified_leap output;

    //get the palm point objects
    geometry_msgs::Point leftPalm1 = handLeft1.palm_center;
    geometry_msgs::Point rightPalm1 = handRight1.palm_center;
    geometry_msgs::Point leftPalm2 = handLeft2.palm_center;
    geometry_msgs::Point rightPalm2 = handRight2.palm_center;
    
    //Get center of mass of both hands
    output.left_location = {comparison(leftPalm1.x, leftPalm2.x, CalibrationDistance), comparison(leftPalm1.y, leftPalm2.y), comparison(leftPalm1.z, leftPalm2.z)};
    output.right_location = {comparison(rightPalm1.x, rightPalm2.x, CalibrationDistance), comparison(rightPalm1.y, rightPalm2.y), comparison(rightPalm1.z, rightPalm2.z)};

    //Get the roll, pitch and yaw of both hands in radians
    output.left_orientation = {comparison(handLeft1.roll, handLeft2.roll), comparison(handLeft1.pitch, handLeft2.pitch), comparison(handLeft1.yaw, handLeft2.yaw)};
    output.right_orientation = {comparison(handRight1.roll, handRight2.roll), comparison(handRight1.pitch, handRight2.pitch), comparison(handRight1.yaw, handRight2.yaw)};

    //Get the grab and pinch strengths
    output.grab = comparison(handLeft1.grab_strength, handLeft2.grab_strength);
    output.pinch = comparison(handRight1.pinch_strength, handRight2.pinch_strength);

    //sets the calibration distance
    output.calibration = CalibrationDistance;

    //Publish the new Modified_Leap message
    publisher.publish(output);
  }

  //timer called publish function for when using a single leap device
  void leap_publish(const ros::TimerEvent& event)
  {
    //create output message
    leap_panda_telemanipulation::Modified_leap output;

    //get the palm point objects
    geometry_msgs::Point leftPalm = handLeft1.palm_center;
    geometry_msgs::Point rightPalm = handRight1.palm_center;

    //Get center of mass of both hands
    output.left_location = {leftPalm.x, leftPalm.y, leftPalm.z};
    output.right_location = {rightPalm.x, rightPalm.y, rightPalm.z};

    //Get the roll, pitch and yaw of both hands in radians
    output.left_orientation = {handLeft1.roll, handLeft1.pitch, handLeft1.yaw};
    output.right_orientation = {handRight1.roll, handRight1.pitch, handRight1.yaw};

    //Get the grab and pinch strengths
    output.grab = handLeft1.grab_strength;
    output.pinch = handRight1.pinch_strength;

    //sets the calibration distance
    output.calibration = 0;

    //Publish the new Modified_Leap message
    publisher.publish(output);
  }

  //leap subscriber callback to assign host device to class variables
  void Leap_filteredCallback1(const leap_motion::Human::ConstPtr& msg)
  {
    handLeft1 = msg->left_hand;
    handRight1 = msg->right_hand;
  }

  //leap subscriber callback to assign host device to class variables
  void Leap_filteredCallback2(const leap_motion::Human::ConstPtr& msg)
  {

    handLeft2 = msg->left_hand;
    handRight2 = msg->right_hand;
  }

public:
  //Constructor
  LeapToPanda()
  {    
    //are you using two leap motions?
    DualLeaps = true;
    //if so how far apart are they in meters centre to centre?
    CalibrationDistance = 0.32;

    //Topic you want to publish
    publisher = node.advertise<leap_panda_telemanipulation::Modified_leap>("leap_to_panda", 1);

    //Topic of the host leap you want to subscribe to
    subscriber1 = node.subscribe("/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback1, this);

    //if using two leaps get the other subscriber and activate the correct publish timer accordingly
    //timers are used to publish 10 times a second in order to prevent oversaturating the network
    if (DualLeaps)
    {
      //Topic of the host leap you want to subscribe to
      subscriber2 = node.subscribe("/leap2/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback2, this);
      timer = node.createTimer(ros::Duration(0.1), &LeapToPanda::dual_leap_publish, this);
    }
    else
    {
      timer = node.createTimer(ros::Duration(0.1), &LeapToPanda::leap_publish, this);
    }    
  }

};

//Main
int main(int argc, char **argv)
{
  //Init ROS
  ros::init(argc, argv, "leapToPanda");
  //Create an object of class LeapToPanda
  LeapToPanda LeapToPandaObj;

  //Spin
  ros::spin();

  return 0;
}
