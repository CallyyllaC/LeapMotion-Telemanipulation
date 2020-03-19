//References
//Wiki.ros.org. (2019). ROS/Tutorials/Writingpublishersubscriber(C++) - ROS Wiki. [online] Available at: https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B) [Accessed 9 February 2020].
//Wiki.ros.org. (2017). Roscpp/Overview/Timers - ROS Wiki. [online] Available at: https://wiki.ros.org/roscpp/Overview/Timers [Accessed 9 February 2020].
//Docs.ros.org. (2020). Geometry_Msgs/Pose Documentation. [online] Available at: https://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html [Accessed 9 February 2020].
//GitHub. (2013). ros-drivers/leap_motion. [online] Available at: https://github.com/ros-drivers/leap_motion/tree/hydro/ [Accessed 25 Jan. 2020].


//
//  An example script changing the support up to 4 Leap Motion devices
//  the devices are in the order:
//
//  1  2
//  3  4
//

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
  ros::Subscriber subscriber3;
  ros::Subscriber subscriber4;
  ros::Timer timer;             //holds ROS timer
  leap_motion::Hand handLeft1;  //holds host leap left hand
  leap_motion::Hand handRight1; //holds host leap right hand
  leap_motion::Hand handLeft2;  //holds client leap left hand
  leap_motion::Hand handRight2; //holds client leap right hand
  leap_motion::Hand handLeft3;
  leap_motion::Hand handRight3;
  leap_motion::Hand handLeft4;
  leap_motion::Hand handRight4;
  double CalibrationDistance;   //holds the distance between the two leap devices

  //Used to compare leap devices and return whichever device provides the best "confidence" value
  double comparison(double x1, double x2, double x3, double x4, double c1, double c2, double c3, double c4)
  {
    //edited confidence values
    double e1 = c1;
    double e2 = c2;
    double e3 = c3;
    double e4 = c4;

    //ensure that if the value is 0 so is the confidence
    if (x1 == (double)0)
      e1 = 0;
    else if (x2 == (double)0)
      e1 = 0;
    else if (x3 == (double)0)
      e1 = 0;
    else if (x4 == (double)0)
      e1 = 0;

    //if leap 1 has highest confidence
    if ( e1 > e2 && e1 > e3 && e1 > e4 )
      return x1;
    //if leap 1 has highest confidence
    else if ( e2 > e1 && e2 > e3 && e2 > e4 )
      return x2;
    //if leap 1 has highest confidence
    else if ( e3 > e2 && e3 > e1 && e3 > e4 )
      return x3;
    //otherwise return leap 4
    else
      return x4;
  }

  //overload compare function that takes in the calibration distance to return the modified x value by either adding or subtracting half of the calibration distance
  double comparisonx(double x1, double x2, double x3, double x4, double c1, double c2, double c3, double c4, double d)
  {
    //edited confidence values
    double e1 = c1;
    double e2 = c2;
    double e3 = c3;
    double e4 = c4;

    //ensure that if the value is 0 so is the confidence
    if (x1 == (double)0)
      e1 = 0;
    else if (x2 == (double)0)
      e1 = 0;
    else if (x3 == (double)0)
      e1 = 0;
    else if (x4 == (double)0)
      e1 = 0;

    //if leap 1 has highest confidence
    if ( e1 > e2 && e1 > e3 && e1 > e4 )
      return x1 + (d/2);
    //if leap 1 has highest confidence
    else if ( e2 > e1 && e2 > e3 && e2 > e4 )
      return x2 - (d/2);
    //if leap 1 has highest confidence
    else if ( e3 > e2 && e3 > e1 && e3 > e4 )
      return x3 + (d/2);
    //otherwise return leap 4
    else
      return x4 - (d/2);
  }

  //overload compare function that takes in the calibration distance to return the modified x value by either adding or subtracting half of the calibration distance
  double comparisonz(double x1, double x2, double x3, double x4, double c1, double c2, double c3, double c4, double d)
  {
    //edited confidence values
    double e1 = c1;
    double e2 = c2;
    double e3 = c3;
    double e4 = c4;

    //ensure that if the value is 0 so is the confidence
    if (x1 == (double)0)
      e1 = 0;
    else if (x2 == (double)0)
      e1 = 0;
    else if (x3 == (double)0)
      e1 = 0;
    else if (x4 == (double)0)
      e1 = 0;

    //if leap 1 has highest confidence
    if ( e1 > e2 && e1 > e3 && e1 > e4 )
      return x1 - (d/2);
    //if leap 1 has highest confidence
    else if ( e2 > e1 && e2 > e3 && e2 > e4 )
      return x2 - (d/2);
    //if leap 1 has highest confidence
    else if ( e3 > e2 && e3 > e1 && e3 > e4 )
      return x3 + (d/2);
    //otherwise return leap 4
    else
      return x4 + (d/2);
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
    geometry_msgs::Point leftPalm3 = handLeft3.palm_center;
    geometry_msgs::Point rightPalm3 = handRight3.palm_center;
    geometry_msgs::Point leftPalm4 = handLeft4.palm_center;
    geometry_msgs::Point rightPalm4 = handRight4.palm_center;
    
    //Get center of mass of both hands
    output.left_location = {comparisonx(leftPalm1.x, leftPalm2.x, leftPalm3.x, leftPalm4.x, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence, CalibrationDistance), comparison(leftPalm1.y, leftPalm2.y, leftPalm3.y, leftPalm4.y, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence), comparisonz(leftPalm1.z, leftPalm2.z, leftPalm3.z, leftPalm4.z, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence, CalibrationDistance)};
    output.right_location = {comparisonx(rightPalm1.x, rightPalm2.x, rightPalm3.x, rightPalm4.x, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence, CalibrationDistance), comparison(rightPalm1.y, rightPalm2.y, rightPalm3.y, rightPalm4.y, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence), comparisonz(rightPalm1.z, rightPalm2.z, rightPalm3.z, rightPalm4.z, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence, CalibrationDistance)};

    //Get the roll, pitch and yaw of both hands in radians
    output.left_orientation = {comparison(handLeft1.roll, handLeft2.roll, handLeft3.roll, handLeft4.roll, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence), comparison(handLeft1.pitch, handLeft2.pitch, handLeft3.pitch, handLeft4.pitch, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence), comparison(handLeft1.yaw, handLeft2.yaw, handLeft3.yaw, handLeft4.yaw, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence)};
    output.right_orientation = {comparison(handRight1.roll, handRight2.roll, handRight3.roll, handRight4.roll, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence), comparison(handRight1.pitch, handRight2.pitch, handRight3.pitch, handRight4.pitch, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence), comparison(handRight1.yaw, handRight2.yaw, handRight3.yaw, handRight4.yaw, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence)};

    //Get the grab and pinch strengths
    output.grab = comparison(handLeft1.grab_strength, handLeft2.grab_strength, handLeft3.grab_strength, handLeft4.grab_strength, handLeft1.confidence, handLeft2.confidence, handLeft3.confidence, handLeft4.confidence);
    output.pinch = comparison(handRight1.pinch_strength, handRight2.pinch_strength, handRight3.pinch_strength, handRight4.pinch_strength, handRight1.confidence, handRight2.confidence, handRight3.confidence, handRight4.confidence);

    //sets the calibration distance
    output.calibration = CalibrationDistance;

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

  //leap subscriber callback to assign host device to class variables
  void Leap_filteredCallback3(const leap_motion::Human::ConstPtr& msg)
  {

    handLeft3 = msg->left_hand;
    handRight3 = msg->right_hand;
  }

  //leap subscriber callback to assign host device to class variables
  void Leap_filteredCallback4(const leap_motion::Human::ConstPtr& msg)
  {

    handLeft4 = msg->left_hand;
    handRight4 = msg->right_hand;
  }

public:
  //Constructor
  LeapToPanda()
  {
    //if so how far apart are they in meters centre to centre?
    CalibrationDistance = 0.32;

    //Topic you want to publish
    publisher = node.advertise<leap_panda_telemanipulation::Modified_leap>("leap_to_panda", 1);

    //Topic of the host leap you want to subscribe to
    subscriber1 = node.subscribe("/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback1, this);

    //Topic of the host leap you want to subscribe to
    subscriber2 = node.subscribe("/leap2/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback2, this);

    //Topic of the host leap you want to subscribe to
    subscriber2 = node.subscribe("/leap3/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback3, this);

    //Topic of the host leap you want to subscribe to
    subscriber2 = node.subscribe("/leap4/leap_motion/leap_filtered", 1, &LeapToPanda::Leap_filteredCallback4, this);

    timer = node.createTimer(ros::Duration(0.1), &LeapToPanda::dual_leap_publish, this);
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
