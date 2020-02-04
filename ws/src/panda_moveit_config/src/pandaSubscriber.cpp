#include "ros/ros.h"
#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "geometry_msgs/Point.h"
#include "leap_motion/Gesture.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * void Leap_filteredCallback(const std_msgs::String::ConstPtr& msg)
 */
void Leap_filteredCallback(const leap_motion::Human::ConstPtr& msg)
{
  //get the hand objects
  leap_motion::Hand handLeft = msg->left_hand;
  leap_motion::Hand handRight = msg->right_hand;

  //Get left hand center of mass and save it as CoML   
  geometry_msgs::Point leftPalm = handLeft.palm_center;
  double CoML [3] = {leftPalm.x, leftPalm.y, leftPalm.z};
  
  //Get right hand center of mass and save it as CoMR
  geometry_msgs::Point rightPalm = handRight.palm_center;
  double CoMR [3]= {rightPalm.x, rightPalm.y, rightPalm.z};

  //Get the roll, pitch and yaw of the hand in an array [r,p,y] in radians
  double rpyL [3] = {handLeft.roll, handLeft.pitch, handLeft.yaw};
  double rpyR [3] = {handRight.roll, handRight.pitch, handRight.yaw};


  //Get the current hand pose and its confidence
  bool valL = handLeft.valid_gestures;
  bool valR = handRight.valid_gestures;
  double conL = handLeft.confidence;
  double conR = handRight.confidence;
//  leap_motion::Gesture gesturesL [] = &handLeft.gesture_list;
//  leap_motion::Gesture gesturesR [] = &handRight.gesture_list;


  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "pandaSubscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/leap_motion/leap_filtered", 1000, Leap_filteredCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
