#include <iostream>
#include <vector>
#include <cstdlib>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>


ros::NodeHandle n;
ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("triggerTurnover");
std_srvs::Trigger srv;



void callback(const geometry_msgs::Pose& msg)
{
    
    double warning_angle = 225./180 *3.14;

    if(msg.orientation.z>warning_angle)
    {
        client.call(srv);
        ROS_INFO("Turnover triggered");
    }

}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "turnover_detection");
    
  ros::Subscriber sub = n.subscribe("move_base_topic/cmd_vel", 1, callback); 
  ros::Rate r(10);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }

  return 0;

}