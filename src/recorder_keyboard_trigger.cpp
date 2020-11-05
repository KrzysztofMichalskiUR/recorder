#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include<recorder/SetState.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recorder_keyboard_trigger");
  ros::NodeHandle nh;
  ros::ServiceClient client1 = nh.serviceClient<std_srvs::Trigger>("triggerGhosting");
  ros::ServiceClient client2 = nh.serviceClient<std_srvs::Trigger>("triggerTurnover");
  ros::ServiceClient client3 = nh.serviceClient<recorder::SetState>("set_bag_state");
  std_srvs::Trigger srv1;
  std_srvs::Trigger srv2;
  recorder::SetState srv3;
  char c;
  ROS_INFO("Press 'g' to trigger recording ghosting and 't' to trigger recording turnover, 'q' to quit");
  while(ros::ok())
  {
    c = getchar();
    if(c == 'g')
      client1.call(srv1);
    else if(c=='t')
      client2.call(srv2);
    else if(c=='n'){
      srv3.request.state=0;
      client3.call(srv3);
    }
     else if(c=='r'){
      srv3.request.state=1;
      client3.call(srv3);
    }
     else if(c=='o'){
      srv3.request.state=2;
      client3.call(srv3);
    }
    else if(c == 'q')
      return 0;
  }  
  return 0;
}