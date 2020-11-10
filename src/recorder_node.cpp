#include <rosbag/bag.h>
#include<ros/ros.h>
#include "../include/recorder/topics.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include<stdio.h>
#define STRLEN 256
#include <unistd.h>
#include <limits.h>
#include<string>
#include<iostream>
#include<mutex>
#include<recorder/SetState.h>
#include <fstream>


//Global variables



typedef enum BagState{BAG_NORMAL,BAG_RECOVERY,BAG_OFF} BagState;
	
	BagState stateCurr=BAG_NORMAL;
	BagState statePrev=BAG_NORMAL;
	BagState stateCurr_T=BAG_NORMAL;


bool triggeredG = false;
bool triggeredT = false;
bool recording = false;

std::mutex mtx;

//service callback (when mode is changed)
bool setState(recorder::SetState::Request &req,recorder::SetState::Response & resp)
{
	std::lock_guard<std::mutex> guard(mtx);
	stateCurr=static_cast<BagState>(req.state);
	ROS_INFO("Set state service recived: %d",stateCurr);
}


//Service callback (when ghost detected) changing value of 'triggered' parameter
bool triggerCbG(std_srvs::Trigger::Request& req,
             std_srvs::Trigger::Response& res)
{
  triggeredG = true;
  res.success = true;
  ROS_INFO("Triggered - ghosting");
  return true;
}


//Service callback (when turnover detected) changing value of 'triggered' parameter
bool triggerCbT(std_srvs::Trigger::Request& req,
             std_srvs::Trigger::Response& res)
{
  triggeredT = true;
  res.success = true;
  ROS_INFO("Triggered - turnover");
  return true;
}

int main(int argc, char** argv)
{

	ros::init(argc,argv,"recorder");
	ros::NodeHandle n;

	typedef std::array<std::string,2> _2str;
	std::vector<_2str> topics;
	std::vector<_2str> topics_recovery;

	char type[STRLEN],topicname[STRLEN],nr[STRLEN];


	// -------------------- loading parameters --------------------
    std::string homepath = getenv("HOME");
	
	std::string file1_dir; //absolute path
  	n.param<std::string>("file1_dir", file1_dir, homepath+"/catkin_ws/src/recorder/topics.txt"); //3rd value is a default value

	std::string file2_dir; // absolute path
  	n.param<std::string>("file2_dir", file2_dir, homepath+"/catkin_ws/src/recorder/topics_recovery.txt"); //3rd value is a default value

	std::string outputDir;
	n.param<std::string>("output_dir", outputDir, homepath+"/catkin_ws/src/recorder/bags"); //3rd value is a default value
	
	double beforeTrigger; // absolute path
	n.param("before_trigger", beforeTrigger, 10.0); //3rd value is a default value

	double afterTrigger;
	n.param("after_trigger", afterTrigger, 10.0); //3rd value is a default value

	//to have clear output
	std::cout<<"<------------------------------------------------------------------------------------->"<<std::endl;
	std::cout<<std::endl;
	std::cout<<std::endl;
	std::cout<<std::endl;
	std::cout<<std::endl;
	std::cout<<std::endl;

	std::cout<<"<*****\t\t\t Loading parameters and files \t\t\t*****>"<<std::endl<<std::endl;
	std::cout<<"Loaded parameters:"<<std::endl;
	std::cout<<"file1_dir: "<<file1_dir<<std::endl;
	std::cout<<"file2_dir: "<<file2_dir<<std::endl;
	std::cout<<"output_dir: "<<outputDir<<std::endl;
	std::cout<<"before_trigger: "<<beforeTrigger<<std::endl;
	std::cout<<"after_trigger: "<<afterTrigger<<std::endl;


	std::cout<<std::endl;


	// -------------------- services --------------------

	//trigger changing mode
	ros::ServiceServer ss = n.advertiseService("set_bag_state", setState);

	//ghosting trigger
	ros::ServiceServer serviceG = n.advertiseService("triggerGhosting", triggerCbG);

	//turnover trigger
	ros::ServiceServer serviceT = n.advertiseService("triggerTurnover", triggerCbT);

	

	
	//-------------------- important variables --------------------
	std::string name_postfix="";
	std::string reason;
	ros::Time currentTime, startTime, endTime,triggerTime, recordTime, stopTime, start2Time;
	rosbag::Bag bag;
	bool first_time=false;
  TopicPack tp;
  tp.subscribe(n);
  tp.setActive(DEFAULT,true);
  tp.setActive(RECOVERY,true);




	// -------------------- main loop --------------------

	std::cout<<"<*****\t\t\t Ready to get triggers \t\t\t*****>"<<std::endl<<std::endl;
	
	while(ros::ok())
	{
		
		currentTime = ros::Time(std::time(0) );

		// -------------------- reaction on triggers --------------------

		//if we get signal about ghosting (or rotation), we go into this clause
		if((triggeredG||triggeredT)&&stateCurr!=BAG_OFF)
		{
      if(stateCurr==BAG_NORMAL)
      {
        tp.setClearingPast(DEFAULT,false);
      }
      if(stateCurr==BAG_RECOVERY)
      {
        tp.setClearingPast(RECOVERY,false);
      }
			startTime = currentTime - ros::Duration(beforeTrigger);
			endTime = currentTime + ros::Duration(afterTrigger);

			if(!recording)
			{

				ROS_INFO("Recording triggered. Saving buffered messages (since %d) and continuing until %d",
				startTime.sec, endTime.sec);
				bag.open(outputDir+"/recorder_"+std::to_string(currentTime.sec)+".bag",rosbag::bagmode::Write);
				recording=true;
				
			}
			else
			{
				//time is extended before, and here only displayed info
				ROS_INFO("Ghosting again in the same time buffer!!!");
				ROS_INFO("Recording extended until %d", endTime.sec);
			
			}

			if(triggeredG)
				reason="_G";
			if(triggeredT)
				reason="_T";
			
			name_postfix+=reason;

			triggeredG=false;
			triggeredT=false;
			first_time=true;

		}

		// -------------------- name creation - tagging --------------------

		// stateCurr would be used in two places and could be changed between them.
		// For such a short period of time variable won't be blocked by mutex
		// but stored and used as a temporary variable, which can not be changed
		//the state won't be changed till next loop rotation
		
		stateCurr_T=stateCurr;

		if(first_time)
		{
			switch(stateCurr_T)
			{				
				case BAG_NORMAL:
				{
					
					name_postfix+="N";
					break;

				}

				case BAG_RECOVERY:
				{

					name_postfix+="R";
					break;

				}	
			}
			first_time=false;	

			//update of the mode(important in case it was done before trigger)
			statePrev=stateCurr_T;

		} 
		else if (!first_time && recording && stateCurr_T!=statePrev)
		{
			statePrev=stateCurr_T;

			//if mode changed, the name is updated. 

			name_postfix+=reason;

			switch(stateCurr_T)
			{
					
				case BAG_NORMAL:
				{

					name_postfix+="N";
					break;

				}

				case BAG_RECOVERY:
				{

					name_postfix+="R";
					break;

				}	
			}

		
		// -------------------- recording --------------------

		start2Time = ros::Time(std::time(0)); //helpful to calculate time of recording message per each topic

		ros::spinOnce();

      }

		currentTime = ros::Time(std::time(0));
		
		
		// -------------------- finishing recording --------------------


//     tp.dumpAnyTopicRecovery( bag,n);
		//if time has elapsed, recording is stopped and values of auxiliary variables restored to inital values, bag closed
		if((recording && (currentTime>endTime || stateCurr==BAG_OFF))){

			switch(stateCurr_T)
      {
        case BAG_NORMAL:
          {
            tp.dumpAnyTopic(DEFAULT, bag,n);//poping messages from program buffor to file KM
            break;
          }
          
        case BAG_RECOVERY:
          {
            tp.dumpAnyTopic( RECOVERY,bag,n);//poping messages from program buffor to file KM
            break;
          }
      }

      tp.setClearingPast(RECOVERY,true);
      tp.setClearingPast(DEFAULT,true);
			recording=false;
			//buffer unlock
			ROS_INFO("Recording finished");
			ROS_INFO("Time of recording messages: %f",
           // (stateCurr_T==DEFAULT?tp.defaultTopics:tp.recoveryTopics)
            currentTime.toSec()
            -startTime.toSec());
			std::cout<<std::endl<<std::endl;
			
			//rename file
			rename(bag.getFileName().c_str(), (bag.getFileName().erase(bag.getFileName().size()-4) + name_postfix+".bag").c_str());
      		bag.close();
		}

	ros::Duration(0.05).sleep();
	ros::spinOnce();

	}
}


