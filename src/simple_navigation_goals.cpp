#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Empty.h"
#include <sstream>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;

int main(int argc, char** argv){
   tf2::Quaternion myQuaternion;

   ros::init(argc, argv, "waypoint_server");
   ros::NodeHandle nh;
   std_msgs::Empty msg;
   ros::Publisher pub = nh.advertise<std_msgs::Empty>("toggle_led", 1);

   ros::Rate loop_rate(2);

   int num_waypoints;

   ifstream infile;
   infile.open("/home/eced3901/my_catkin_ws/src/eced3901_2021_team33/src/waypoints.txt");
   if(!infile){
      cout << "no file found" << endl;
   }
   else{
      infile >> num_waypoints;
      double x[num_waypoints];
      double y[num_waypoints];
      double theta[num_waypoints];

      for (int i = 0; i < num_waypoints; i++){

	 /* Try to read in the values */
         try{
	 	infile >> x[i] >> y[i] >> theta[i];
 	 }
	 catch (...) {
	 	ROS_INFO("An exception occured reading in the waypoints");
	 }
       
         
         cout << "x: " << x[i] << "y: " << y[i] << "theta: " << theta[i] << endl;
      }
      infile.close();

      /* Make sure waypoints are within final challenge boundaries */
      for (int i = 0; i < num_waypoints; i++){
      		if ( (x[i] > 4.2672 || x[i] < 0) || (y[i] > 4.2672 || x[i] < 0 ) ){
			ROS_INFO("Waypoints are out of bounds of the final challenge map");
			return 0;
		}
      }

   
      ROS_INFO("Started");

      ros::init(argc, argv, "simple_navigation_goals");

      //tell the action client that we want to spin a thread by default
      MoveBaseClient ac("move_base", true);

      //wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for the move_base action server to come up");
      }   

      move_base_msgs::MoveBaseGoal goal;

  
      goal.target_pose.header.frame_id="map";
      goal.target_pose.header.stamp = ros::Time::now();

      /* Publish the move goal */
      for (int i = 0; i < num_waypoints; i++){
	
	 /* Convert yaw to quaternion */
	 myQuaternion.setRPY(0, 0, theta[i]);
	 myQuaternion = myQuaternion.normalize();

	 /* Set target poses */
         goal.target_pose.pose.position.x = x[i];
         goal.target_pose.pose.position.y = y[i];
         goal.target_pose.pose.orientation.w = myQuaternion.w();

   
         ROS_INFO("Sending goal");
         ac.sendGoal(goal);
   
         ac.waitForResult();
   
         if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

	    /* Blink LED on arduino when waypoint is reached */
	    for (int i = 0; i < 6; i++){
		pub.publish(msg);
	    	ros::spinOnce();
	    	loop_rate.sleep();
	        ros::Duration(1).sleep();
	    }

            ROS_INFO("Horray, the base moved successfuly");
	    
         }   
         else{   
            ROS_INFO("The base failed to move for some reason");
         }
      }
   }   
   
   return 0;
}
