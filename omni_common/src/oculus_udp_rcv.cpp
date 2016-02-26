#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>



void error( char *msg)
{
 perror(msg);
 exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    //Startup ros
    ros::init(argc, argv, "oculus_udp_rcv");
    ros::AsyncSpinner spinner(1);
    spinner.start();
   
    // Load grasp data specific to our robot
    ros::NodeHandle nh("~");
   
  ros::Publisher oculus_publisher = nh.advertise<geometry_msgs::PoseStamped>("/oculus/pose", 1);
       
  int sockfd;
  sockfd = socket(AF_INET,SOCK_DGRAM,0);
  struct sockaddr_in serv,client;
 
  serv.sin_family = AF_INET;
  serv.sin_port = htons(53000);
  serv.sin_addr.s_addr = inet_addr("10.1.8.120");

  char buffer[2048];
  socklen_t l = sizeof(client);
  //socklen_t m = client;
 
  //bind socket to port
  bind(sockfd, (struct sockaddr*)&serv, sizeof(serv) );
 
  while(ros::ok())
  {
	ROS_INFO("Bucle");
    int rc= recvfrom(sockfd,buffer,sizeof(buffer),0,(struct sockaddr *)&client,&l);
  
	ROS_INFO("RECIBO");
    std::vector<std::string> strs;
    boost::split(strs, buffer, boost::is_any_of("_"));
  
    double qx = atof(strs[0].c_str());
    double qy = atof(strs[1].c_str());
    double qz = atof(strs[2].c_str());
    double qw = atof(strs[3].c_str());
   
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.x = qx;
    pose_msg.pose.orientation.y = qy;
    pose_msg.pose.orientation.z = qz;
    pose_msg.pose.orientation.w = qw;
    oculus_publisher.publish(pose_msg);

  }
}
