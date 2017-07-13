#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <ros/ros.h>
#include <ros/console.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "PCD_Function.h"
#include "VotingSchemePoseEstimation_Class.h"
#include "CADDatabaseClass.h"
#include "PCL_KinectClass.h"

#include "tm_msgs/SetIO.h"
/*Socket for KUKA
#include "MySocket.h"
*/

/*The libraries in Windows
#include <Windows.h>
#include <process.h>
*/

using namespace std;

//WgSocket MySocket;
VotingSchemePoseEstimationClass PoseEstimationObj;
CADDatabaseClass CADDatabaseObj;
KinectClass KinectObj;

 void Auto_RecognitionFun();
 void Manual_RecognitionFun();
 void decode_TCPPosition(char *encode_TCP);
 void Manual_Fun();
 void PrintPosition();

 int tm5_state = 0;
 float Xyzabc_CommandData[6] = {0};
 float TCP_PositionData[6] = {0};

 int show_Mode = 0;
 int CADModel_Number = 3;
 float CADModel_Normal_radius = 7.5;
 float CADModel_Voxel_radius = 5.0;//(1 = 1mm)
 float Scene_Voxel_radius = 6.0;
 float Scene_Normal_radius = 7.5;
 float SACSegmentationFromNormal_radius = 12;
 float HashMapSearch_Position = 20.0;// No use
 float HashMapSearch_Rotation = 15.0;
 float Clustter_Position = 3.5;
 float Cluster_Rotation = 30.0;
 float SamplingRate = 20;
 int showPose_num = 0;
 int DivideObject_ClusterNumber = 0;
 pcl::PointXYZ Arm_PickPoint;
 float ObjectPose_EulerAngle[3];
 bool _IsPoseEstimationDone = true;
 std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > DivideObject_ClusterPCDResult;
 char *AllCADModel_pcdFileName[3] = {"VirtualObject_1_CADModel_PCD.pcd", "VirtualObject_3_CADModel_PCD.pcd", "VirtualObject_6_CADModel_PCD.pcd"};
 char *CADModel_pcdFileName[1] = {"VirtualObject_1_CADModel_PCD.pcd"};
 int Grasp_ObjectType;
 boost::shared_ptr<pcl::visualization::PCLVisualizer> RecognitionPCD_Viewer (new pcl::visualization::PCLVisualizer("RecognitionPCD_Viewer"));
 float segmentation_Range[3][2] =
 {
   {120, 385},
   {270, 440},
   {100, 800}
 };


 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "tm700_bin_picking");
   ros::NodeHandel node_handle;
   ros::ServiceClient set_io_client = node_handle.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
   tm_msgs::SetIO io_srv;
   io_srv.request.fun = 2;
   io_srv.request.ch = 0;
   io_srv.request.value = 0.0;

   // start a background "spinner", so our node can process ROS messages
   //- this lets us know when the move is completed
   ros::AsyncSpinner spinner(1);
   spinner.start();

   sleep(1);
   
   KinectObj.KinectInitial();

   StartSocket();

   return 0;
 }

/*
 bool CreateClient(WgSocket &clientSocket)
 {
   if(clientSocket.Open("100.100.100.1", 6008))
   {
     cout <<"Correct : Create a virtual client and connect to server correct!!\n";
     return true;
   }
   else
   {
     return false;
   }
 }
*/
 void StartSocket()
 {
   bool check_OK;
   //Create a ClientSocket
   //check_OK = CreateClient(MySocket);
   check_OK = true;

   if(check_OK)
   {
     char Sent_ClientData[1024] = {0};
     char Recieve_ServerData[1024] = {0};
     long ret_len;

     compute_VotingEstimation_OffinPhase(CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);

   }
 }
