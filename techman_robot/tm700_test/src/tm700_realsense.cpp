/*********************************************************************
 * tm700_test_node.cpp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Yu-Hsien Chang, ISCI, NCTU
 */

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


#include "tm_msgs/SetIO.h"
//#include "tm_msgs/SetIORequest.h"
//#include "tm_msgs/SetIOResponse.h"

//ROS PCL Library
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//Bin Picking related Libraries
#include <CADDatabaseClass.h>
#include <PCD_Function.h>
#include <PCDProcessClass.h>
#include <VotingSchemePoseEstimation_Class.h>

using namespace std;

/*
 * Functions declaration
 */
void Bin_Picking(const sensor_msgs::PointCloud2Ptr& input);
void Manual_RecognitionFun(const sensor_msgs::PointCloud2Ptr& input);
void Auto_RecognitionFun(const sensor_msgs::PointCloud2Ptr& input);
void cloud_cb(const sensor_msgs::PointCloud2Ptr& input);
void get_current_joint_values(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              std::vector<double>& record_joint);
bool try_move_to_named_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::string& target_name,
                              unsigned int max_try_times);
bool try_move_to_joint_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::vector<double>& joint_target,
                              unsigned int max_try_times);


PositionData  PositionData_Main;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > determine_GraspICP_Cloud;
std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > determine_GraspObjectMat;
Eigen::Matrix< float, 4, 1 > Camera_ObjectGraspPoint;
int WhichOneBeGrasp = 0;

/*
 *   Object
 */
VotingSchemePoseEstimationClass PoseEstimationObj;
CADDatabaseClass CADDatabaseObj;

/*
 *  ------------ Socket Info -----------------
 *
 *  KukaState = 0 -> Wait KUKA arrive image capturing point
 *	KukaState = 1 -> Image capturing + Pose Estimation
 *  KukaState = 2 -> Wait KUKA complete components grasping等
 *  KukaState = -1 -> Close
 */

 int KukaState = 0;
 float Xyzabc_CommandData[6] = {0};
 float TCP_PositionData[6] = {244.4473, -114.9051, 386.892, -178.8034, -1.1524, 89.6715};
 int PositionOrder = 1;
 int cmp;

/*
 *   Global Variables
 */

int show_Mode = 0;
int CADModel_Number = 3;
float CADModel_Normal_radius = 7.5;//7.5;
float CADModel_Voxel_radius = 5.0;//5.0;//(1 = 1mm)
float Scene_Voxel_radius = 5.0; //6.0;
float Scene_Normal_radius = 7.5; //7.5;
float SACSegmentationFromNormal_radius = 12; //12;
float HashMapSearch_Position = 20.0; // No use
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
char const*AllCADModel_pcdFileName[3] = {"VirtualObject_1_CADModel_PCD.pcd", "VirtualObject_3_CADModel_PCD.pcd", "VirtualObject_6_CADModel_PCD.pcd"};
//char const*CADModel_pcdFileName[1] = {"VirtualObject_1_CADModel_PCD.pcd"};
char const* pcdFileName_scene[1] = {"scene.pcd"};
char const* pcdFileName_input[1] = {"input.pcd"};
int Grasp_ObjectType;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> RecognitionPCD_Viewer (new pcl::visualization::PCLVisualizer ("PCD_Viewer")); //RecognitionPCD_Viewer
float segmentation_Range[3][2] =
{
	{120, 385},
	{270, 440},
	{100, 800}
};
int CaptureImage_Again = 1;
int Times_Counter = 0;

ros::Publisher pub, pub_scene, pub_seg, pub_downsample, pub_estimation;
//Set data path
//std::string pcd_data_path = "/home/isci/Documents/tm5_ws/src/isci_ros_tm5/techman_robot/tm700_test/pcd/";
//std::string pcd_save_path = "/home/isci/Documents/tm5_ws/src/isci_ros_tm5/techman_robot/tm700_test/src/data/";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tm700_realsense");
    ros::NodeHandle node_handle, node_handle_file;
    sensor_msgs::PointCloud2 pcd_data;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //TCP_PositionData = {0}

    ROS_INFO("compute_VotingEstimation_OffinePhase Start");
  	compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
    ROS_INFO("compute_VotingEstimation_OffinePhase Finish");

    //Create ROS publisher
    pub_scene = node_handle.advertise<sensor_msgs::PointCloud2>("scene", 10);
    pub_seg = node_handle.advertise<sensor_msgs::PointCloud2>("scene_seg", 10);
    pub_downsample = node_handle.advertise<sensor_msgs::PointCloud2>("scene_downsample", 10);
    pub_estimation = node_handle.advertise<sensor_msgs::PointCloud2>("scene_estiamtion", 10);
    //Create a ROS subscriber for hte input point cloud from Realsense
    ROS_INFO("Subscribe /camera/depth/points");
    ros::Subscriber sub;
    sub = node_handle.subscribe("/camera/depth/points", 10, Auto_RecognitionFun);
    //ros::Subscriber sub = node_handle.subscribe("/camera/depth/points", 10, Auto_RecognitionFun);

/*
    ros::ServiceClient set_io_client = node_handle.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
    tm_msgs::SetIO io_srv;
    io_srv.request.fun = 2;//tm_msgs::SetIORequest::FUN_SET_EE_DIGITAL_OUT;
    io_srv.request.ch = 0;
    io_srv.request.value = 0.0;
*/
    // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    //ros::spin();

    ros::Rate r(10);
    while(ros::ok() || CaptureImage_Again == 1)
    {
      //ROS_INFO("In while");
      //sub = node_handle.subscribe("/camera/depth/points", 10, Auto_RecognitionFun);
      //cout << " CaptureImage_Again : (1)->Yes, 0->No" << endl;
      //cin >> CaptureImage_Again;
      ROS_INFO("CTRL + C to CLOSE the PROGRAM");
      //ROS_INFO("SPINONCE START");
      ros::spinOnce();
      //ROS_INFO("SPINONCE END");
      r.sleep();
    }

    //ros::waitForShutdown();
    cout << " CaptureImage_Again = "<< CaptureImage_Again << endl;
    //spinner.stop();
    cout << " spinner.stop "<< endl;
    //sub.shutdown();
    cout << " sub.shutdown "<< endl;
    ros::shutdown();
    cout << " ros::shutdown "<< endl;

/*
    sleep(1);

    // Setup
    // ^^^^^
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name
    // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroup group("manipulator");
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // (Optional) Create a publisher for visualizing plans in Rviz.
    //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    moveit::planning_interface::MoveGroup::Plan my_plan;

    set_io_client.call(io_srv);

    group.setPlanningTime(30.0);

    try_move_to_named_target(group, my_plan, "home", 100);

    std::vector<double> joint_target_1;
    std::vector<double> joint_target_2;

    joint_target_1.assign(6, 0.0f);
    joint_target_1[0] = 0.0174533 * ( 30.0);
    joint_target_1[1] = 0.0174533 * ( 15.0);
    joint_target_1[2] = 0.0174533 * (105.0);
    joint_target_1[3] = 0.0174533 * (-30.0);
    joint_target_1[4] = 0.0174533 * ( 90.0);
    joint_target_1[5] = 0.0174533 * ( 30.0);

    joint_target_2.assign(6, 0.0f);
    joint_target_2[0] = 0.0174533 * (-30.0);
    joint_target_2[1] = 0.0174533 * (-15.0);
    joint_target_2[2] = 0.0174533 * ( 90.0);
    joint_target_2[3] = 0.0174533 * (-75.0);
    joint_target_2[4] = 0.0174533 * (120.0);

    int step = 0;

    while (ros::ok())
    {
        switch (step)
        {
        case 0: //move to ready1
            ROS_INFO("move...to ready1");
            try_move_to_named_target(group, my_plan, "ready1", 100);
            break;
        case 1: //move to ready2
            ROS_INFO("move...to ready2");
            try_move_to_named_target(group, my_plan, "ready2", 100);
            break;
        case 2: //move to ready3
            ROS_INFO("move...tKinectObj.Sceno ready3");
            try_move_to_named_target(group, my_plan, "ready3", 100);
            break;
        case 3: //move to ready4
            ROS_INFO("move...to ready4");
            try_move_to_named_target(group, my_plan, "ready4", 100);
            break;

        case 4: //move 1
            ROS_INFO("move...to joint_target_1");
            try_move_to_joint_target(group, my_plan, joint_target_1, 100);
            break;

        case 5: //move 2...
            ROS_INFO("move...to joint_target_2");
            try_move_to_joint_target(group, my_plan, joint_target_2, 100);
            break;
        }
        step = (step + 1) % 6;
    }
*/
    return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2Ptr& input)
{
  //Create a containimpl_er for the data.
  sensor_msgs::PointCloud2 output;

  //Do data processing here
  output = *input;

  //Publish the data.
  ROS_INFO("Publish output");
  pub.publish(output);
}

void get_current_joint_values(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              std::vector<double>& record_joint
                              )
{
  /*
   * Get current joint values of TM5
   */
  //if(!ros::ok()) return false;
  bool success = false;

  std::vector<double> joint_value;
  joint_value = group.getCurrentJointValues();
  record_joint = group.getCurrentJointValues();

  ROS_INFO("In get_current_joint_values()");
  for(int i = 0; i<joint_value.size(); i++)
  {
    joint_value[i] = joint_value[i]*180/M_PI;
    printf("Joint %d: %lf\n",i+1, joint_value[i]);
  }
}

bool try_move_to_named_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::string& target_name,
                              unsigned int max_try_times = 1
                             )
{
    if (!ros::ok()) return false;
    bool success = false;

    for (unsigned int i = 0; i < max_try_times; i++)
    {

        group.setNamedTarget(target_name);

        if (group.move())
        {
            success = true;
            break;
        }
        else
        {
            if (!ros::ok()) break;
            sleep(1);
        }
    }
    return success;
}

bool try_move_to_joint_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::vector<double>& joint_target,
                              unsigned int max_try_times = 1
                             )
{
    if (!ros::ok()) return false;
    bool success = false;

    for (unsigned int i = 0; i < max_try_times; i++)
    {

        group.setJointValueTarget(joint_target);

        if (group.move())
        {
            success = true;
            break;
        }
        else
        {
            if (!ros::ok()) break;
            sleep(1);
        }
    }
}

void Bin_Picking(const sensor_msgs::PointCloud2Ptr& input)
{
  if(CaptureImage_Again == 1)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_seg_rviz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampling (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_segmentation (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_estimation (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_downsampling;
    sensor_msgs::PointCloud2 output_segmentation;
    sensor_msgs::PointCloud2 output_estimation;

    pcl::PointXYZ temp_point;

    ROS_INFO("======== TM5 State = 1 : Image Capturing and Pose Estimation ======== ");
    _IsPoseEstimationDone = false;
    int CaptureImage_Again = 1;
    //int Times_Counter = 0;

    while(!_IsPoseEstimationDone) //Capture Image and Pose Estimation
    {
      /*
  	   *   Image capturing
  	   */
      ROS_INFO("Publish topic: scene");
      output = *input;
      //pub_scene.publish(output);
      pcl::fromROSMsg(*input, *scene);

      for (size_t i = 0; i< scene->points.size(); i++)
  	  {
        /*
         * Convert unit: meter(m) -> millimeter(mm)
         */
         scene->points[i].x = 1000* scene->points[i].x;
         scene->points[i].y = 1000* scene->points[i].y;
         scene->points[i].z = 1000* scene->points[i].z;
  	  }
      //SavePCD(scene, pcdFileName_scene[0]);
      //SavePCD(input_cloud, pcdFileName_scene[0]);
      pcl::toROSMsg(*scene, output);
      pub_scene.publish(output);
      //voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
      voxelGrid_Filter(scene, scene_downsampling, Scene_Voxel_radius);
      pcl::toROSMsg(*scene_downsampling, output_downsampling);
      ROS_INFO("Publish topic: scene_downsample");
      pub_downsample.publish(output_downsampling);

      ROS_INFO("compute_SACSegmentationFromNormals Start");
      //compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 1);
      compute_SACSegmentationFromNormals( scene_downsampling, scene_segmentation, SACSegmentationFromNormal_radius, 1);
      ROS_INFO("compute_SACSegmentationFromNormals Finished");

      if(scene_segmentation->empty() || scene_segmentation->size() < 100)
      {
        Times_Counter++;
        if(Times_Counter < 3)
        {
          CaptureImage_Again = 1;
          break;
        }
        else
        {
          cout<<"System : Capture Image again...? ( Enter: 1 -> again, 0 -> end)" << endl;
          cin >> CaptureImage_Again;
          Times_Counter = 0;
          break;
        }
      }

      scene_seg_rviz->clear();
      *scene_seg_rviz = *scene_segmentation;

      for (size_t i = 0; i< scene_segmentation->points.size(); i++)
      {
        /*
         * Convert unit: millimeter(mm) -> meter(m)
         */
         scene_seg_rviz->points[i].x = 0.001* scene_seg_rviz->points[i].x;
         scene_seg_rviz->points[i].y = 0.001* scene_seg_rviz->points[i].y;
         scene_seg_rviz->points[i].z = 0.001* scene_seg_rviz->points[i].z;

         //cout << "temp_point.x = " << temp_point.x << endl;
         //cout << "temp_point.y = " << temp_point.y << endl;
         //cout << "temp_point.z = " << temp_point.z << endl;
         //scene_seg_rviz->push_back(temp_point);
      }

      pcl::toROSMsg(*scene_seg_rviz, output_segmentation);
      ROS_INFO("Publish topic: scene_segmentation");
      pub_seg.publish(output_segmentation);

    }
  }
  else
  {
    return;
  }
}

void Auto_RecognitionFun(const sensor_msgs::PointCloud2Ptr& input)
{
  /*****************************************************************************************
   * Steps:
   * (1) VoxelGrid Filter: voxelGrid_Filter -> Topic: scene_downsample
   * (2) SAC Segmentation: compute_SACSegmentationFromNormals -> Topic: scene_segmentation
   * (3) Pose Estimation: compute_VotingEstimation_OnlinePhase -> Topic: scene_estimation
   *****************************************************************************************
   */

  if(CaptureImage_Again == 1)
  {
    CaptureImage_Again = 0;
	  float background_color[3] = { 0, 0, 0 };
	  float point_color[3] = { 255, 255, 255 };
    //int CaptureImage_Again = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_seg_rviz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampling (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_segmentation (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_estimation (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_downsampling;
    sensor_msgs::PointCloud2 output_segmentation;
    sensor_msgs::PointCloud2 output_estimation;

    pcl::PointXYZ temp_point;

  /*
    ROS_INFO("compute_VotingEstimation_OffinePhase Start");
	  compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
    ROS_INFO("compute_VotingEstimation_OffinePhase Start");
  */
    ROS_INFO("==========================================");
    ROS_INFO("Ready for Auto_RecognitionFun");
    ROS_INFO("==========================================");
	  /*
	   *   Image capturing
	   */
    ROS_INFO("Publish topic: scene");
    output = *input;
    //pub_scene.publish(output);
    pcl::fromROSMsg(*input, *scene);

    for (size_t i = 0; i< scene->points.size(); i++)
	  {
      /*
       * Convert unit: meter(m) -> millimeter(mm)
       */
        scene->points[i].x = 1000* scene->points[i].x;
        scene->points[i].y = 1000* scene->points[i].y;
        scene->points[i].z = 1000* scene->points[i].z;
	  }

    //SavePCD(scene, pcdFileName_scene[0]);
    //SavePCD(input_cloud, pcdFileName_scene[0]);
    pcl::toROSMsg(*scene, output);
    pub_scene.publish(output);
	  //voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
    voxelGrid_Filter(scene, scene_downsampling, Scene_Voxel_radius);
    pcl::toROSMsg(*scene_downsampling, output_downsampling);
    ROS_INFO("Publish topic: scene_downsample");
    pub_downsample.publish(output_downsampling);

    ROS_INFO("compute_SACSegmentationFromNormals Start");
	  //compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 1);
    compute_SACSegmentationFromNormals( scene_downsampling, scene_segmentation, SACSegmentationFromNormal_radius, 1);
    ROS_INFO("compute_SACSegmentationFromNormals Finished");

    scene_seg_rviz->clear();
    *scene_seg_rviz = *scene_segmentation;

    for (size_t i = 0; i< scene_segmentation->points.size(); i++)
	  {
      /*
       * Convert unit: millimeter(mm) -> meter(m)
       */
      scene_seg_rviz->points[i].x = 0.001* scene_seg_rviz->points[i].x;
      scene_seg_rviz->points[i].y = 0.001* scene_seg_rviz->points[i].y;
      scene_seg_rviz->points[i].z = 0.001* scene_seg_rviz->points[i].z;

      //cout << "temp_point.x = " << temp_point.x << endl;
      //cout << "temp_point.y = " << temp_point.y << endl;
      //cout << "temp_point.z = " << temp_point.z << endl;
      //scene_seg_rviz->push_back(temp_point);
	  }

    pcl::toROSMsg(*scene_seg_rviz, output_segmentation);
    ROS_INFO("Publish topic: scene_segmentation");
    pub_seg.publish(output_segmentation);

	  /*
	   *   Pose Estimating
	   */
    ROS_INFO("compute_VotingEstimation_OnlinePhase Start");
    //compute_VotingEstimation_OnlinePhase( RecognitionPCD_Viewer, PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone);
	  compute_VotingEstimation_OnlinePhase(scene,  scene_segmentation, CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, ObjectPose_EulerAngle, Grasp_ObjectType, _IsPoseEstimationDone, scene_estimation);
    ROS_INFO("compute_VotingEstimation_OnlinePhase Finished");

    pcl::toROSMsg(*scene_estimation, output_estimation);
    ROS_INFO("Publish topic: scene_estimation");
    pub_estimation.publish(output_estimation);

    cout << " CaptureImage_Again : (1)->Yes, 0->No" << endl;
    cin >> CaptureImage_Again;
  }//end of if
}


void Manual_RecognitionFun(const sensor_msgs::PointCloud2Ptr& input)
{
  if(CaptureImage_Again == 1)
  {
    CaptureImage_Again = 0;
    float background_color[3] = { 0, 0, 0 };
  	float point_color[3] = { 255, 255, 255 };
  	float reference_point_color[3] = { 255, 0, 0 };
  	int CAD_Type = 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_seg_rviz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_downsampling (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_segmentation (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_estimation (new pcl::PointCloud<pcl::PointXYZ>);

    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_downsampling;
    sensor_msgs::PointCloud2 output_segmentation;
    sensor_msgs::PointCloud2 output_estimation;

    pcl::PointXYZ temp_point;

  /*
    ROS_INFO("compute_VotingEstimation_OffinePhase Start");
	  compute_VotingEstimation_OffinePhase( CADModel_Number, AllCADModel_pcdFileName, CADModel_Normal_radius, HashMapSearch_Position, HashMapSearch_Rotation);
    ROS_INFO("compute_VotingEstimation_OffinePhase Start");
  */
    ROS_INFO("==========================================");
    ROS_INFO("Ready for Manual_RecognitionFun");
    ROS_INFO("==========================================");
	  /*
	   *   Image capturing
	   */
    ROS_INFO("Publish topic: scene");
    output = *input;
    //pub_scene.publish(output);
    pcl::fromROSMsg(*input, *scene);

    for (size_t i = 0; i< scene->points.size(); i++)
	  {
      /*
       * Convert unit: meter(m) -> millimeter(mm)
       */
        scene->points[i].x = 1000* scene->points[i].x;
        scene->points[i].y = 1000* scene->points[i].y;
        scene->points[i].z = 1000* scene->points[i].z;
	  }

    //SavePCD(scene, pcdFileName_scene[0]);
    //SavePCD(input_cloud, pcdFileName_scene[0]);
    pcl::toROSMsg(*scene, output);
    pub_scene.publish(output);
	  //voxelGrid_Filter( PoseEstimationObj.getSceneCloud(), PoseEstimationObj.getDownsampling_SceneCloud(), Scene_Voxel_radius );
    voxelGrid_Filter(scene, scene_downsampling, Scene_Voxel_radius);
    pcl::toROSMsg(*scene_downsampling, output_downsampling);
    ROS_INFO("Publish topic: scene_downsample");
    pub_downsample.publish(output_downsampling);

    ROS_INFO("compute_SACSegmentationFromNormals Start");
	  //compute_SACSegmentationFromNormals( PoseEstimationObj.getDownsampling_SceneCloud(), PoseEstimationObj.getSceneSegmentationCloud(), SACSegmentationFromNormal_radius, 1);
    compute_SACSegmentationFromNormals( scene_downsampling, scene_segmentation, SACSegmentationFromNormal_radius, 1);
    ROS_INFO("compute_SACSegmentationFromNormals Finished");

    scene_seg_rviz->clear();
    *scene_seg_rviz = *scene_segmentation;

    for (size_t i = 0; i< scene_segmentation->points.size(); i++)
	  {
      /*
       * Convert unit: millimeter(mm) -> meter(m)
       */
      scene_seg_rviz->points[i].x = 0.001* scene_seg_rviz->points[i].x;
      scene_seg_rviz->points[i].y = 0.001* scene_seg_rviz->points[i].y;
      scene_seg_rviz->points[i].z = 0.001* scene_seg_rviz->points[i].z;

      //cout << "temp_point.x = " << temp_point.x << endl;
      //cout << "temp_point.y = " << temp_point.y << endl;
      //cout << "temp_point.z = " << temp_point.z << endl;
      //scene_seg_rviz->push_back(temp_point);
	  }

    pcl::toROSMsg(*scene_seg_rviz, output_segmentation);
    ROS_INFO("Publish topic: scene_segmentation");
    pub_seg.publish(output_segmentation);

	  /*
	   *   Pose Estimating and Veryify Precision
	   */
    ROS_INFO("compute_VotingEstimation_OnlinePhase_VeryfyPrecision Start");
    compute_VotingEstimation_OnlinePhase_VerifyPrecision(scene, scene_segmentation, CADDatabaseObj.getCADModel_OriginalPCDVector(), CADModel_Number, Scene_Normal_radius , Clustter_Position, Cluster_Rotation, SamplingRate, Arm_PickPoint, TCP_PositionData, _IsPoseEstimationDone, CAD_Type, CADDatabaseObj.getCADModelCloud());
    ROS_INFO("compute_VotingEstimation_OnlinePhase_VeryfyPrecision Finished");

    cout << " CaptureImage_Again : (1)->Yes, 0->No" << endl;
    cin >> CaptureImage_Again;
  }//end of if
}
