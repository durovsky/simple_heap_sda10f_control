/*********************************************************************************************//**
* @file simple_heap_lib.h
* 
* class Robot handles communication using basic robot driver without MoveIt library
*
* Copyright (c)
* Frantisek Durovsky 
* Department of Robotics
* Technical University Kosice 
* February 2015
*   
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/
#ifndef SIMPLE_HEAP_LIB_H
#define SIMPLE_HEAP_LIB_H

//Standard ROS headers
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//Headers for direct Robot driver access
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//Headers for MoveIT library
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//Custom service headers
#include <bin_picking_region_grow/bin_picking_region_grow_service.h>
#include <sub20_my_pkg/sub20_DO.h>

////////////////////////////////////////////////////////////////////////////////////////////

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class Robot
{
public:

     Robot(ros::NodeHandle *nh);
     ~Robot();
          
     void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
     control_msgs::FollowJointTrajectoryGoal Trajectory_init();
     control_msgs::FollowJointTrajectoryGoal Trajectory_zero_arm();
     control_msgs::FollowJointTrajectoryGoal Trajectory_start();
     control_msgs::FollowJointTrajectoryGoal Trajectory_torso_60();
          
     actionlib::SimpleClientGoalState getState();

     void r1_Callback(const sensor_msgs::JointState::ConstPtr &msg);
     void r2_Callback(const sensor_msgs::JointState::ConstPtr &msg);
     void b1_Callback(const sensor_msgs::JointState::ConstPtr &msg);
     void b2_Callback(const sensor_msgs::JointState::ConstPtr &msg);
       
     bool getFlag(bool &flag_name, bool value);
     void setFlag(bool &flag_name);

     //Left arm waypoints
     geometry_msgs::Pose bin_approach_torso_90;
     geometry_msgs::Pose bin_release_torso_90;
     geometry_msgs::Pose bin_dump_torso_90;
     geometry_msgs::Pose end_pose_torso_90;

     geometry_msgs::Pose bin_approach_torso_60;
     geometry_msgs::Pose bin_release_torso_60;
     geometry_msgs::Pose bin_dump_torso_60;
     geometry_msgs::Pose end_pose_torso_60;

     geometry_msgs::Pose object_approach;
     geometry_msgs::Pose object_grasp;

     //Right arm waypoints
     geometry_msgs::Pose approach_to_table;
     geometry_msgs::Pose place_bin_on_table;
     geometry_msgs::Pose release_contact;
     geometry_msgs::Pose move_away_from_bin;
     geometry_msgs::Pose back_to_initial_pose;

     const double no_object_to_grasp;

private:

     TrajClient* traj_client_;

     //"Wait for message" flags for joint_states Callbacks
     bool r1_recieved;
     bool r2_recieved;
     bool b1_recieved;
     bool b2_recieved;

     //Joint State variables for current robot state
     sensor_msgs::JointState r1;
     sensor_msgs::JointState r2;
     sensor_msgs::JointState b1;
     sensor_msgs::JointState b2;




};  
#endif  //SIMPLE_HEAP_LIB_H
