/*********************************************************************************************//**
* @file simple_heap_lib.cpp
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
* 
* *********************************************************************************************/
#ifndef SIMPLE_HEAP_LIB_CPP
#define SIMPLE_HEAP_LIB_CPP

////////////////////////////////////////////////////////////////////////////////////////////////

#include <simple_heap_lib.h>

//initialize the action client and wait for action server to come up
Robot::Robot(ros::NodeHandle nh) :
    r1_recieved(false),
    r2_recieved(false),
    b1_recieved(false),
    b2_recieved(false),
    no_object_to_grasp(100)
{
    //tell the action client that we want to spon thread by default
    traj_client_ = new TrajClient("/joint_trajectory_action", true);    

    //wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0)))
    {
       ROS_INFO("Waiting for the trajectory_action server");
    }

    fill_waypoint_values();
    ROS_INFO_STREAM("Joint trajectory action client initialized");
}

//clean up the action client
Robot::~Robot()
{
    delete traj_client_;
}

//Save current joint state values and set up the initial flag
void
Robot::r1_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   r1.position = msg->position;
   if(r1_recieved == false) r1_recieved = true;
}

void
Robot::r2_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   r2.position = msg->position;
   if(r2_recieved == false) r2_recieved = true;
}

void
Robot::b1_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   b1.position = msg->position;
   if (b1_recieved == false) b1_recieved = true;
}

void
Robot::b2_Callback(const sensor_msgs::JointState::ConstPtr &msg)
{
   b2.position = msg->position;
   if(b2_recieved == false) b2_recieved = true;
}

//Sends the command to start a given trajectory
void
Robot::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
   goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(2.0);
   traj_client_->sendGoal(goal);
}

/////////////////////////////////////////////////////////////////////////

//Trajecotry_init  - initial robot movement to power up servo controller
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_init()
{
    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Wait for current robot state messages to come up
    while((r1_recieved == false) || (r2_recieved == false) || (b1_recieved == false) || (b2_recieved == false))
    {
        ros::Duration(0.1).sleep();
    }

    //Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(6);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0];
    goal.trajectory.points[ind].positions[15] = b2.position[0];

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);


    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0] + 0.05;
    goal.trajectory.points[ind].positions[15] = b2.position[0] + 0.05;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    //return the goal
    return goal;

}

/////////////////////////////////////////////////////////////////////////

//Trajecotry zero_arm
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_zero_arm()
{
    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Wait for current robot state messages to come up
    while((r1_recieved == false) || (r2_recieved == false) || (b1_recieved == false) || (b2_recieved == false))
    {
        ros::Duration(0.1).sleep();
    }

//Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(6);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0];
    goal.trajectory.points[ind].positions[15] = b2.position[0];

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);


    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = 0;
    goal.trajectory.points[ind].positions[1]  = 0;
    goal.trajectory.points[ind].positions[2]  = 0;
    goal.trajectory.points[ind].positions[3]  = 0;
    goal.trajectory.points[ind].positions[4]  = 0;
    goal.trajectory.points[ind].positions[5]  = 0;
    goal.trajectory.points[ind].positions[6]  = 0;
    goal.trajectory.points[ind].positions[7]  = 0;
    goal.trajectory.points[ind].positions[8]  = 0;
    goal.trajectory.points[ind].positions[9]  = 0;
    goal.trajectory.points[ind].positions[10] = 0;
    goal.trajectory.points[ind].positions[11] = 0;
    goal.trajectory.points[ind].positions[12] = 0;
    goal.trajectory.points[ind].positions[13] = 0;
    goal.trajectory.points[ind].positions[14] = 1.57;
    goal.trajectory.points[ind].positions[15] = 1.57;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);

    //return the goal
    return goal;
}

/////////////////////////////////////////////////////////////////////////

//Trajecotry start
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_start()
{
    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Wait for current robot state messages to come up
    while((r1_recieved == false) || (r2_recieved == false) || (b1_recieved == false) || (b2_recieved == false))
    {
        ros::Duration(0.1).sleep();
    }

//Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(6);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0];
    goal.trajectory.points[ind].positions[15] = b2.position[0];

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);


    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.8112;
    goal.trajectory.points[ind].positions[2] = -0.0331;
    goal.trajectory.points[ind].positions[3] = -2.2273;
    goal.trajectory.points[ind].positions[4] = 1.57;
    goal.trajectory.points[ind].positions[5] = 1.57;
    goal.trajectory.points[ind].positions[6] = 0.0;
    goal.trajectory.points[ind].positions[7] = 0.0;
    goal.trajectory.points[ind].positions[8] = -0.555;
    goal.trajectory.points[ind].positions[9] = 0.0;
    goal.trajectory.points[ind].positions[10] = -2.0947;
    goal.trajectory.points[ind].positions[11] = 0.0;
    goal.trajectory.points[ind].positions[12] = -1.1954;
    goal.trajectory.points[ind].positions[13] = 0.0;
    goal.trajectory.points[ind].positions[14] = 1.57;
    goal.trajectory.points[ind].positions[15] = 1.57;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    //return the goal
    return goal;

}

/////////////////////////////////////////////////////////////////////////

//Trajecotry torso 60
control_msgs::FollowJointTrajectoryGoal
Robot::Trajectory_torso_60()
{
    //goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    //Wait for current robot state messages to come up
    while((r1_recieved == false) || (r2_recieved == false) || (b1_recieved == false) || (b2_recieved == false))
    {
        ros::Duration(0.1).sleep();
    }

    //Define robot joint names
    goal.trajectory.joint_names.push_back("arm_left_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_left_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_left_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_left_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_left_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_left_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_left_joint_7_t");

    goal.trajectory.joint_names.push_back("arm_right_joint_1_s");
    goal.trajectory.joint_names.push_back("arm_right_joint_2_l");
    goal.trajectory.joint_names.push_back("arm_right_joint_3_e");
    goal.trajectory.joint_names.push_back("arm_right_joint_4_u");
    goal.trajectory.joint_names.push_back("arm_right_joint_5_r");
    goal.trajectory.joint_names.push_back("arm_right_joint_6_b");
    goal.trajectory.joint_names.push_back("arm_right_joint_7_t");

    goal.trajectory.joint_names.push_back("torso_joint_b1");
    goal.trajectory.joint_names.push_back("torso_joint_b2");

    //Trajectory points
    goal.trajectory.points.resize(6);

    //===========================================================
    //First trajectory point
    //===========================================================

    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = b1.position[0];
    goal.trajectory.points[ind].positions[15] = b2.position[0];

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(0);


    //=====================================================
    // Second trajectory point
    //=====================================================
    // Positions
    ind = 1;
    goal.trajectory.points[ind].positions.resize(16);
    goal.trajectory.points[ind].positions[0]  = r1.position[0];
    goal.trajectory.points[ind].positions[1]  = r1.position[1];
    goal.trajectory.points[ind].positions[2]  = r1.position[2];
    goal.trajectory.points[ind].positions[3]  = r1.position[3];
    goal.trajectory.points[ind].positions[4]  = r1.position[4];
    goal.trajectory.points[ind].positions[5]  = r1.position[5];
    goal.trajectory.points[ind].positions[6]  = r1.position[6];
    goal.trajectory.points[ind].positions[7]  = r2.position[0];
    goal.trajectory.points[ind].positions[8]  = r2.position[1];
    goal.trajectory.points[ind].positions[9]  = r2.position[2];
    goal.trajectory.points[ind].positions[10] = r2.position[3];
    goal.trajectory.points[ind].positions[11] = r2.position[4];
    goal.trajectory.points[ind].positions[12] = r2.position[5];
    goal.trajectory.points[ind].positions[13] = r2.position[6];
    goal.trajectory.points[ind].positions[14] = 1.04;
    goal.trajectory.points[ind].positions[15] = 1.04;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].velocities[j] = 0.0;

    //Accelerations
    goal.trajectory.points[ind].accelerations.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].accelerations[j] = 0.0;

    //Effort
    goal.trajectory.points[ind].effort.resize(16);
    for (size_t j = 0; j < 16; ++j) goal.trajectory.points[ind].effort[j] = 0.0;

    //To be reached 10 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    //return the goal
    return goal;

}

/////////////////////////////////////////////////////////////////////////

//Returns the current state of the action
actionlib::SimpleClientGoalState 
Robot::getState()
    {
        return traj_client_->getState();
    }

/////////////////////////////////////////////////////////////////////////

void
Robot::fill_waypoint_values()
{
    //Left Arm waypoints
    ///////////////////////////////////////////
    //Left 90deg torso orientation
    bin_approach_torso_90.position.x = -0.0116;
    bin_approach_torso_90.position.y = 0.5860;
    bin_approach_torso_90.position.z = 1.21;

    bin_approach_torso_90.orientation.x = 0;
    bin_approach_torso_90.orientation.y = 0;
    bin_approach_torso_90.orientation.z = 0;
    bin_approach_torso_90.orientation.w = 1;

    //------------------------------------------------

    bin_release_torso_90.position.x = 0.25;
    bin_release_torso_90.position.y = 0.40;
    bin_release_torso_90.position.z = 1.25;

    bin_release_torso_90.orientation.x = 0;
    bin_release_torso_90.orientation.y = 0;
    bin_release_torso_90.orientation.z = 0;
    bin_release_torso_90.orientation.w = 1;
    //------------------------------------------------

    bin_dump_torso_90.position.x = 0.25;
    bin_dump_torso_90.position.y = 0.40;
    bin_dump_torso_90.position.z = 1.21;

    bin_dump_torso_90.orientation.x = 0;
    bin_dump_torso_90.orientation.y = 0;
    bin_dump_torso_90.orientation.z = 0;
    bin_dump_torso_90.orientation.w = 1;
    //--------------------------------------------------

    end_pose_torso_90.position.x = -0.0116;
    end_pose_torso_90.position.y = 0.4;
    end_pose_torso_90.position.z = 1.21;

    end_pose_torso_90.orientation.x = 0;
    end_pose_torso_90.orientation.y = 0;
    end_pose_torso_90.orientation.z = 0;
    end_pose_torso_90.orientation.w = 1;
    //---------------------------------------------------

    //Left 60deg torso orientation
    bin_approach_torso_60.position.x = 0.2950;
    bin_approach_torso_60.position.y = 0.5597;
    bin_approach_torso_60.position.z = 1.21;

    bin_approach_torso_60.orientation.x = 0;
    bin_approach_torso_60.orientation.y = 0;
    bin_approach_torso_60.orientation.z = 0;
    bin_approach_torso_60.orientation.w = 1;

    //------------------------------------------------

    bin_release_torso_60.position.x = 0.42;
    bin_release_torso_60.position.y = 0.3;
    bin_release_torso_60.position.z = 1.25;

    bin_release_torso_60.orientation.x = 0;
    bin_release_torso_60.orientation.y = 0;
    bin_release_torso_60.orientation.z = 0;
    bin_release_torso_60.orientation.w = 1;
    //------------------------------------------------

    bin_dump_torso_60.position.x = 0.42;
    bin_dump_torso_60.position.y = 0.3;
    bin_dump_torso_60.position.z = 1.21;

    bin_dump_torso_60.orientation.x = 0;
    bin_dump_torso_60.orientation.y = 0;
    bin_dump_torso_60.orientation.z = 0;
    bin_dump_torso_60.orientation.w = 1;
    //--------------------------------------------------

    end_pose_torso_60.position.x = 0.20;
    end_pose_torso_60.position.y = 0.4;
    end_pose_torso_60.position.z = 1.21;

    end_pose_torso_60.orientation.x = 0;
    end_pose_torso_60.orientation.y = 0;
    end_pose_torso_60.orientation.z = 0;
    end_pose_torso_60.orientation.w = 1;
    //---------------------------------------------------

    //Right Arm waypoints
    //////////////////////////////////////////////
    approach_to_table.position.x = 0.4082;
    approach_to_table.position.y = 0.4081;
    approach_to_table.position.z = 1.1271;

    approach_to_table.orientation.x = 0.478716;
    approach_to_table.orientation.y = 0.520033;
    approach_to_table.orientation.z = 0.514818;
    approach_to_table.orientation.w = 0.485139;

    place_bin_on_table.position.x = 0.4082;
    place_bin_on_table.position.y = 0.4081;
    place_bin_on_table.position.z = 0.95;

    place_bin_on_table.orientation.x = 0.478716;
    place_bin_on_table.orientation.y = 0.520033;
    place_bin_on_table.orientation.z = 0.514818;
    place_bin_on_table.orientation.w = 0.485139;

    release_contact.position.x = 0.4082;
    release_contact.position.y = 0.4081;
    release_contact.position.z = 0.88;

    release_contact.orientation.x = 0.478716;
    release_contact.orientation.y = 0.520033;
    release_contact.orientation.z = 0.514818;
    release_contact.orientation.w = 0.485139;

    move_away_from_bin.position.x = 0.50;
    move_away_from_bin.position.y = 0.4081;
    move_away_from_bin.position.z = 0.88;

    move_away_from_bin.orientation.x = 0.478716;
    move_away_from_bin.orientation.y = 0.520033;
    move_away_from_bin.orientation.z = 0.514818;
    move_away_from_bin.orientation.w = 0.485139;

    back_to_initial_pose.position.x = 0.318045;
    back_to_initial_pose.position.y = 0.585989;
    back_to_initial_pose.position.z = 1.21752;

    back_to_initial_pose.orientation.x = 0.478716;
    back_to_initial_pose.orientation.y = 0.520033;
    back_to_initial_pose.orientation.z = 0.514818;
    back_to_initial_pose.orientation.w = 0.485139;

}

#endif  //SIMPLE_HEAP_LIB_CPP
