/*********************************************************************************************//**
* @file simple_heap.cpp
*
* Main simple_heap node
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

#include <ros/ros.h>
#include <simple_heap_lib.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_heap");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");

    //Initialize perception node
    ros::service::waitForService("bin_picking_region_grow/bin_picking_region_grow_service",10);
    ros::ServiceClient object_pose_client = nh.serviceClient<bin_picking_region_grow::bin_picking_region_grow_service>("/object_pose",1);
    std::cout << "Object pose client initialized" << std::endl;

    //Initialize IO node
    ros::service::waitForService("sub20/controll", 20);
    ros::ServiceClient sub20_client = nh.serviceClient<sub20_my_pkg::sub20_DO>("sub20/controll");
    std::cout << "Sub20 client initialized!" << std::endl;

    //Create objects for manipulation with robot
    Robot sda10f(nh);
    moveit::planning_interface::MoveGroup group_left_arm("arm_left");
    moveit::planning_interface::MoveGroup group_right_arm("arm_right");

    //Joint state subscribers
    ros::Subscriber r1_sub = nh.subscribe("/sda10f/sda10f_r1_controller/joint_states",10, &Robot::r1_Callback, &sda10f);
    ros::Subscriber r2_sub = nh.subscribe("/sda10f/sda10f_r2_controller/joint_states",10, &Robot::r2_Callback, &sda10f);
    ros::Subscriber b1_sub = nh.subscribe("/sda10f/sda10f_b1_controller/joint_states",10, &Robot::b1_Callback, &sda10f);
    ros::Subscriber b2_sub = nh.subscribe("/sda10f/sda10f_b2_controller/joint_states",10, &Robot::b2_Callback, &sda10f);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::cout << "Simple heap main node initialized!" << std::endl;

    //=========================================
    //Read and set parameters from launch file
    //=========================================
    double planningTime;
    double orientationTolerance;
    double positionTolerance;
    int numPlanningAttempts;

    //Read parameters from launch file
    priv_nh_.getParam("planning_time", planningTime);
    priv_nh_.getParam("orientation_tolerance", orientationTolerance);
    priv_nh_.getParam("position_tolerance", positionTolerance);
    priv_nh_.getParam("planning_attempts", numPlanningAttempts);

    group_left_arm.setPlanningTime(planningTime);
    group_right_arm.setPlanningTime(planningTime);

    group_left_arm.setGoalOrientationTolerance(orientationTolerance);
    group_right_arm.setGoalOrientationTolerance(orientationTolerance);

    group_left_arm.setGoalPositionTolerance(positionTolerance);
    group_right_arm.setGoalPositionTolerance(positionTolerance);

    group_left_arm.setNumPlanningAttempts(numPlanningAttempts);
    group_right_arm.setNumPlanningAttempts(numPlanningAttempts);

    std::string arm_left_effector;
    arm_left_effector = group_left_arm.getEndEffectorLink();

    //==========================================================
    //Robot Manipulation
    //==========================================================
    moveit::planning_interface::MoveGroup::Plan arm_left_plan;
    moveit::planning_interface::MoveGroup::Plan arm_right_plan;

    std::cout << "Press enter to power up servo controller: ";
    std::cin.get();

    //First small torso movement to power on servo controller
    sda10f.startTrajectory(sda10f.Trajectory_init());

    //Move robot to zero_arm position
    std::cout << "Press enter to move the robot to scan position: ";
    std::cin.get();
    sda10f.startTrajectory(sda10f.Trajectory_zero_arm());
    ros::Duration(10).sleep();

     while(ros::ok())
     {
         std::cout << "Press enter to pick an object: ";
         std::cin.get();

         //Basic robot position
         sda10f.startTrajectory(sda10f.Trajectory_start());
         ros::Duration(3).sleep();

         //====================================================
         //Get object positions from pointcloud
         //====================================================
         int number_of_remaining_objects;
         geometry_msgs::Pose current_object_pose;
         geometry_msgs::Vector3 current_object_normal;

         bin_picking_region_grow::bin_picking_region_grow_service object_pose_service;
         object_pose_service.request.coordinates_demand = true;
         if(object_pose_client.call(object_pose_service))
         {
            std::cout << "Object pose resposne recieved!" << std::endl;
            try
            {
                current_object_pose.position.x = object_pose_service.response.x_pose;
                current_object_pose.position.y = object_pose_service.response.y_pose;
                current_object_pose.position.z = object_pose_service.response.z_pose;

                current_object_pose.orientation.x = object_pose_service.response.x_orientation;
                current_object_pose.orientation.y = object_pose_service.response.y_orientation;
                current_object_pose.orientation.z = object_pose_service.response.z_orientation;
                current_object_pose.orientation.w = object_pose_service.response.w_orientation;

                current_object_normal.x = object_pose_service.response.nx;
                current_object_normal.y = object_pose_service.response.ny;
                current_object_normal.z = object_pose_service.response.nz;

                number_of_remaining_objects = object_pose_service.response.remaining_objects;
            }
            catch(int e)
            {
                std::cout << "An exception nr. " << e << " occured" << std::endl;
            }
          }
         else
         {
             ROS_ERROR("Failed to call service object_poses!");
             return 1;
         }

         //==================================================
         //Non-Static waypoints definitions
         //==================================================

         sda10f.object_approach.position.x = current_object_pose.position.x + 0.1 * current_object_normal.x;
         sda10f.object_approach.position.y = current_object_pose.position.y + 0.1 * current_object_normal.y;
         sda10f.object_approach.position.z = current_object_pose.position.z + 0.1 * current_object_normal.z;

         sda10f.object_approach.orientation.x = current_object_pose.orientation.x;
         sda10f.object_approach.orientation.y = current_object_pose.orientation.y;
         sda10f.object_approach.orientation.z = current_object_pose.orientation.z;
         sda10f.object_approach.orientation.w = current_object_pose.orientation.w;
         //---------------------------------------------------

         sda10f.object_grasp.position.x = current_object_pose.position.x;
         sda10f.object_grasp.position.y = current_object_pose.position.y;
         sda10f.object_grasp.position.z = current_object_pose.position.z + 0.02;

         sda10f.object_grasp.orientation.x = current_object_pose.orientation.x;
         sda10f.object_grasp.orientation.y = current_object_pose.orientation.y;
         sda10f.object_grasp.orientation.z = current_object_pose.orientation.z;
         sda10f.object_grasp.orientation.w = current_object_pose.orientation.w;
         //-----------------------------------------------------

         //==============================================================
         //Turn torso to reach object with "x" coordinate more than 0
         //==============================================================

         if((current_object_pose.position.x > 0) && (current_object_pose.position.x < 1))
         {
            sda10f.startTrajectory(sda10f.Trajectory_torso_60());
            ros::Duration(3).sleep();
         }
         //============================================================
         //Trajectory composition
         //============================================================
         std::vector<geometry_msgs::Pose> arm_left_cartesian_waypoints;
         moveit_msgs::RobotTrajectory arm_left_trajectory;

         if(current_object_pose.position.x != sda10f.no_object_to_grasp)
         {
            if(current_object_pose.position.x > 0)
            {
               arm_left_cartesian_waypoints.push_back(sda10f.object_approach);
               arm_left_cartesian_waypoints.push_back(sda10f.object_grasp);
               arm_left_cartesian_waypoints.push_back(sda10f.object_approach);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_approach_torso_60);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_release_torso_60);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_dump_torso_60);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_release_torso_60);
               arm_left_cartesian_waypoints.push_back(sda10f.end_pose_torso_60);
             }
             else
             {
               arm_left_cartesian_waypoints.push_back(sda10f.object_approach);
               arm_left_cartesian_waypoints.push_back(sda10f.object_grasp);
               arm_left_cartesian_waypoints.push_back(sda10f.object_approach);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_approach_torso_90);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_release_torso_90);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_dump_torso_90);
               arm_left_cartesian_waypoints.push_back(sda10f.bin_release_torso_90);
               arm_left_cartesian_waypoints.push_back(sda10f.end_pose_torso_90);
             }

             //Several attempts of cartesian trajectory computation
             double arm_left_fraction = 0;
             const int max_cartesian_attempts = 20;
             int acutal_number_of_cartesian_attempts = 1;
             while ((arm_left_fraction != 1) && (acutal_number_of_cartesian_attempts < max_cartesian_attempts))
             {
                arm_left_fraction = group_left_arm.computeCartesianPath(arm_left_cartesian_waypoints, 0.02, 0, arm_left_trajectory, false);
                ROS_INFO("Cartesian computation n. %i (cartesian path) (%.2f%% acheived)",acutal_number_of_cartesian_attempts, arm_left_fraction * 100.0);
                acutal_number_of_cartesian_attempts++;
             }


             //==========================================================
             //Robot movement
             //==========================================================
             if (arm_left_fraction == 1)
             {
                //Adding velocities to cartesian trajectory
                robot_trajectory::RobotTrajectory arm_left_rt_01(group_left_arm.getCurrentState()->getRobotModel(),"arm_left");
                arm_left_rt_01.setRobotTrajectoryMsg(*group_left_arm.getCurrentState(), arm_left_trajectory);
                trajectory_processing::IterativeParabolicTimeParameterization arm_left_iptp;
                //Compute Time Stamps
                const double max_velocity_scaling_factor = 1.0;
                bool arm_left_success = arm_left_iptp.computeTimeStamps(arm_left_rt_01, max_velocity_scaling_factor);
                ROS_INFO("Computed time stamps: %s", arm_left_success ? "SUCCEDED" : "FAILED");

                //Get robot trajectory from RobotTrajectory
                arm_left_rt_01.getRobotTrajectoryMsg(arm_left_trajectory);
                arm_left_plan.trajectory_ = arm_left_trajectory;
                group_left_arm.asyncExecute(arm_left_plan);
             }

             //Suction handler using sub20 board
             geometry_msgs::PoseStamped actual_pose;
             sub20_my_pkg::sub20_DO sub20_srv;

             bool suction_flag = false;
             int loop_limit = 1000;
             float suction_on_thresh,
                   suction_off_y_thresh,
                   suction_off_z_thresh;

             suction_on_thresh = sda10f.object_grasp.position.z + 0.01;
             suction_off_z_thresh = 1.23;

             //Different thresholds for torso_60 and torso_90 bin_dump
             if((current_object_pose.position.x > 0) && (current_object_pose.position.x < 1)) suction_off_y_thresh = 0.32;
             else suction_off_y_thresh = 0.42;

             for(size_t i = 0; i < loop_limit; i++)
             {
                 actual_pose = group_left_arm.getCurrentPose("suction_cup");
                 if ((actual_pose.pose.position.z < suction_on_thresh) && (suction_flag == false))
                 {
                    //Suck the object
                    ros::Duration(0.1).sleep();
                    sub20_srv.request.PORT0_5_command = true;
                    sub20_srv.request.pin_to_change = "PORT0_5";
                    sub20_client.call(sub20_srv);
                    suction_flag = true;
                 }

                 if ((actual_pose.pose.position.y < suction_off_y_thresh) && (actual_pose.pose.position.z < suction_off_z_thresh) && (suction_flag == true))
                 {
                     //Release the object
                     sub20_srv.request.PORT0_5_command = false;
                     sub20_srv.request.pin_to_change = "PORT0_5";
                     sub20_client.call(sub20_srv);
                     suction_flag = false;
                 }
                 ros::Duration(0.01).sleep();
             }
             ros::Duration(2).sleep();
         }

         else
         {
             std::cout << "Not able to reach the object" << std::endl;
             ros::Duration(1).sleep();\
         }

          //======================================================
          //Place bin on table
          //======================================================
          if(number_of_remaining_objects == 0)
          {
             std::vector<geometry_msgs::Pose> arm_right_cartesian_waypoints;
             moveit_msgs::RobotTrajectory arm_right_trajectory;

             //=====================================================
             //Trajectory composition
             //=====================================================
             arm_right_cartesian_waypoints.push_back(sda10f.approach_to_table);
             arm_right_cartesian_waypoints.push_back(sda10f.place_bin_on_table);
             arm_right_cartesian_waypoints.push_back(sda10f.release_contact);
             arm_right_cartesian_waypoints.push_back(sda10f.move_away_from_bin);
             arm_right_cartesian_waypoints.push_back(sda10f.back_to_initial_pose);

             //Several attempts of cartesian trajectory computation
             const int max_cartesian_attempts = 20;
             int acutal_number_of_cartesian_attempts = 1;
             double arm_right_fraction = 0;
             while ((arm_right_fraction != 1) && (acutal_number_of_cartesian_attempts < max_cartesian_attempts))
             {
                arm_right_fraction = group_right_arm.computeCartesianPath(arm_right_cartesian_waypoints, 0.02, 0, arm_right_trajectory, false);
                ROS_INFO("Cartesian computation n. %i (cartesian path) (%.2f%% acheived)",acutal_number_of_cartesian_attempts, arm_right_fraction * 100.0);
                acutal_number_of_cartesian_attempts++;
             }

             //=======================================================
             //Robot movement
             //=======================================================
             if (arm_right_fraction == 1)
             {
                //Adding velocities to cartesian trajectory
                robot_trajectory::RobotTrajectory arm_right_rt_01(group_right_arm.getCurrentState()->getRobotModel(),"arm_right");
                arm_right_rt_01.setRobotTrajectoryMsg(*group_right_arm.getCurrentState(), arm_right_trajectory);
                trajectory_processing::IterativeParabolicTimeParameterization arm_right_iptp;
                //Compute Time Stamps
                const double max_velocity_scaling = 1.0;
                bool arm_right_success = arm_right_iptp.computeTimeStamps(arm_right_rt_01, max_velocity_scaling);
                ROS_INFO("Computed time stamps: %s", arm_right_success ? "SUCCEDED" : "FAILED");

                //Get robot trajectory from RobotTrajectory
                arm_right_rt_01.getRobotTrajectoryMsg(arm_right_trajectory);
                arm_right_plan.trajectory_ = arm_right_trajectory;
                //ROS_INFO_STREAM("Trajectory" << arm_left_trajectory);
                group_right_arm.execute(arm_right_plan);
            }
            ros::Duration(2).sleep();
        }
     }

    return(EXIT_SUCCESS);
}
