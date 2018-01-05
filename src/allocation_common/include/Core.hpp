#ifndef CORE_HPP
#define CORE_HPP

#include <Circle.hpp>
#include <Line.hpp>
#include <PPoint.hpp>
#include <DPoint.hpp>
#include <Angle.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
	
#define MAXNUM_AGENT 10
#define LOCATION_ERROR 0.05

enum Robots_mode
{
    IDLE,                            //has nothing to do
    PLAN,                            //make a plan to complete a task
    EXECUTE,                         //execute the task
    EXPLORE,                         //explore the unknown region
    HIT,                             //hit the target region
    DAMAGE                           //robot is damaged
};

enum Allocation_mode
{
    ALLOCATION_START,
    ALLOCATION_PAUSE,
    ALLOCATION_STOP
};

struct Gazebo_robot_info
{
    /// these information are obtained from gazebo
    int    robot_ID;
    DPoint robot_pos;
    double robot_ori;
    DPoint robot_vel;
    double robot_w;
};

struct Gazebo_task_info
{
    /// these information are obtained from gazebo
    int    task_ID;
    DPoint task_pos;
};

struct Allocation_robot_info
{
    /// these information are obtained form control_terminal
    int    robot_power;
    /// these information are obtained from task_allocation
    int    robot_ID;
    char   robot_mode;
    int    which_task;
    int    which_target;
    bool   isvalid;
    Allocation_robot_info()
    {
        robot_power=0;
        robot_ID=0;
        robot_mode=IDLE;
        which_task=-1;
        which_target=-1;
        isvalid=true;
    }
};

struct Allocation_task_info
{
    /// these information are obtained form control_terminal
    int    task_power;
    /// these information are obtained from task_allocation
    int    task_ID;
    int    known_power;
    double current_distance;
    bool   istarget;
    bool   iscomplete;
    bool   isexplored;
    Allocation_task_info()
    {
        task_power=0;
        task_ID=0;
        known_power=0;
        current_distance=1000;
        istarget=false;
        iscomplete=false;
        isexplored=false;
    }
};

struct Robot_info
{
    Gazebo_robot_info gazebo_robot_info;
    Allocation_robot_info allocation_robot_info;
};

struct Task_info
{
    Gazebo_task_info gazebo_task_info;
    Allocation_task_info allocation_task_info;
};

struct Terminal2Gazebo_info
{
    std::vector<int>  robot_pos_x;
    std::vector<int>  robot_pos_y;
    std::vector<int>  task_pos_x;
    std::vector<int>  task_pos_y;
    bool isNew_allocation;
};

struct Terminal2Robots_info
{
    char allocation_mode;              //start 1, stop 0
    bool greedorprobability;           //greed 0, probability 1
    bool recordornot;                  //not record 0, record 1
    bool powerordistance;              //power 0, distance 1

    Allocation_robot_info all_allocation_robot_info[MAXNUM_AGENT];
    Allocation_task_info all_allocation_task_info[MAXNUM_AGENT];
};

struct Gazebo2World_info
{
    std::vector<Gazebo_robot_info> gazebo_robots_info;
    std::vector<Gazebo_task_info>  gazebo_tasks_info;
};
#endif //CORE_HPP
