#ifndef TASK_GAZEBO_H
#define TASK_GAZEBO_H

#include "gazebo_info.h"

class Task_Gazebo : public gazebo::ModelPlugin
{
private:

    physics::WorldPtr                   world_;                              //a pointer to the gazebo world.
    physics::ModelPtr                   task_model_;                         //pointer to the robot model

    ros::NodeHandle*                    rosnode_;
    ros::Subscriber                     robot2task_sub_;
    ros::Subscriber                     terminal2robot_sub_;                //scriber the terminal info

    boost::thread                       message_callback_queue_thread_;      //thead object for the running callback Thread.
    boost::mutex                        msgCB_lock_;                         //a mutex to lock access to fields that are used in ROS message callbacks
    ros::CallbackQueue                  message_queue_;                      //custom Callback Queue

    ignition::math::Rand                rand_;

    std::string                         model_name_;
    std::string                         robot_namespace_;
    int                                 taskID_;
    std::vector<int>                    destroy_tasks_;                      //the task which have been destroyed

    event::ConnectionPtr                updateConnection_;

    Terminal2Robots_info                terminal_info_;                     //the current terminal info for allocation

public:
    Task_Gazebo();
    virtual ~Task_Gazebo();
    void    message_queue_thread();
    void    OnUpdate();
    void    update_terminal_info(const allocation_common::terminal2robot_info::ConstPtr & _msg);    
    
//    void    task_state_CB(const allocation_common::allocation_task_info::ConstPtr & _msg);
    
protected:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void Reset();
};

#endif //! TASK_GAZEBO_H
