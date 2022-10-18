#include <task_gazebo.h>
GZ_REGISTER_MODEL_PLUGIN(Task_Gazebo)

Task_Gazebo::Task_Gazebo()
{
    taskID_ = 0;
}

Task_Gazebo::~Task_Gazebo()
{
    // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish.
    message_queue_.clear();
    // Disable the queue, meaning any calls to addCallback() will have no effect.
    message_queue_.disable();
    rosnode_->shutdown();
    message_callback_queue_thread_.join();

    delete rosnode_;
}

/// \brief Load the controller. Required by model plugin. Will be called secondly
void Task_Gazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Get the world name.
    world_ = _model->GetWorld();
    task_model_= _model;
    model_name_= _model->GetName();
    robot_namespace_ = _model->GetName();
    taskID_ = atoi(model_name_.substr(task_name.size(),2).c_str());

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libnubot_gazebo.so' in the gazebo_ros package)");
        return;
    }
    rosnode_ = new ros::NodeHandle();

    //Subscribers
    ros::SubscribeOptions so = ros::SubscribeOptions::create <allocation_common::terminal2robot_info>(
                "/control_terminal/terminal2robot_info",100, boost::bind( &Task_Gazebo::update_terminal_info,this,_1),
                ros::VoidPtr(), &message_queue_);
    terminal2robot_sub_ = rosnode_->subscribe(so);

    //Subscribers
//    ros::SubscribeOptions so = ros::SubscribeOptions::create <allocation_common::allocation_task_info>(
//                "/task_allocation/task_state_info",1, boost::bind( &Task_Gazebo::task_state_CB,this,_1),
//                ros::VoidPtr(), &message_queue_);
//    robot2task_sub_ = rosnode_->subscribe(so);

    // Custom Callback Queue Thread. Use threads to process message and service callback queue
    message_callback_queue_thread_ = boost::thread( boost::bind( &Task_Gazebo::message_queue_thread,this ) );

    // Output info
    std::cout<<model_name_.c_str()<<" has "<<task_model_->GetPluginCount()<<" plugins, "<<"taskID: "<<taskID_<<std::endl;
    //task_model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

    // Listen to the update event. This event is broadcast every
      // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&Task_Gazebo::OnUpdate, this));
}

// Called by the world update start event
void Task_Gazebo::OnUpdate()
{
    
    if (terminal_info_.allocation_mode==ALLOCATION_START)
    {
        task_model_->SetLinearVel(ignition::math::Vector3d(0.1, 0.1, 0.0));
    }
    else
    {
        task_model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    }
    //task_model_->SetLinearVel(ignition::math::Vector3d(.1, .1, 0));
    // Apply a small linear velocity to the model.
    //std::cout<<terminal_info_.allocation_mode<<" mode is on "<<std::endl;
    // while (terminal_info_.allocation_mode==ALLOCATION_START)
    // {
    //     task_model_->SetLinearVel(ignition::math::Vector3d(.1, .1, 0));
    // }
    
}


/// \brief Model Reset function. Not required by model plugin. It is triggered when the world resets.
void Task_Gazebo::Reset()
{
    taskID_ = 0;
}

/// \brief Custom message callback queue thread
void Task_Gazebo::message_queue_thread()
{
    static const double timeout = 0.01;
    while (rosnode_->ok())
    {
        // Invoke all callbacks currently in the queue. If a callback was not ready to be called,
        // pushes it back onto the queue. This version includes a timeout which lets you specify
        // the amount of time to wait for a callback to be available before returning.
        message_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

/// \brief according to the current robot state which allocated this task, change the task state
//void Task_Gazebo::task_state_CB(const allocation_common::allocation_task_info::ConstPtr &_msg)
//{
//    msgCB_lock_.lock();
//    math::Pose obs_position;

//    if(_msg->task_ID!=taskID_)
//        return;
//    else if(_msg->iscomplete)
//    {
//        obs_position=math::Pose(math::Vector3(10,10,0), math::Quaternion(0,0,0));
//        task_model_->SetWorldPose(obs_position);
//    }
//    else if(_msg->istarget)
//    {

//    }
//    msgCB_lock_.unlock();
//}


/// \brief terminal information CB, about the position and num of tasks or robots
void Task_Gazebo::update_terminal_info(const allocation_common::terminal2robot_info::ConstPtr & _msg)
{
    msgCB_lock_.lock();
    terminal_info_.allocation_mode=_msg->allocation_mode;
    //std::cout<<_msg->allocation_mode<<" mode is on "<<std::endl;
    msgCB_lock_.unlock();
}