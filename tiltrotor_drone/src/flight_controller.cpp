/**
 * package:  tiltrotor_drone
 * node:     flight_controller
 * file:     flight_controller.cpp
 * authors:  Efe Camci
             Yun Ting Chen
 * company:  Institute for Infocomm Research (I2R), A*STAR Singapore
 * date:     Jan 2025
 *
 * @copyright
 * Copyright (C) 2025.
 */

#include <flight_controller.h>

// Callback functions
void FlightController::cbReconfig(tiltrotor_drone::flight_controllerConfig &config, uint32_t level)
{
    _desired.x = config.desired_x;
    _desired.y = config.desired_y;
    _desired.z = config.desired_z;
    _desired.f = config.desired_f;
    _desired.yaw_deg = config.desired_yaw;
    _desired.pitch = config.desired_pitch;
    _desired.servo_angle = config.desired_servo_angle;
    _kp.f = config.kp_f;
    _ki.f = config.ki_f;
    _kd.f = config.kd_f;
}

void FlightController::cbGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    int i = 0;
    while (!_drone_name_received)
    {
        if (!msg->name[i].compare(_drone_name))
        {
            _index = i;
            _drone_name_received = true;
        }
        i++;
    }

    if (_drone_name_received)
    {
        _current_drone_pose_msg.pose.position.x = msg->pose[_index].position.x;
        _current_drone_pose_msg.pose.position.y = msg->pose[_index].position.y;
        _current_drone_pose_msg.pose.position.z = msg->pose[_index].position.z;

        _current_drone_pose_msg.pose.orientation.x = msg->pose[_index].orientation.x;
        _current_drone_pose_msg.pose.orientation.y = msg->pose[_index].orientation.y;
        _current_drone_pose_msg.pose.orientation.z = msg->pose[_index].orientation.z;
        _current_drone_pose_msg.pose.orientation.w = msg->pose[_index].orientation.w;
    }
}

void FlightController::cbDroneState(const mavros_msgs::State::ConstPtr &msg)
{
    _current_drone_state_msg = *msg;
}

void FlightController::cbContactsState(const gazebo_msgs::ContactsState::ConstPtr &msg)
{
    if (!msg->states.empty()) 
    {
        _contact_f = msg->states[0].total_wrench.force.x; 
        _contact_force_msg.data = msg->states[0].total_wrench.force.x; 
    } 
    else 
    {
        _contact_f = 0.0;
        _contact_force_msg.data = 0.0;
    }
    ROS_INFO("contact force: %f", _contact_f);
    _contact_force_pub.publish(_contact_force_msg);
}

// Initialize the parameters
void FlightController::initRosComms(ros::NodeHandle *nh)
{
    // Publishers
    _setpoint_raw_pub =
    nh->advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

    _current_drone_pose_pub =
    nh->advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);

    _setpoint_pitch_pub = 
    nh->advertise<mavros_msgs::DebugValue>("mavros/debug_value/send", 1);

    // _servo_angle_pub = 
    // nh->advertise<std_msgs::Float64>("joint_position_cmd", 1);

    _servo_angle_pub = //no cable setup
    nh->advertise<std_msgs::Float64>("gripper_joint_position_cmd", 1);

    _contact_force_pub = 
    nh->advertise<std_msgs::Float64>("force_value", 1);

    // Subscribers
    _gazebo_model_state_sub =
    nh->subscribe<gazebo_msgs::ModelStates>("gazebo/model_states",
                                            1,
                                            &FlightController::cbGazeboModelState,
                                            this);
    _current_drone_state_sub =
    nh->subscribe<mavros_msgs::State>("mavros/state",
                                      1,
                                      &FlightController::cbDroneState,
                                      this);

    _contact_force_sub = 
    nh->subscribe<gazebo_msgs::ContactsState>("contactsensor",
                                      1,
                                      &FlightController::cbContactsState,
                                      this);

    // Servers
    dynamic_reconfigure::Server<tiltrotor_drone::flight_controllerConfig>::CallbackType reconfig_f;
    reconfig_f = boost::bind(&FlightController::cbReconfig, this, _1, _2);
    reconfig_ser.setCallback(reconfig_f);

    // Clients
    _arm_cli =
    nh->serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    _set_mode_cli =
    nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void FlightController::initParams()
{
    ros::param::get(ros::this_node::getName() + "/node_rate", node_rate);
    ros::param::get(ros::this_node::getName() + "/drone_name", _drone_name);
    ros::param::get(ros::this_node::getName() + "/kp_xy", _kp.xy);
    ros::param::get(ros::this_node::getName() + "/ki_xy", _ki.xy);
    ros::param::get(ros::this_node::getName() + "/kd_xy", _kd.xy);
    ros::param::get(ros::this_node::getName() + "/kp_z", _kp.z);
    ros::param::get(ros::this_node::getName() + "/ki_z", _ki.z);
    ros::param::get(ros::this_node::getName() + "/kd_z", _kd.z);
    ros::param::get(ros::this_node::getName() + "/lim_uxy", _lim_u.xy);
    ros::param::get(ros::this_node::getName() + "/lim_uz", _lim_u.z);

    _drone_name_received = false;
    _sp_adjusted = false;
}

void FlightController::sendCurrentDronePose()
{
    if (_drone_name_received)
    {
        _current_drone_pose_msg.header.stamp = ros::Time::now();
        _current_drone_pose_msg.header.frame_id = "map";
        _current_drone_pose_pub.publish(_current_drone_pose_msg);

        _current.x = _current_drone_pose_msg.pose.position.x;
        _current.y = _current_drone_pose_msg.pose.position.y;
        _current.z = _current_drone_pose_msg.pose.position.z;
        _current.yaw_deg = getYawDeg(_current_drone_pose_msg);
        _current.yaw_rad =  3.1416 * (_current.yaw_deg / 180);
        _current.f = _contact_f;
    }
}

void FlightController::sendCmds()
{
    if (_current_drone_state_msg.mode != "OFFBOARD")
    {
        _mode_msg.request.custom_mode = "OFFBOARD";
        _set_mode_cli.call(_mode_msg);
    }

    if (!_current_drone_state_msg.armed)
    {
        _arm_msg.request.broadcast = false;
        _arm_msg.request.command = 400;
        _arm_msg.request.confirmation = 0;
        _arm_msg.request.param1 = 1;
        _arm_msg.request.param2 = 21196;
        _arm_cli.call(_arm_msg);
    }

    if (_desired.servo_angle >= 90)
    {
        //current xyz = setpoint xyz
        if (_sp_adjusted == false)
        {
            _x_sp_temp = _current.x;
            _y_sp_temp = _current.y;
            _z_sp_temp = _current.z;
            _sp_adjusted = true;
        }
        _desired.x = _x_sp_temp;
        _desired.y = _y_sp_temp;
        _desired.z = _z_sp_temp;
    }
    else 
    {
        _sp_adjusted = false;
    }
    ROS_INFO("sp x: %f", _desired.x);
    ROS_INFO("sp y: %f", _desired.y);
    ROS_INFO("sp z: %f", _desired.z);

    // Calculate errors and changes in errors to be input to PD controllers
    _t1 = std::chrono::high_resolution_clock::now();
    _dt = std::chrono::duration_cast<std::chrono::duration<double>>(_t1 - _t0);

    dt = std::max(_dt.count(), 0.001);

    _previous_e.x = _e.x;
    _previous_e.y = _e.y;
    _previous_e.z = _e.z;
    _previous_e.f = _e.f;

    _e.x = _desired.x - _current.x;
    _e.y = _desired.y - _current.y;
    _e.z = _desired.z - _current.z;
    _e.f = _current.f - _desired.f;

    _i_e.x += _e.x * dt;
    _i_e.y += _e.y * dt;
    _i_e.z += _e.z * dt;
    _i_e.f += _e.f * dt;

    _d_e.x = (_e.x - _previous_e.x) / dt;
    _d_e.y = (_e.y - _previous_e.y) / dt;
    _d_e.z = (_e.z - _previous_e.z) / dt;
    _d_e.f = (_e.f - _previous_e.f) / dt;

    _t0 = std::chrono::high_resolution_clock::now();

    if (_current.f < 0) 
    {
        _local_u.x = (_kp.f * _e.f + _kd.f * _d_e.f + _ki.f * _i_e.f)*0.01; 
    }
    else 
    {
        _local_u.x = _kp.xy * _e.x + _kd.xy * _d_e.x + _ki.xy * _i_e.x;
    }

    _local_u.y = _kp.xy * _e.y + _kd.xy * _d_e.y + _ki.xy * _i_e.y;
    _u.z = _kp.z * _e.z + _kd.z * _d_e.z + _ki.z * _i_e.z;

    if (_current.f < 0)
    {
        _u.x = _local_u.x;
    }
    else 
    {
        _u.x = cos(_current.yaw_rad) * _local_u.x + sin(_current.yaw_rad) * _local_u.y;
    }
    _u.y = -sin(_current.yaw_rad) * _local_u.x + cos(_current.yaw_rad) * _local_u.y;

    _u.x = std::min(_u.x, _lim_u.xy);
    _u.x = std::max(_u.x, -_lim_u.xy);

    _u.y = std::min(_u.y, _lim_u.xy);
    _u.y = std::max(_u.y, -_lim_u.xy);
    
    _u.z = std::min(_u.z, _lim_u.z);
    _u.z = std::max(_u.z, -_lim_u.z);

    ROS_WARN("vel x sp: %f", _u.x);
    ROS_WARN("vel y sp: %f", _u.y);
    ROS_WARN("vel z sp: %f", _u.z);

    _setpoint_raw_msg.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 512 + 2048;
    _setpoint_raw_msg.coordinate_frame = 8; //1
    _setpoint_raw_msg.velocity.x = _u.x;
    _setpoint_raw_msg.velocity.y = _u.y;
    _setpoint_raw_msg.velocity.z = _u.z;
    _setpoint_raw_msg.yaw = 3.1416 * (_desired.yaw_deg / 180);
    _setpoint_raw_pub.publish(_setpoint_raw_msg);

    _setpoint_pitch_msg.type = 0;
    _setpoint_pitch_msg.value_float = 3.1416 * (_desired.pitch / 180);
    _setpoint_pitch_pub.publish(_setpoint_pitch_msg);

    _servo_angle_msg.data = 3.1416 * (_desired.servo_angle / 180);
    _servo_angle_pub.publish(_servo_angle_msg);
    
    _setpoint_pitch_msg.type = 2;
    _setpoint_pitch_msg.data.resize(1);
    _setpoint_pitch_msg.data[0] = _contact_f;
    _setpoint_pitch_pub.publish(_setpoint_pitch_msg);

}

double FlightController::getYawDeg(geometry_msgs::PoseStamped &msg)
{
    tf2::Quaternion q(msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r,p,y;
    m.getRPY(r,p,y);
    y = (y / 3.1416) * 180;
    if (y < 0)
        y += 360;

    return y;
}