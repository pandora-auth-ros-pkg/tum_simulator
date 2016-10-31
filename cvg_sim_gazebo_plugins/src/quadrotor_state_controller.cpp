/*
* quadrotor_state_controller:
*
* This software is a state control gazebo plugin for the Ardrone simulator
*
* It receives the joystick command and the current simulator information to generate corresponding information in rostopic /ardrone/navdata
*
* Created on: Oct 22, 2012
* Author: Hongrong huang
*
*
*/

#define _RAD2DEG 57.2957795

#include <hector_quadrotor_controller/quadrotor_state_controller.h>
#include "gazebo/physics/physics.hh"

namespace gazebo {

GazeboQuadrotorStateController::GazeboQuadrotorStateController()
{
  robot_current_state = INITIALIZE_MODEL;
  m_isFlying          = false;
  m_takeoff           = false; 
  m_drainBattery      = true;
  m_batteryPercentage = 100;
  m_maxFlightTime     = 1200;
  m_timeAfterTakeOff  = 0;
  m_selected_cam_num  = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorStateController::~GazeboQuadrotorStateController()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorStateController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    world = _model->GetWorld();

    // ardrone autonomy topic names
    takeoff_topic_ = "/ardrone/takeoff";
    land_topic_ = "/ardrone/land";
    reset_topic_ = "/ardrone/reset";
    navdata_topic_ = "/ardrone/navdata";
    navdataraw_topic_ = "/ardrone/navdata_raw_measures";
    imu_topic_ = "/ardrone/imu";
    mag_topic_ = "/ardrone/mag";

    // load parameters
    if (!_sdf->HasElement("robotNamespace")) namespace_.clear();
    else namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    if (!_sdf->HasElement("imuTopic")) imu_plugin_topic_.clear();
    else imu_plugin_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();

    if (!_sdf->HasElement("sonarTopic")) sonar_plugin_topic_.clear();
    else sonar_plugin_topic_ = _sdf->GetElement("sonarTopic")->Get<std::string>();

    if (!_sdf->HasElement("magTopic")) mag_topic_.clear();
    else mag_plugin_topic_ = _sdf->GetElement("magTopic")->Get<std::string>();

    if (!_sdf->HasElement("bodyName")) {
        link = _model->GetLink();
        link_name_ = link->GetName(); }
    else {
        link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
        link = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_name_)); }

    if (!link) {
        ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
        return; }

    node_handle_ = new ros::NodeHandle(namespace_);

    // Initialize optical flow speed model
    speedModel.Load(_sdf, "speed");

    // subscribe command: take off command
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(takeoff_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::TakeoffCallback, this, _1), ros::VoidPtr(), &callback_queue_);
    takeoff_subscriber_ = node_handle_->subscribe(ops);

    // subscribe command: land command
    ops = ros::SubscribeOptions::create<std_msgs::Empty>(land_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::LandCallback, this, _1), ros::VoidPtr(), &callback_queue_);
    land_subscriber_ = node_handle_->subscribe(ops);

    // subscribe command: reset command
    ops = ros::SubscribeOptions::create<std_msgs::Empty>(reset_topic_, 1,
      boost::bind(&GazeboQuadrotorStateController::ResetCallback, this, _1), ros::VoidPtr(), &callback_queue_);
    reset_subscriber_ = node_handle_->subscribe(ops);

  	// publish ardrone autonomy topics
    m_navdataPub = node_handle_->advertise< ardrone_autonomy::Navdata >( navdata_topic_ , 25 );
    m_navdatarawPub = node_handle_->advertise< ardrone_autonomy::navdata_raw_measures >( navdataraw_topic_ , 25 );
    m_imuPub = node_handle_->advertise< sensor_msgs::Imu >( imu_topic_ , 25 );
    m_magPub = node_handle_->advertise< geometry_msgs::Vector3Stamped >( mag_topic_, 25);

    // subscribe imu plugin
    if (!imu_plugin_topic_.empty()) {
        ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(imu_plugin_topic_, 1,
        boost::bind(&GazeboQuadrotorStateController::ImuCallback, this, _1), ros::VoidPtr(), &callback_queue_);
        imu_subscriber_ = node_handle_->subscribe(ops);

        ROS_INFO_NAMED("quadrotor_state_controller", "Using imu information on topic %s as source of orientation, "
                "angular velocity and linear acceleration.", imu_plugin_topic_.c_str());
    }

    // subscribe sonar plugin
    if (!sonar_plugin_topic_.empty()) {
        ops = ros::SubscribeOptions::create<sensor_msgs::Range>(sonar_plugin_topic_, 1,
          boost::bind(&GazeboQuadrotorStateController::SonarCallback, this, _1), ros::VoidPtr(), &callback_queue_);
        sonar_subscriber_ = node_handle_->subscribe(ops);

        ROS_INFO_NAMED("quadrotor_state_controller", "Using sonar information on topic %s as source of altitude.",
                       sonar_plugin_topic_.c_str());
    }

    // subscribe magnetometer plugin
    if (!mag_plugin_topic_.empty()) {
        ops = ros::SubscribeOptions::create<geometry_msgs::Vector3Stamped>(mag_plugin_topic_, 1,
            boost::bind(&GazeboQuadrotorStateController::MagCallback, this, _1), ros::VoidPtr(), &callback_queue_);
        mag_subscriber_ = node_handle_->subscribe(ops);

        ROS_INFO_NAMED("quadrotor_state_controller", "Using magnetic information on topic %s as source of orientation.", mag_topic_.c_str());
    }

    // for camera control
    // switch camera server
    std::string toggleCam_topic  = "ardrone/togglecam";
    ros::AdvertiseServiceOptions toggleCam_ops = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(toggleCam_topic,
    boost::bind(&GazeboQuadrotorStateController::toggleCamCallback, this, _1,_2), ros::VoidPtr(), &callback_queue_);

    toggleCam_service = node_handle_->advertiseService(toggleCam_ops);

    // camera image data
    std::string cam_out_topic  = "/ardrone/image_raw";
    std::string cam_front_in_topic = "/ardrone/front/image_raw";
    std::string cam_bottom_in_topic = "/ardrone/bottom/image_raw";
    std::string in_transport = "raw";

    camera_it_ = new image_transport::ImageTransport(*node_handle_);
    camera_publisher_ = camera_it_->advertise(cam_out_topic, 1);

    camera_front_subscriber_ = camera_it_->subscribe(cam_front_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraFrontCallback, this, _1), ros::VoidPtr(), in_transport);

    camera_bottom_subscriber_ = camera_it_->subscribe(cam_bottom_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraBottomCallback, this, _1), ros::VoidPtr(), in_transport);

    // camera image data
    std::string cam_info_out_topic  = "/ardrone/camera_info";
    std::string cam_info_front_in_topic = "/ardrone/front/camera_info";
    std::string cam_info_bottom_in_topic = "/ardrone/bottom/camera_info";

    camera_info_publisher_ = node_handle_->advertise<sensor_msgs::CameraInfo>(cam_info_out_topic,1);

    ros::SubscribeOptions cam_info_front_ops = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(cam_info_front_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraInfoFrontCallback, this, _1), ros::VoidPtr(), &callback_queue_);
    camera_info_front_subscriber_ = node_handle_->subscribe(cam_info_front_ops);

    ros::SubscribeOptions cam_info_bottom_ops = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(cam_info_bottom_in_topic, 1,
    boost::bind(&GazeboQuadrotorStateController::CameraInfoBottomCallback, this, _1), ros::VoidPtr(), &callback_queue_);
    camera_info_bottom_subscriber_ = node_handle_->subscribe(cam_info_bottom_ops);

    Reset();

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboQuadrotorStateController::Update, this));

    robot_current_state = LANDED_MODEL;
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorStateController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
    pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    euler = pose.rot.GetAsEuler();
    angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
    acceleration.Set(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
}

void GazeboQuadrotorStateController::SonarCallback(const sensor_msgs::RangeConstPtr& sonar_info)
{
    robot_altitude = sonar_info->range;
}

void GazeboQuadrotorStateController::MagCallback(const geometry_msgs::Vector3StampedConstPtr& mag){
    magX = mag->vector.x;
    magY = mag->vector.y;
    magX = mag->vector.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorStateController::Update()
{

    // Get new commands/state
    callback_queue_.callAvailable();

    // Get simulator time
    common::Time sim_time = world->GetSimTime();
    double dt = (sim_time - last_time).Double();
    // Update rate is 200/per second
    if (dt < 0.005) return;

    // Get Pose/Orientation from Gazebo (if no imu subscriber is active)
    if (imu_topic_.empty()) {
        pose = link->GetWorldPose();
        angular_velocity = link->GetWorldAngularVel();
        euler = pose.rot.GetAsEuler();
        acceleration = (link->GetWorldLinearVel() - velocity) / dt;
    }

    // Get velocity from gazebo and add noise in order to simulate ux,uy from vision
    velocity = link->GetRelativeLinearVel();

    // Update velocity in order to add errors
    velocity += speedModel.update(dt);

    // process robot operation information
    if((m_takeoff)&&(robot_current_state == LANDED_MODEL))
    {
        m_timeAfterTakeOff = 0;
        m_takeoff = false;
        robot_current_state = TAKINGOFF_MODEL;
    }
    else if(robot_current_state == TAKINGOFF_MODEL)
    {
        // take off phase need more power
        if (!sonar_plugin_topic_.empty())
        {
          if(robot_altitude > 0.5)
          {
            robot_current_state = FLYING_MODEL;
          }
        }
        else
        {
            m_timeAfterTakeOff += dt;
            if(m_timeAfterTakeOff > 0.5)
            {
                //ROS_INFO("%f",m_timeAfterTakeOff);
                robot_current_state = FLYING_MODEL;
            }
        }
        if(m_isFlying == false)
        {
            m_timeAfterTakeOff = 0;
            robot_current_state = LANDING_MODEL;
        }
    }
    else if((robot_current_state == FLYING_MODEL)||(robot_current_state == TO_FIX_POINT_MODEL))
    {
        if(m_isFlying == false)
        {
            m_timeAfterTakeOff = 0;
            robot_current_state = LANDING_MODEL;
        }
    }
    else if(robot_current_state == LANDING_MODEL)
    {
        if (!sonar_plugin_topic_.empty())
        {
            m_timeAfterTakeOff += dt;
            if((robot_altitude < 0.2)||(m_timeAfterTakeOff > 5.0))
            {
                robot_current_state = LANDED_MODEL;
            }
        }
        else
        {
            m_timeAfterTakeOff += dt;
            if(m_timeAfterTakeOff > 1.0)
            {
                robot_current_state = LANDED_MODEL;
            }
        }

        if(m_isFlying == true)
        {
            m_timeAfterTakeOff = 0;
            m_takeoff = false;
            robot_current_state = TAKINGOFF_MODEL;
        }
    }

    if( ((robot_current_state != LANDED_MODEL)||m_isFlying) && m_drainBattery )
    m_batteryPercentage -= dt / m_maxFlightTime * 100.;

    ardrone_autonomy::Navdata navdata;
    navdata.batteryPercent = m_batteryPercentage;

    // from imu
    navdata.rotX = pose.rot.GetRoll() / M_PI * 180.;
    navdata.rotY = pose.rot.GetPitch() / M_PI * 180.;
    navdata.rotZ = pose.rot.GetYaw() / M_PI * 180.;

    // from sonar
    if (!sonar_plugin_topic_.empty()) navdata.altd = int(robot_altitude*1000);
    else navdata.altd = pose.pos.z * 1000.f;

    // from gazebo
    navdata.vx = 1000*velocity.x;
    navdata.vy = 1000*velocity.y;
    navdata.vz = -0.0;

    // from imu
    navdata.ax = acceleration.x/10;
    navdata.ay = acceleration.y/10;
    navdata.az = acceleration.z/10 + 1;

    // FIXME what is the real drone sending here?
    navdata.tm = ros::Time::now().toSec()*1000000;

    navdata.header.stamp = ros::Time::now();
    navdata.header.frame_id = "ardrone_base_link";
    navdata.state = robot_current_state;

    // from magnetometer
    navdata.magX = magX * _RAD2DEG;
    navdata.magY = magY * _RAD2DEG;
    navdata.magZ = magZ * _RAD2DEG;

    // todo add pressure from baro
    navdata.pressure = 0;

    // not available
    navdata.temp = 0;

    // not available
    // todo add wind variables from air friction plugin which could simulate also wind force
    navdata.wind_speed = 0.0;
    navdata.wind_angle = 0.0;
    navdata.wind_comp_angle = 0.0;

    //not available
    navdata.tags_count = 0;

    m_navdataPub.publish( navdata );

    ardrone_autonomy::navdata_raw_measures navdataraw;

    // todo add proper values to variables now its just a dummy topic
    navdataraw.header.stamp = ros::Time::now();
    navdataraw.header.frame_id = "ardrone_base_link";
    navdataraw.drone_time = 0;
    navdataraw.tag = 0;
    navdataraw.size = 0;
    navdataraw.vbat_raw = 0;
    navdataraw.us_debut_echo = 0;
    navdataraw.us_fin_echo = 0;
    navdataraw.us_association_echo = 0;
    navdataraw.us_distance_echo = 0;
    navdataraw.us_courbe_temps= 0;
    navdataraw.us_courbe_valeur = 0;
    navdataraw.us_courbe_ref = 0;
    navdataraw.flag_echo_ini = 0;
    navdataraw.nb_echo = 0;
    navdataraw.sum_echo = 0;
    if (!sonar_plugin_topic_.empty()) navdataraw.alt_temp_raw = int(robot_altitude*1000);
    else navdataraw.alt_temp_raw = int(pose.pos.z * 1000.f);
    navdataraw.gradient = 0;

    m_navdatarawPub.publish( navdataraw );

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "ardrone_base_link";
    imu_msg.angular_velocity.x = angular_velocity.x;
    imu_msg.angular_velocity.y = angular_velocity.y;
    imu_msg.angular_velocity.z = angular_velocity.z;
    imu_msg.linear_acceleration.x = acceleration.x;
    imu_msg.linear_acceleration.y = acceleration.y;
    imu_msg.linear_acceleration.z = acceleration.z;
    imu_msg.orientation.x = pose.rot.x;
    imu_msg.orientation.y = pose.rot.y;
    imu_msg.orientation.z = pose.rot.z;
    imu_msg.orientation.w = pose.rot.w;

    m_imuPub.publish(imu_msg);

    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header.stamp = ros::Time::now();
    mag_msg.header.frame_id = "ardrone_base_link";
    mag_msg.vector.x = magX;
    mag_msg.vector.y = magY;
    mag_msg.vector.z = magZ;

    m_magPub.publish(mag_msg);

    // save last time stamp
    last_time = sim_time;

}
////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorStateController::Reset()
{
    // reset state
    pose.Reset();
    velocity.Set();
    angular_velocity.Set();
    acceleration.Set();
    euler.Set();
    state_stamp = ros::Time();
}

////////////////////////////////////////////////////////////////////////////////
// controller callback
void GazeboQuadrotorStateController::TakeoffCallback(const std_msgs::EmptyConstPtr& msg)
{
    if(robot_current_state == LANDED_MODEL)
    {
        m_isFlying = true;
        m_takeoff = true;
        m_batteryPercentage = 100.;
        ROS_INFO("%s","\nQuadrotor takes off!!");
    }
    else if(robot_current_state == LANDING_MODEL)
    {
        m_isFlying = true;
        m_takeoff = true;
        ROS_INFO("%s","\nQuadrotor takes off!!");
    }
}

void GazeboQuadrotorStateController::LandCallback(const std_msgs::EmptyConstPtr& msg)
{
    if((robot_current_state == FLYING_MODEL)||(robot_current_state == TO_FIX_POINT_MODEL)||(robot_current_state == TAKINGOFF_MODEL))
    {
        m_isFlying = false;
        m_takeoff = false;
        ROS_INFO("%s","\nQuadrotor lands!!");
    }
}

void GazeboQuadrotorStateController::ResetCallback(const std_msgs::EmptyConstPtr& msg)
{
    ROS_INFO("%s","\nReset quadrotor!!");
}

bool GazeboQuadrotorStateController::toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(m_selected_cam_num==0) m_selected_cam_num = 1;
    else if(m_selected_cam_num==1) m_selected_cam_num = 0;

    ROS_INFO("\nSetting camera channel to : %d.\n", m_selected_cam_num);
    return true;
}

void GazeboQuadrotorStateController::CameraFrontCallback(const sensor_msgs::ImageConstPtr& image)
{
    if(m_selected_cam_num==0) camera_publisher_.publish(image);
}

void GazeboQuadrotorStateController::CameraBottomCallback(const sensor_msgs::ImageConstPtr& image)
{
    if(m_selected_cam_num==1) camera_publisher_.publish(image);
}

void GazeboQuadrotorStateController::CameraInfoFrontCallback(const sensor_msgs::CameraInfoConstPtr&  image_info)
{
    if(m_selected_cam_num==0) camera_info_publisher_.publish(image_info);
}

void GazeboQuadrotorStateController::CameraInfoBottomCallback(const sensor_msgs::CameraInfoConstPtr&  image_info)
{
    if(m_selected_cam_num==1) camera_info_publisher_.publish(image_info);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorStateController)

} // namespace gazebo
