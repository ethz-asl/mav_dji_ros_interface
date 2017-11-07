/*
 Copyright (c) 2016, Mina Kamel and Inkyu Sa, ASL, ETH Zurich, Switzerland
 You can contact the author at <mina.kamel@mavt.ethz.ch> or <inkyu.sa@mavt.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "dji_interface/dji_interface.h"

namespace dji_interface {

const std::string DJIInterface::kScreenPrefix = "[dji interface]: ";
constexpr double DJIInterface::kAngularVelocityNoiseVariance;
constexpr double DJIInterface::kLinearAccelerationNoiseVariance;

DJIInterface::DJIInterface(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      dji_comm_(nh, private_nh),
      thrust_coefficient_(kDefaultThrustCoefficient),
      minimum_thrust_(kDefaultMinimumThrust),
      maximum_thrust_(kDefaultMaximumThrust),
      thrust_offset_(kDefaultMinimumThrust),
      initialized_(false)
{
  loadParameters();
  init();
}

DJIInterface::~DJIInterface()
{

}

void DJIInterface::loadParameters()
{
  private_nh_.param<std::string>("frame_id", frame_id_, "dji_interface");
  private_nh_.param<std::string>("device", device_, "/dev/ttyUSB0");
  private_nh_.param<double>("thrust_coefficient", thrust_coefficient_, thrust_coefficient_);
  private_nh_.param<double>("minimum_thrust", minimum_thrust_, minimum_thrust_);
  private_nh_.param<double>("maximum_thrust", maximum_thrust_, maximum_thrust_);
  private_nh_.param<double>("thrust_offset", thrust_offset_, thrust_offset_);
  std::cout << "device: " << device_ << std::endl;
  private_nh_.param<int>("baudrate", baudrate_, 921600);
  int app_id;
  private_nh_.param<int>("app_id", app_id, 1022384);
  activation_data_.ID = app_id;
  std::string enc_key;
  private_nh_.param<std::string>("enc_key", enc_key,
                                 "e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d");

  char key[65];
  activation_data_.encKey = key;
  strcpy(activation_data_.encKey, enc_key.c_str());

  private_nh_.param<std::string>("drone_version", drone_version_, "M100");
  std::cout << "drone_version: " << drone_version_ << std::endl;

  if (drone_version_ == "M100") {
    activation_data_.version = DJI::onboardSDK::versionM100_31;
  } else if (drone_version_ == "A3_31") {
    activation_data_.version = DJI::onboardSDK::versionA3_31;
  } else if (drone_version_ == "A3_32") {
    activation_data_.version = DJI::onboardSDK::versionA3_32;
  } else {
    ROS_ERROR_STREAM(kScreenPrefix + "Unknown drone version");
    abort();
  }

  // load packets freq
  int imu_update_rate;
  int gps_updae_rate;
  int time_stamp_update_rate;
  int magnetometer_update_rate;
  int rc_update_rate;
  int status_update_rate;

  private_nh_.param<int>("imu_update_rate", imu_update_rate, DJI::onboardSDK::BROADCAST_FREQ_100HZ);
  private_nh_.param<int>("gps_updae_rate", gps_updae_rate, DJI::onboardSDK::BROADCAST_FREQ_10HZ);
  private_nh_.param<int>("time_stamp_update_rate", time_stamp_update_rate, DJI::onboardSDK::BROADCAST_FREQ_1HZ);
  private_nh_.param<int>("magnetometer_update_rate", magnetometer_update_rate, DJI::onboardSDK::BROADCAST_FREQ_10HZ);
  private_nh_.param<int>("rc_update_rate", rc_update_rate, DJI::onboardSDK::BROADCAST_FREQ_50HZ);
  private_nh_.param<int>("status_update_rate", status_update_rate, DJI::onboardSDK::BROADCAST_FREQ_10HZ);

  //  = {DJI::onboardSDK::BROADCAST_FREQ_0HZ};
  broadcast_frequency_.resize(kBroadcastFrequencySize, DJI::onboardSDK::BROADCAST_FREQ_0HZ);

  if (drone_version_ == "M100") {
    // M100

    //timestamp
    broadcast_frequency_[0] = getFrequencyValue(time_stamp_update_rate);
    //IMU
    broadcast_frequency_[1] = getFrequencyValue(imu_update_rate);
    broadcast_frequency_[2] = getFrequencyValue(imu_update_rate);
    broadcast_frequency_[4] = getFrequencyValue(imu_update_rate);
    //GPS
    broadcast_frequency_[3] = getFrequencyValue(gps_updae_rate);
    broadcast_frequency_[5] = getFrequencyValue(gps_updae_rate);
    //magnetometer
    broadcast_frequency_[6] = getFrequencyValue(magnetometer_update_rate);
    //rc data
    broadcast_frequency_[7] = getFrequencyValue(rc_update_rate);
    //status
    broadcast_frequency_[9] = getFrequencyValue(status_update_rate);
    broadcast_frequency_[10] = getFrequencyValue(status_update_rate);
    broadcast_frequency_[11] = getFrequencyValue(status_update_rate);
  } else {
    // A3

    //timestamp
    broadcast_frequency_[0] = getFrequencyValue(time_stamp_update_rate);
    //IMU
    broadcast_frequency_[1] = getFrequencyValue(imu_update_rate);
    broadcast_frequency_[2] = getFrequencyValue(imu_update_rate);
    broadcast_frequency_[4] = getFrequencyValue(imu_update_rate);
    //GPS
    broadcast_frequency_[3] = getFrequencyValue(gps_updae_rate);
    broadcast_frequency_[5] = getFrequencyValue(gps_updae_rate);
    broadcast_frequency_[6] = getFrequencyValue(gps_updae_rate);
    broadcast_frequency_[7] = getFrequencyValue(gps_updae_rate);
    //magnetometer
    broadcast_frequency_[8] = getFrequencyValue(magnetometer_update_rate);
    //rc data
    broadcast_frequency_[9] = getFrequencyValue(rc_update_rate);
    //status
    broadcast_frequency_[11] = getFrequencyValue(status_update_rate);
    broadcast_frequency_[12] = getFrequencyValue(status_update_rate);
    broadcast_frequency_[13] = getFrequencyValue(status_update_rate);
  }

  ROS_INFO("parameters loaded correctly");
}

void DJIInterface::init()
{
  dji_comm_.init(device_, baudrate_);
  dji_comm_.getFirmwareVersion(&firmware_version_);
  dji_comm_.activate(&activation_data_, NULL);
  dji_comm_.setBroadcastFrequency(broadcast_frequency_.data());
  dji_comm_.setBroadcastCallback(&DJIInterface::broadcastCallback, this);

  setPublishers();
  setSubscribers();
  initialized_ = true;
}

void DJIInterface::setPublishers()
{
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(mav_msgs::default_topics::IMU, 1);
  rc_pub_ = nh_.advertise<sensor_msgs::Joy>(mav_msgs::default_topics::RC, 1);
  status_pub_ = nh_.advertise<mav_msgs::Status>(mav_msgs::default_topics::STATUS, 1);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("gps", 1); //Hard-coded topic name, change this later.
}

void DJIInterface::setSubscribers()
{
  command_roll_pitch_yawrate_thrust_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
                                                         &DJIInterface::commandRollPitchYawrateThrustCallback, this, 
                                                         ros::TransportHints().tcpNoDelay());
}

void DJIInterface::commandRollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& msg)
{
  if(!initialized_){
    return;
  }

  ROS_INFO_STREAM_ONCE(kScreenPrefix + "Received first roll pitch yawrate thrust command msg");

  double roll_cmd = msg->roll*180.0/M_PI;
  double pitch_cmd = -msg->pitch*180.0/M_PI;
  double yaw_rate_cmd = -msg->yaw_rate*180.0/M_PI;
  double throttle_cmd = thrust_offset_ + msg->thrust.z*thrust_coefficient_;

  if(throttle_cmd < minimum_thrust_){
    ROS_WARN_STREAM_THROTTLE(0.1, kScreenPrefix + "Throttle command is below minimum.. set to minimum");
    throttle_cmd = minimum_thrust_;
  }
  if(throttle_cmd > maximum_thrust_){
    ROS_WARN_STREAM_THROTTLE(0.1, kScreenPrefix + "Throttle command is too high.. set to max");
    throttle_cmd = maximum_thrust_;
  }

  dji_comm_.setRollPitchYawrateThrust(roll_cmd, pitch_cmd, yaw_rate_cmd, throttle_cmd);
}

void DJIInterface::broadcastCallback()
{
  if(!initialized_){
    return;
  }

  ROS_INFO_ONCE("Received broadcast data");
  DJI::onboardSDK::BroadcastData data;
  dji_comm_.getBroadcastData(&data);

  processIMU(data);
  processRc(data);
  processGPS(data);
  processStatusInfo(data);
  processTimeStamp(data);

  updateControlMode(data);

  ros::WallTime t2 = ros::WallTime::now();

}

bool DJIInterface::checkNewData(FlightDataType data_type, const unsigned short msg_flag)
{
  bool version_A3 = firmware_version_ == DJI::onboardSDK::versionA3_31
      || firmware_version_ == DJI::onboardSDK::versionA3_32;

  bool version_M100 = firmware_version_ == DJI::onboardSDK::versionM100_31
      || firmware_version_ == DJI::onboardSDK::versionM100_23;

  if (version_A3) {
    switch (data_type) {
      case (FlightDataType::TimeStamp): {
        if (flight_data_mask_A3_.kTimeStampMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::IMU): {
        if (flight_data_mask_A3_.kIMUMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::RCData): {
        if (flight_data_mask_A3_.kRcMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::GPSLocation): {
        if (flight_data_mask_A3_.kGPSMask & msg_flag) {
          return true;
        }
        break;
      }
      case (FlightDataType::Status): {
        if (flight_data_mask_A3_.kStatusMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
    }
  }  // end A3

  if (version_M100) {
    switch (data_type) {
      case (FlightDataType::TimeStamp): {
        if (flight_data_mask_M100_.kTimeStampMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::IMU): {
        if (flight_data_mask_M100_.kIMUMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::RCData): {
        if (flight_data_mask_M100_.kRcMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::GPSLocation): {
        if (flight_data_mask_M100_.kGPSMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
      case (FlightDataType::Status): {
        if (flight_data_mask_M100_.kStatusMask & msg_flag) {
          return true;
        } else {
          return false;
        }
        break;
      }
    }
  }  // end M100
  return false;
}

//process data
void DJIInterface::processIMU(const DJI::onboardSDK::BroadcastData& data)
{

  if (!checkNewData(FlightDataType::IMU, data.dataFlag)) {
    // no new imu data
    return;
  }

  // new IMU msg
  sensor_msgs::Imu msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();  //todo(fmina) time sync

  //transform attitude from NED to ENU
  Eigen::Quaterniond q_NED(data.q.q0, data.q.q1, data.q.q2, data.q.q3);
  Eigen::Quaterniond q_ENU = Eigen::Quaterniond(0, 1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0) * q_NED
      * Eigen::Quaterniond(0, 1, 0, 0);

  msg.orientation.w = q_ENU.w();
  msg.orientation.x = q_ENU.x();
  msg.orientation.y = q_ENU.y();
  msg.orientation.z = q_ENU.z();

  msg.orientation_covariance.fill(0);
  msg.orientation_covariance[0] = -1;

  msg.angular_velocity.x = data.w.x;
  msg.angular_velocity.y = -data.w.y;
  msg.angular_velocity.z = -data.w.z;

  msg.angular_velocity_covariance.fill(0);
  msg.angular_velocity_covariance[0] = kAngularVelocityNoiseVariance;
  msg.angular_velocity_covariance[4] = kAngularVelocityNoiseVariance;
  msg.angular_velocity_covariance[8] = kAngularVelocityNoiseVariance;

  msg.linear_acceleration.x = data.a.x * kGravity;
  msg.linear_acceleration.y = -data.a.y * kGravity;
  msg.linear_acceleration.z = -data.a.z * kGravity;

  msg.linear_acceleration_covariance.fill(0);
  msg.linear_acceleration_covariance[0] = kLinearAccelerationNoiseVariance;
  msg.linear_acceleration_covariance[4] = kLinearAccelerationNoiseVariance;
  msg.linear_acceleration_covariance[8] = kLinearAccelerationNoiseVariance;

  imu_pub_.publish(msg);

}

void DJIInterface::processRc(const DJI::onboardSDK::BroadcastData& data)
{
  if (!checkNewData(FlightDataType::RCData, data.dataFlag)) {
    // no new RC data
    return;
  }

  sensor_msgs::Joy msg;

  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();  //todo(fmina) time sync

  msg.axes.resize(8);
  // axis 0 is pitch
  msg.axes[0] = data.rc.pitch / kRCStickMaxValue;
  // axis 1 is roll
  msg.axes[1] = -data.rc.roll / kRCStickMaxValue;
  // axis 2 is thrust
  msg.axes[2] = data.rc.throttle / kRCStickMaxValue;
  //axis 3 is yaw
  msg.axes[3] = -data.rc.yaw / kRCStickMaxValue;
  //axis 4 is enable/disable external commands
  if (data.rc.gear < int(-kRCStickMaxValue/2)) {
    msg.axes[4] = 1;
  } else {
    msg.axes[4] = -1;
  }
  //axis 5 is mode
  if (data.rc.mode == 8000) {
    msg.axes[5] = 1;  // F mode
  } else if (data.rc.mode == 0) {
    msg.axes[5] = 0;  // A mode
  } else if (data.rc.mode == -8000) {
    msg.axes[5] = -1;  // P mode
  }

  msg.axes[6] = 0;
  msg.axes[7] = 0;

  msg.buttons.resize(1);
  if (data.rc.gear != 0) {
    msg.buttons[0] = 1;  // rc is on
  } else {
    msg.buttons[0] = 0;
  }

  rc_pub_.publish(msg);
}
void DJIInterface::processTimeStamp(const DJI::onboardSDK::BroadcastData& data)
{
  if (!checkNewData(FlightDataType::TimeStamp, data.dataFlag)) {
    // no new timestamp data
    return;
  }

}

void DJIInterface::processGPS(const DJI::onboardSDK::BroadcastData& data)
{
  if (!checkNewData(FlightDataType::GPSLocation, data.dataFlag)) {
    // no new timestamp data
    return;
  }
  sensor_msgs::NavSatFix gps_msg;
  // Update gps mesage
  gps_msg.header.frame_id = "/world";
  gps_msg.header.stamp = ros::Time::now();  //todo(fmina) time sync
  gps_msg.latitude = data.pos.latitude * 180.0 / C_PI;
  gps_msg.longitude = data.pos.longitude * 180.0 / C_PI;
  gps_msg.altitude = data.pos.height; //data.pos.altitude;
  gps_pub_.publish(gps_msg);

}

void DJIInterface::processStatusInfo(const DJI::onboardSDK::BroadcastData& data)
{
  if (!checkNewData(FlightDataType::Status, data.dataFlag)) {
    // no new battery info data
    return;
  }

  mav_msgs::Status msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  msg.battery_voltage = data.battery;

  //todo get more flight status data

  status_pub_.publish(msg);
}

void DJIInterface::updateControlMode(const DJI::onboardSDK::BroadcastData& data)
{

  //need control status and RC data
  if (!checkNewData(FlightDataType::Status, data.dataFlag)) {
    return;
  }
  if (!checkNewData(FlightDataType::RCData, data.dataFlag)) {
    return;
  }
  //if RC is on F mode and serial is enabled, external control should be enabled
  bool rc_mode_F = data.rc.mode == 8000;
  bool rc_serial_enabled = data.rc.gear < int(-kRCStickMaxValue/2);
  bool external_control_mode = data.ctrlInfo.deviceStatus == DJI::onboardSDK::Flight::DEVICE_SDK;

  if(rc_mode_F){
    if (!external_control_mode) {
      dji_comm_.setExternalControl(true);
    }
  }
}

int DJIInterface::getFrequencyValue(int freq_hz)
{
switch (freq_hz) {
  case (0):
    return 0;
  case (1):
    return 1;
  case (10):
    return 2;
  case (50):
    return 3;
  case (100):
    return 4;
  default:
    ROS_WARN_STREAM(kScreenPrefix + "Unacceptable frequency value. Acceptable values are 0, 1, 10, 50, 100 Hz.");
    return 5;
}
}

} /* namespace mav_disturbance_observer */
