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
#ifndef SRC_DJI_INTERFACE_H_
#define SRC_DJI_INTERFACE_H_

#include <ros/ros.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Status.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Eigen>

#include <dji_comm/dji_comm.h>
#define C_PI (double)3.141592653589793
 
namespace dji_interface {

class DJIInterface
{
 public:
  DJIInterface(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~DJIInterface();

 private:
  static constexpr double kAngularVelocityNoiseVariance = 0.013 * 0.013;
  static constexpr double kLinearAccelerationNoiseVariance = 0.083 * 0.083;
  static constexpr double kGravity = 9.807;
  static constexpr double kRCStickMaxValue = 10000.0;
  static const std::string kScreenPrefix;
  static constexpr double kDefaultThrustCoefficient = 0.7389;
  static constexpr double kDefaultMinimumThrust = 10.0;
  static constexpr double kDefaultMaximumThrust = 100.0;



  enum FlightDataType
  {
    TimeStamp = 0,
    IMU,
    LinearVelocity,
    GPSLocation,
    RTK,
    Magnetometer,
    RCData,
    RPYGimbal,
    Status
  };

  //ros nh
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  //ros publishers
  ros::Publisher imu_pub_;
  ros::Publisher rc_pub_;
  ros::Publisher status_pub_;
  ros::Publisher gps_pub_;

  //ros subscribers
  ros::Subscriber command_roll_pitch_yawrate_thrust_sub_;

  //callbacks
  void commandRollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& msg);

  std::string frame_id_;
  std::string device_;
  int baudrate_;

  double thrust_coefficient_;
  double minimum_thrust_;
  double maximum_thrust_;
  double thrust_offset_;

  void loadParameters();
  void init();
  void setPublishers();
  void setSubscribers();

  bool initialized_;

  dji_comm::DJIComm dji_comm_;
  DJI::onboardSDK::ActivateData activation_data_;
  std::string drone_version_;

  void broadcastCallback();

  //process packets
  void processIMU(const DJI::onboardSDK::BroadcastData& data);
  void processRc(const DJI::onboardSDK::BroadcastData& data);
  void processTimeStamp(const DJI::onboardSDK::BroadcastData& data);
  void processStatusInfo(const DJI::onboardSDK::BroadcastData& data);
  void processGPS(const DJI::onboardSDK::BroadcastData& data);

  void updateControlMode(const DJI::onboardSDK::BroadcastData& data);

  DJI::onboardSDK::Version firmware_version_;
  //flight data masks
  typedef struct FlightDataMaskM100{
    static constexpr unsigned short kTimeStampMask = 0x0001;
    static constexpr unsigned short kIMUMask = 0x0016;
    static constexpr unsigned short kGPSMask = 0x0020;
    static constexpr unsigned short kMagnetometerMask = 0x0040;
    static constexpr unsigned short kRcMask = 0x0080;
    static constexpr unsigned short kRPYGimbalMask = 0x0100;
    static constexpr unsigned short kStatusMask = 0xE00;
  } FlightDataMaskM100_t;

  typedef struct FlightDataMaskA3{
    static constexpr unsigned short kTimeStampMask = 0x0001;
    static constexpr unsigned short kIMUMask = 0x0016;
    static constexpr unsigned short kGPSMask = 0x0040;
    static constexpr unsigned short kRTKMask = 0x0080;
    static constexpr unsigned short kMagnetometerMask = 0x0100;
    static constexpr unsigned short kRcMask = 0x0200;
    static constexpr unsigned short kRPYGimbalMask = 0x0400;
    static constexpr unsigned short kStatusMask = 0x3800;
  } FlightDataMaskA3_t;

  FlightDataMaskM100_t flight_data_mask_M100_;
  FlightDataMaskA3_t flight_data_mask_A3_;

  // helper methods
  bool checkNewData(FlightDataType data_type, const unsigned short msg_flag);

  static constexpr size_t kBroadcastFrequencySize = 16;
  std::vector<uint8_t> broadcast_frequency_;
  /*
  0 <- 0Hz
  1 <- 1Hz
  2 <- 10Hz
  3 <- 50Hz
  4 <- 100Hz
  5 <- anything else (don't change freq)
  */
  int getFrequencyValue(int freq_hz);

};

} /* namespace dji_interface */

#endif /* SRC_DJI_INTERFACE_H_ */
