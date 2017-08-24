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
#ifndef SRC_DJI_COMM_H_
#define SRC_DJI_COMM_H_

#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <functional>

#include <ros/ros.h>

#include <dji_sdk_lib/DJI_API.h>
#include <dji_sdk_lib/DJI_Flight.h>
#include <dji_sdk_lib/DJI_Camera.h>
#include <dji_sdk_lib/DJI_VirtualRC.h>
#include <dji_sdk_lib/DJI_WayPoint.h>
#include <dji_sdk_lib/DJI_HotPoint.h>
#include <dji_sdk_lib/DJI_Follow.h>

#include "dji_comm/dji_serial_port.h"

namespace dji_comm {

class DJIComm
{
 public:
  DJIComm(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~DJIComm();

  void init(std::string device, unsigned int baudrate);

  template<class T>
  void setBroadcastCallback(void (T::*func)(), T *obj)
  {
    broadcast_callback_ = std::bind(func, obj);
    core_api_ptr_->setBroadcastCallback(&DJIComm::broadcastCallback, this);
    printf("Broadcast call back received \n");
  }

  template<class T>
  void setFromMobileCallback(void (T::*func)(uint8_t *, uint8_t), T *obj)
  {
    mobile_callback_ = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
    core_api_ptr_->setFromMobileCallback(&DJIComm::fromMobileCallback, this);
  }

  template<class T>
  void setMissionStatusCallback(void (T::*func)(uint8_t *, uint8_t), T *obj)
  {
    mission_status_callback_ = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
    core_api_ptr_->setWayPointCallback(&DJIComm::missionStatusCallback, this);
  }

  template<class T>
  void setMissionEventCallback(void (T::*func)(uint8_t *, uint8_t), T *obj)
  {
    mission_event_callback_ = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
    core_api_ptr_->setWayPointEventCallback(&DJIComm::missionEventCallback, this);
  }

  void activate(DJI::onboardSDK::ActivateData *data, DJI::onboardSDK::CallBack callback);

  void getBroadcastData(DJI::onboardSDK::BroadcastData* data);
  void getFirmwareVersion(DJI::onboardSDK::Version* firmware_version);

  //enable external control
  void setExternalControl(bool enable);
  //set roll pitch yawrate thrust
  void setRollPitchYawrateThrust(double roll_cmd, double pitch_cmd, double yaw_rate, double thrust);
  //set broadcast freq
  void setBroadcastFrequency(uint8_t* freq);

 private:
  static constexpr int kSerialTimeout_ms = 100;
  //ros
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::shared_ptr<DJISerialPort> serial_;
  pthread_t communication_thread_;
//  pthread_t send_communication_thread_;


  // callbacks
  std::function<void()> broadcast_callback_;
  std::function<void(uint8_t*, uint8_t)> mobile_callback_;
  std::function<void(uint8_t*, uint8_t)> mission_status_callback_;
  std::function<void(uint8_t*, uint8_t)> mission_event_callback_;

  //api
  std::shared_ptr<DJI::onboardSDK::CoreAPI> core_api_ptr_;
  std::shared_ptr<DJI::onboardSDK::Flight> flight_ptr_;
  std::shared_ptr<DJI::onboardSDK::Camera> camera_ptr_;
  std::shared_ptr<DJI::onboardSDK::VirtualRC> virtual_rc_ptr_;
  std::shared_ptr<DJI::onboardSDK::WayPoint> waypoint_ptr_;
  std::shared_ptr<DJI::onboardSDK::HotPoint> hot_point_ptr_;
  std::shared_ptr<DJI::onboardSDK::Follow> follow_ptr_;

  static void broadcastCallback(DJI::onboardSDK::CoreAPI *coreAPI, DJI::onboardSDK::Header *header, void *userData);
  static void fromMobileCallback(DJI::onboardSDK::CoreAPI *coreAPI, DJI::onboardSDK::Header *header, void *userData);
  static void missionStatusCallback(DJI::onboardSDK::CoreAPI *coreAPI, DJI::onboardSDK::Header *header, void *userData);
  static void missionEventCallback(DJI::onboardSDK::CoreAPI *coreAPI, DJI::onboardSDK::Header *header, void *userData);

//  static void* mainReadCommunicationThread(void* core_api);
  static void* mainCommunicationThread(void* core_api);

};

} /* namespace dji_comm */

#endif /* SRC_DJI_COMM_H_ */
