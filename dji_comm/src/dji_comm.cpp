/*
 Copyright (c) 2016, Mina Kamel, ASL, ETH Zurich, Switzerland
 You can contact the author at <mina.kamel@mavt.ethz.ch>

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
#include "dji_comm/dji_comm.h"

namespace dji_comm {

DJIComm::DJIComm(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh)
{
}

DJIComm::~DJIComm()
{
}

void DJIComm::init(std::string device, unsigned int baudrate)
{
  serial_.reset(new DJISerialPort(device, baudrate));
  serial_->init();

  core_api_ptr_.reset(new DJI::onboardSDK::CoreAPI(serial_.get()));
  core_api_ptr_->setHotPointData(false);

  flight_ptr_.reset(new DJI::onboardSDK::Flight(core_api_ptr_.get()));
  camera_ptr_.reset(new DJI::onboardSDK::Camera(core_api_ptr_.get()));
  virtual_rc_ptr_.reset(new DJI::onboardSDK::VirtualRC(core_api_ptr_.get()));
  waypoint_ptr_.reset(new DJI::onboardSDK::WayPoint(core_api_ptr_.get()));
  hot_point_ptr_.reset(new DJI::onboardSDK::HotPoint(core_api_ptr_.get()));
  follow_ptr_.reset(new DJI::onboardSDK::Follow(core_api_ptr_.get()));

  int ret = pthread_create(&communication_thread_, 0, mainCommunicationThread, (void*) core_api_ptr_.get());
  if (0 != ret) {
    ROS_FATAL("Cannot create new thread for readPoll!");
  } else {
    ROS_INFO("Succeed to create thread for readPoll");
  }

  core_api_ptr_->getDroneVersion();
  ros::Duration(1.0).sleep();

  printf("==============\n");
  printf("DJI Comm initialized correctly.\n");
  printf("Hardware : %s\n", core_api_ptr_->getHwVersion());
  printf("Firmware : 0x0%X\n", core_api_ptr_->getFwVersion());
  printf("==============\n");
}

void DJIComm::activate(DJI::onboardSDK::ActivateData* data, DJI::onboardSDK::CallBack callback)
{
  core_api_ptr_->activate(data, callback);
}

void DJIComm::getBroadcastData(DJI::onboardSDK::BroadcastData* data)
{
  *data = core_api_ptr_->getBroadcastData();
}
void DJIComm::getFirmwareVersion(DJI::onboardSDK::Version* firmware_version)
{
  *firmware_version = core_api_ptr_->getFwVersion();
}

void DJIComm::broadcastCallback(DJI::onboardSDK::CoreAPI* coreAPI, DJI::onboardSDK::Header* header, void* userData)
{
  ((DJIComm*) userData)->broadcast_callback_();
}

void* DJIComm::mainCommunicationThread(void* core_api)
{
  DJI::onboardSDK::CoreAPI* p_coreAPI = (DJI::onboardSDK::CoreAPI*) core_api;
  while (ros::ok()) {
    p_coreAPI->readPoll();
    p_coreAPI->sendPoll();
    usleep(1000);
  }
}

void DJIComm::setExternalControl(bool enable)
{
  core_api_ptr_->setControl(enable);
}

void DJIComm::setRollPitchYawrateThrust(double roll_cmd, double pitch_cmd, double yaw_rate, double thrust)
{
  flight_ptr_->setMovementControl(0x2A, roll_cmd, pitch_cmd, thrust, yaw_rate);
}

void DJIComm::setBroadcastFrequency(uint8_t* freq)
{
  core_api_ptr_->setBroadcastFreq(freq);
}

} /* namespace dji_comm */
