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
#ifndef SRC_DJI_SERIAL_PORT_H_
#define SRC_DJI_SERIAL_PORT_H_

#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <dji_sdk_lib/DJI_Type.h>
#include <dji_sdk_lib/DJI_HardDriver.h>


namespace dji_comm {

#ifndef B921600
  #define B921600 921600
#endif

class DJISerialPort : public DJI::onboardSDK::HardDriver
{
 public:
  DJISerialPort(std::string device, unsigned int baudrate);
  ~DJISerialPort();
  void init();
  void usbHandshake(std::string device);
  void setBaudrate(unsigned int baudrate);
  void setDevice(std::string device);
  bool getDevieStatus();

  size_t send(const uint8_t *buf, size_t len);
  size_t readall(uint8_t *buf, size_t maxlen);
  void lockMemory();

  void freeMemory();

  void lockMSG();

  void freeMSG();
  void lockACK();

  void freeACK();
  void notify();
  void wait(int timeout_ms);

  uint64_t getTimeStamp();

 private:
  std::string device_;
  unsigned int baudrate_;
  pthread_mutex_t memLock_;

  pthread_mutex_t ackLock_;
  pthread_mutex_t msgLock_;
  pthread_cond_t ack_recv_cv_;

  int serial_fd_;
  fd_set serial_fd_set_;

  bool device_status_;

  bool openSerial(const char* dev);
  bool closeSerial();
  bool flushSerial();
  bool configSerial(int baudrate, char data_bits, char parity_bits, char stop_bits);
  int startSerial(const char *dev_name, int baud_rate);
  int writeSerial(const unsigned char *buf, int len);
  int readSerial(unsigned char *buf, int len);
};

} /* namespace dji_comm */

#endif /* SRC_DJI_SERIAL_PORT_H_ */
