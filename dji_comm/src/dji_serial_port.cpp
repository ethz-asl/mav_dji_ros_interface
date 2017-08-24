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
#include <dji_comm/dji_serial_port.h>

namespace dji_comm {

DJISerialPort::DJISerialPort(std::string device, unsigned int baudrate)
    : device_(device),
      baudrate_(baudrate)
{
  memLock_ = PTHREAD_MUTEX_INITIALIZER;
  msgLock_ = PTHREAD_MUTEX_INITIALIZER;
  ackLock_ = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_init(&ack_recv_cv_, NULL);
  device_status_ = false;
}

DJISerialPort::~DJISerialPort()
{
  closeSerial();
  pthread_mutex_destroy(&memLock_);
  pthread_mutex_destroy(&msgLock_);
  pthread_mutex_destroy(&ackLock_);
  pthread_cond_destroy(&ack_recv_cv_);
}

void DJISerialPort::init()
{
//  API_LOG(this, STATUS_LOG, "Open serial device %s with baudrate %u...\n", device_.c_str(), baudrate_);
  if (startSerial(device_.c_str(), baudrate_) < 0) {
    closeSerial();
    API_LOG(this, ERROR_LOG, "Failed to start serial device\n");
    device_status_ = false;
  }
  device_status_ = true;
}

void DJISerialPort::usbHandshake(std::string device)
{
  startSerial(device.c_str(), 38400);
  startSerial(device.c_str(), 19200);
  startSerial(device.c_str(), 38400);
  startSerial(device.c_str(), 19200);
}

void DJISerialPort::setBaudrate(unsigned int baudrate)
{
  baudrate_ = baudrate;
}

void DJISerialPort::setDevice(std::string device)
{
  device_ = device;
}

bool DJISerialPort::getDevieStatus()
{
  return device_status_;
}

uint64_t DJISerialPort::getTimeStamp()
{
#ifdef __MACH__
  struct timeval now;
  gettimeofday(&now, NULL);
  return (uint64_t) now.tv_sec * 1000 + (uint64_t) (now.tv_usec / 1.0e3);
#else
  struct timespec time;
  clock_gettime(CLOCK_REALTIME, &time);
  return (uint64_t)time.tv_sec * 1000 + (uint64_t)(time.tv_nsec / 1.0e6);
#endif
}

void DJISerialPort::wait(int timeout_ms)
{
  struct timespec absTimeout;
#ifdef __MACH__
  struct timeval curTime;
  gettimeofday(&curTime, NULL);
  absTimeout.tv_sec = curTime.tv_sec; //+ (curTime.tv_usec*1000 + timeout_ms*1e6)%1e9;
  absTimeout.tv_nsec = curTime.tv_usec*1000 + timeout_ms*1e6; // - 1e9*(curTime.tv_usec*1000 + timeout_ms*1e6)%1e9;
#else
  struct timespec curTime;
  clock_gettime(CLOCK_REALTIME, &curTime);
  absTimeout.tv_sec = curTime.tv_sec;
  absTimeout.tv_nsec = curTime.tv_nsec + timeout_ms*1e6;

#endif
  pthread_cond_timedwait(&ack_recv_cv_, &ackLock_, &absTimeout);
}

size_t DJISerialPort::send(const uint8_t *buf, size_t len)
{
  return writeSerial(buf, len);
}

size_t DJISerialPort::readall(uint8_t *buf, size_t maxlen)
{
  return readSerial(buf, maxlen);
}

void DJISerialPort::lockMemory()
{
  pthread_mutex_lock(&memLock_);
}

void DJISerialPort::freeMemory()
{
  pthread_mutex_unlock(&memLock_);
}

void DJISerialPort::lockMSG()
{
  pthread_mutex_lock(&msgLock_);
}

void DJISerialPort::freeMSG()
{
  pthread_mutex_unlock(&msgLock_);
}

void DJISerialPort::lockACK()
{
  pthread_mutex_lock(&ackLock_);
}

void DJISerialPort::freeACK()
{
  pthread_mutex_unlock(&ackLock_);
}

void DJISerialPort::notify()
{
  pthread_cond_signal(&ack_recv_cv_);
}

// serial port low level stuff

bool DJISerialPort::openSerial(const char* dev)
{
  serial_fd_ = open(dev, O_RDWR | O_NONBLOCK);
  if (serial_fd_ < 0) {
    API_LOG(this, ERROR_LOG, "Failed to open serial device %s\n", dev);
    return false;
  }
  return true;
}

bool DJISerialPort::closeSerial()
{
  close(serial_fd_);
  serial_fd_ = -1;
  return true;
}

bool DJISerialPort::flushSerial()
{
  if (serial_fd_ < 0) {
    API_LOG(this, ERROR_LOG, "flushing fail because no device is opened\n");
    return false;
  } else {
    tcflush(serial_fd_, TCIFLUSH);
    return true;
  }
}

bool DJISerialPort::configSerial(int baudrate, char data_bits, char parity_bits, char stop_bits)
{
  int st_baud[] = { B4800, B9600, B19200, B38400, B57600, B115200, B230400, B921600 };
  int std_rate[] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 921600 };

  int i, j;
  struct termios newtio, oldtio;
  /* save current port parameter */
  if (tcgetattr(serial_fd_, &oldtio) != 0) {
    API_LOG(this, ERROR_LOG, "fail to save current port\n");
    return false;
  }
  memset(&newtio, 0, sizeof(newtio));

  /* config the size of char */
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  /* config data bit */
  switch (data_bits) {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }
  /* config the parity bit */
  switch (parity_bits) {
    /* odd */
    case 'O':
    case 'o':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      break;
      /* even */
    case 'E':
    case 'e':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      break;
      /* none */
    case 'N':
    case 'n':
      newtio.c_cflag &= ~PARENB;
      break;
  }
  /* config baudrate */
  j = sizeof(std_rate) / 4;
  for (i = 0; i < j; ++i) {
    if (std_rate[i] == baudrate) {
      /* set standard baudrate */
      cfsetispeed(&newtio, st_baud[i]);
      cfsetospeed(&newtio, st_baud[i]);
      break;
    }
  }
  /* config stop bit */
  if (stop_bits == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (stop_bits == 2)
    newtio.c_cflag |= CSTOPB;

  /* config waiting time & min number of char */
  newtio.c_cc[VTIME] = 1;
  newtio.c_cc[VMIN] = 18; // from osdk

  /* using the raw data mode */
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_oflag &= ~OPOST;

  /* flush the hardware fifo */
  tcflush(serial_fd_, TCIFLUSH);

  /* activite the configuration */
  if ((tcsetattr(serial_fd_, TCSANOW, &newtio)) != 0) {
    API_LOG(this, ERROR_LOG, "fail to active configuration\n");
    return false;
  }
  return true;
}

int DJISerialPort::startSerial(const char *dev_name, int baud_rate)
{
  const char *ptemp;
  if (dev_name == NULL) {
    ptemp = "/dev/ttyUSB0";
  } else {
    ptemp = dev_name;
  }
  if (true == openSerial(ptemp) && true == configSerial(baud_rate, 8, 'N', 1)) {

    FD_ZERO(&serial_fd_set_);
    FD_SET(serial_fd_, &serial_fd_set_);
    return serial_fd_;

  }
  return -1;
}

int DJISerialPort::writeSerial(const unsigned char *buf, int len)
{
  return write(serial_fd_, buf, len);
}

int DJISerialPort::readSerial(unsigned char *buf, int len)
{
  int saved = 0;
  int ret = -1;

  if (NULL == buf) {
    return -1;
  } else {
    for (; saved < len;) {
      ret = read(serial_fd_, buf + saved, len - saved);
      if (ret > 0)
        saved += ret;
      else
        break;
    }
    return saved;
  }
}

} /* namespace dji_comm */
