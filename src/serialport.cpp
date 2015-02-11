#include <morbot/serialport.h>

/* usleep after a write operation on serial port, in microsecond.
This is to let the microprocessor time to treat the value. */
#define SLEEP_AFTER_WRITE 100

SerialPort::SerialPort()
{
  fd_ = -1;
}

SerialPort::~SerialPort()
{
  if(isOpened())
    closeSerial();
}

/** Set up serial communication parameters
 *
 * @return True if everything went allright.
 */
bool SerialPort::setSerialParams()
{
  termios options;
  if (!isatty(fd_))
  {
    std::cerr << "Not a terminal.\n";
    return false;
  }

  /* Save old attributes. */    
  tcgetattr(fd_, &saved_attributes_);
  /* Set up new parameters. */
  tcgetattr(fd_, &options);
  /* Set up speed. */        
  cfsetispeed(&options, B57600);
  cfsetospeed(&options, B57600);
  /* No parity bit. */
  options.c_cflag &= ~PARENB;
  /* One stopbit. */
  options.c_cflag &= ~CSTOPB;
  /* Eight bits of data. */
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  /* Povolení pøijímání dat. */
  options.c_cflag |= (CLOCAL | CREAD);	
  /* Non canonical mode. */
  options.c_lflag &= ~ICANON;
  /* Nevypisovat pøijaté znaky. */
  options.c_lflag &= ~ECHO;
  options.c_lflag &= ~(ECHOE | ISIG);
  options.c_lflag &= ~(ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE | TOSTOP);
  options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
  options.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONLRET | OFILL | OFDEL);
  options.c_cc[VMIN] = 1;
  /* No timeout. */
  options.c_cc[VTIME] = 0;
  tcsetattr(fd_, TCSAFLUSH, &options);
  std::cout << "Serial port parameters set.\n";
  return true;
}

/** Reset to old parameters.
 */
void SerialPort::resetSerialParams()
{
  std::cout << "Serial port parameters reset.\n";
  tcsetattr(fd_, TCSAFLUSH, &saved_attributes_);
}

/** Open the serial port.
 *
 * @return True if everything went allright.
 */
bool SerialPort::openSerial(const char* port)
{
  fd_ = open(port, O_RDWR | O_NOCTTY);
  if(fd_ == -1)
  {
    std::cerr << "ERROR opening device " << port << std::endl;
    return false;
  }
  if (!setSerialParams())
  {
    return false;
  }
  std::cout << "Serial port " << port << " opened.\n";
  return true;
}

/** Return true if the port is open.
 */
bool SerialPort::isOpened()
{
  return (fd_ >= 0);
}	

/** Close the serial port.
 */
void SerialPort::closeSerial()
{
  if (fd_ < 0)
  {
    std::cout << "Port was not open, ignoring.\n";
    return;
  }
  resetSerialParams();
  close(fd_);
  fd_ = -1;
  std::cout << "Port closed.\n";
}

/** Write out a character on the serial port.
 *
 * @return the number of bytes written, or -1.
 */
int SerialPort::writeSerial(unsigned char msg)
{
  const int written = write(fd_, &msg, sizeof(msg));
  if(written < 0)
  {
    std::cerr << "ERROR writing on serial port:" << std::strerror(errno) << std::endl;
  }
  usleep(SLEEP_AFTER_WRITE);
  return written;
}

/** Write out an integer on the serial port.
 *
 * @return the number of bytes written, or -1.
 */
int SerialPort::writeSerialInt(int16_t val)
{
  /* Caution: not portable */
  const unsigned char *msg = (unsigned char*)&val;
  const int written1 = writeSerial(msg[1]);
  int written0 = -1;
  if (written1 != -1)
  {
    written0 = writeSerial(msg[0]);
  }
  if (written0 == -1)
  {
    return -1;
  }
  return written0 + written1;
}

/** Write out a float on the serial port.
 */
int SerialPort::writeSerialFloat(float f)
{
  /* Caution: not portable */
  unsigned char *msg = (unsigned char*)&f;
  int sum_written = 0;
  for (unsigned int i = 0; i < sizeof(float); i++)
  {
    const int written = writeSerial(msg[i]);
    if (written == -1)
    {
      sum_written = -1;
      return -1;
    }
    sum_written += written;
  }
  return sum_written;
}

/** Checkout the number of available characters on the serial port.
 *
 * @return The number of available characters, -1 on error.
 */
int SerialPort::serialAvailable()
{
  int available = 0;
  ioctl(fd_, FIONREAD, &available);
  return available;
}

/** Flush the serial port.
 *
 * @return True if everything went allright.
 */
bool SerialPort::serialFlush()
{
  while (serialAvailable() > 0)
  {
    char buf;
    const int nbytes = read(fd_, &buf, sizeof(buf));
    if (nbytes == -1)
    {
      std::cerr << "Serial port read error on flush: " << std::strerror(errno) << std::endl;
      return false;
    }
  }
  return true;
}

/** Read a character from the serial port.
 *
 * @return True if everything went allright (one character read).
 */
bool SerialPort::readSerial(unsigned char* c)
{
  // Initialize file descriptor sets
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  // Set timeout to 1.0 seconds
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // Wait for input to become ready or until the timeout; the first parameter is
  // 1 more than the largest file descriptor in any of the sets
  int select_ret = select(fd_ + 1, &read_fds, NULL, NULL, &timeout);
  if (select_ret == -1)
  {
    // error.
    std::cerr << "Serial port select error: " << std::strerror(errno) << std::endl;
    return false;
  }
  if (select_ret == 0)
  {
    // timeout.
    std::cerr << "Nothing to be read on serial port" << std::endl;
    return false;
  }
  // fd_ is ready for reading
  const int nbytes = read(fd_, c, sizeof(*c));
  if (nbytes == -1)
  {
    std::cerr << "Cannot read on serial port: " << std::strerror(errno) << std::endl;
    return false;
  }
  return  true;
}

/** Read an integer from the serial port.
 */
bool SerialPort::readSerialInt(int16_t* val)
{
  unsigned char* msg = (unsigned char*)val;
  if (!readSerial(msg + 1))
  {
    return false;
  }
  return readSerial(msg);
}

/** Read a float from the serial port.
 */
bool SerialPort::readSerialFloat(float* f)
{
  unsigned char* msg = (unsigned char*)f;
  for (unsigned int i = 0; i < sizeof(float); i++)
  {
    if (!readSerial(msg + i))
    {
      return false;
    }
  }
  return true;
}

