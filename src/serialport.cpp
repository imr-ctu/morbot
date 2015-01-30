#include <morbot/serialport.h>

#define TIMEOUT 1000

SerialPort::SerialPort()
{
  fd = -1;
}

SerialPort::~SerialPort()
{
  if(isOpened())
    closeSerial();
}

/*
 * Nastavení sériového portu pro komunikaci
 * s øídicí deskou. 
 */
void SerialPort::setSerialParams()
{
  termios options;
  if (!isatty(fd))
  {
    std::cerr << "Not a terminal.\n";
    exit(EXIT_FAILURE);
  }

  /* Save old attributes. */    
  tcgetattr(fd, &savedAttributes);
  /* Set up new parameters. */
  tcgetattr(fd, &options);
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
  options.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSAFLUSH, &options);
  std::cout << "Serial port parameters set.\n";
}

/** Reset to old parameters.
 */
void SerialPort::resetSerialParams()
{
  std::cout << "Serial port parameters reset.\n";
  tcsetattr(fd, TCSAFLUSH, &savedAttributes);
}

/** Open the serial port.
 */
void SerialPort::openSerial(const char *port)
{
  fd = open(port, O_RDWR | O_NOCTTY);
  if(fd == -1)
  {
    std::cerr << "ERROR opening device " << port << std::endl;
    exit(EXIT_FAILURE);
  }
  else
  {
    setSerialParams();
    std::cout << "Serial port " << port << " opened.\n";
  }
}

/** Return true if the port is open.
 */
bool SerialPort::isOpened()
{
  return (fd >= 0);
}	

/** Close the serial port.
 */
void SerialPort::closeSerial()
{
  if (fd < 0)
  {
    std::cout << "Port was not open, ignoring.\n";
    return;
  }
  resetSerialParams();
  close(fd);
  fd = -1;
  std::cout << "Port closed.\n";
}

/** Write out a character on the serial port.
 *
 * @return the number of bytes written, or -1.
 */
int SerialPort::writeSerial(unsigned char msg)
{
  const int written = write(fd, &msg, sizeof(msg));
  if(written < 0)
  {
    std::cerr << "ERROR writing on serial port.";
  }
  usleep(TIMEOUT);
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
  ioctl(fd, FIONREAD, &available);
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
    const int nbytes = read(fd, NULL, sizeof(char));
    if (nbytes == -1)
    {
      return false;
    }
  }
  return true;
}

/** Read a character from the serial port.
 */
unsigned char SerialPort::readSerial()
{
  unsigned char response;
  const int nbytes = read(fd, &response, sizeof(response));
  if (nbytes == -1)
  {
    std::cerr << "Cannot read on serial port\n";
    return 0;
  }
  return response;
}

/** Read an integer from the serial port.
 */
void SerialPort::readSerialInt(int16_t* val)
{
  unsigned char *msg = (unsigned char*)val;
  msg[1] = readSerial();
  msg[0] = readSerial();
}

/** Read a float from the serial port.
 */
void SerialPort::readSerialFloat(float* f)
{
  unsigned char *msg = (unsigned char*)f;
  for (unsigned int i = 0; i < sizeof(float); i++)
  {
    msg[i] = readSerial();
  }
}

