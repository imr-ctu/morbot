#include <termios.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <cstring>
#include <cerrno>

class SerialPort
{
  public:

    SerialPort();
    ~SerialPort();

    bool openSerial(const char* port);
    void closeSerial(); 
    bool isOpened();  

    int writeSerial(unsigned char msg);
    int writeSerialInt(int16_t val);
    int writeSerialFloat(float f);
    int serialAvailable();

    bool serialFlush();
    bool readSerial(unsigned char* c);
    bool readSerialInt(int16_t* val);
    bool readSerialFloat(float* f);

  private:

    bool setSerialParams();
    void resetSerialParams();

    int fd_; 
    termios saved_attributes_;
};


