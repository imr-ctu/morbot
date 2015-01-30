#include <termios.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdint.h>

class SerialPort
{
  private:

    int fd; 
    termios savedAttributes;
    void setSerialParams();
    void resetSerialParams();

  public:

    SerialPort();
    ~SerialPort();

    void openSerial(const char*);
    void closeSerial(); 
    bool isOpened();  
    int writeSerial(unsigned char);
    int writeSerialInt(int16_t);
    int writeSerialFloat(float);
    int serialAvailable();
    bool serialFlush();
    bool readSerial(unsigned char* c);
    bool readSerialInt(int16_t*);
    bool readSerialFloat(float*);
};


