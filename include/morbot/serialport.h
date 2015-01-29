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
    void writeSerial(unsigned char);
    void writeSerialInt(int16_t);
    void writeSerialFloat(float);
    int serialAvailable();
    void serialFlush();
    unsigned char readSerial();
    void readSerialInt(int16_t*);
    void readSerialFloat(float*);
};


