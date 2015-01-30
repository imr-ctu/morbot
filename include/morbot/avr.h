#include <cmath>
#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <morbot/serialport.h>

class AVR
{
  public:

    AVR();
    ~AVR();

    bool openSerial(const char* port);
    bool setMotorsOn();
    bool setMotorsOff();
    bool setConstant(float);
    bool setSpeedAndTurn(signed char speed, signed char turn);
    bool setPosition(float posX, float posY, float posYaw);
    bool getPosition(float* px, float* py, float* pa);
    bool getIRs(float* sh1, float* sh2, float* sh3, float* sh4);
    bool getSonars(float* sonar1, float* sonar2);
    bool getBumpers(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*);

  private:

    SerialPort sp;
};


