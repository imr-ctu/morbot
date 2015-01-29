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

    void openSerial(const char* port);
    void setMotorsOn();
    void setMotorsOff();
    void setConstant(float);
    void setSpeedAndTurn(signed char speed, signed char turn);
    void setPosition(float posX, float posY, float posYaw);
    bool getPosition(float* px, float* py, float* pa);
    void getIRs(float* sh1, float* sh2, float* sh3, float* sh4);
    void getSonars(float* sonar1, float* sonar2);
    void getBumpers(uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*, uint8_t*);

  private:

    SerialPort sp;
};


