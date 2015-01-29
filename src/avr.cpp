/** The AVR class is responsible of the communication with the robot board.
 */

#include <morbot/avr.h>

#define TIMEOUT 10

/* Pulse count per meter */
const double M_TICKS = 1164.0; 
//const double RAD_TICKS = 250.8;  //ticks per rad

AVR::AVR()
{
}

AVR::~AVR()
{
  sp.closeSerial();
}

/** Open the serial port and set parameters.
 */
void AVR::openSerial(const char* port)
{
  sp.openSerial(port);
  sp.serialFlush();
}

/** Turn on the motors.
 */
void AVR::setMotorsOn()
{
  sp.writeSerial(0xA5);
}

/** Turn off motors.
 */
void AVR::setMotorsOff()
{
  sp.writeSerial(0x99);
}

/*
 * Nastavení konstanty pro pøevod pùlsù na radiány.
 */
void AVR::setConstant(float c)
{
  sp.writeSerial('c');
  sp.writeSerialFloat(c);
}

/** Set up the robot linear and angular velocities.
 */
void AVR::setSpeedAndTurn(signed char speed, signed char turn)
{
  sp.writeSerial(0x96);
  sp.writeSerial((unsigned char)speed);
  usleep(TIMEOUT);
  sp.writeSerial((unsigned char)turn);
  sp.writeSerial(0x96);
}

/** Set the robot position.
 */
void AVR::setPosition(float posX, float posY, float posYaw)
{
  posX *= M_TICKS;
  posY *= M_TICKS;
  /* Normalizace rozsahu souøadnic. */
  if (posY > 30000)
  {
    posY = 30000;
  }	
  else if (posY < -30000)
  {
    posY = -30000;
  }
  while (posYaw > M_PI)
  {
    posYaw -= 2 * M_PI;
  }
  while (posYaw < -M_PI)
  { 
    posYaw += 2 * M_PI;
  }

  sp.writeSerial(0xAA);
  sp.writeSerialInt((int16_t)posX);
  sp.writeSerialInt((int16_t)posY);
  sp.writeSerialInt((int16_t)(posYaw * 10000.0));
}

/** Get the robot position.
 *
 * @return True if everything went allright.
 */
bool AVR::getPosition(float* px, float* py, float* pa)
{
  sp.serialFlush();
  /* Request. */
  sp.writeSerial(0x55);

  int16_t x;
  int16_t y;
  int16_t a;

  sp.readSerialInt(&x);
  sp.readSerialInt(&y);
  sp.readSerialInt(&a);

  /* Conversion from ticks to meters and radians. */
  *px = ((float)x) / M_TICKS;
  *py = ((float)y) / M_TICKS;
  *pa = ((float)a) / 10000.0;
  return true;
}

/** Get the four infrared sensor readings.
 */
void AVR::getIRs(float* sh1, float* sh2, float* sh3, float* sh4)
{
  sp.serialFlush();

  /* Request. */
  sp.writeSerial(0x69);

  *sh1 = (float)sp.readSerial();
  *sh2 = (float)sp.readSerial();
  *sh3 = (float)sp.readSerial();
  *sh4 = (float)sp.readSerial();
}

/** Get the two sonar readings (in meters).
 */
void AVR::getSonars(float* sonar1, float* sonar2)
{
  sp.serialFlush();

  /* Request. */
  sp.writeSerial(0x5A);

  /* Raw sonar readings are in centimeters. Convert to meters. */
  *sonar1 = (float(sp.readSerial()) / 100);
  *sonar2 = (float(sp.readSerial()) / 100);
}

/** Get the eight bumper readings.
 */
void AVR::getBumpers(uint8_t* bump1, uint8_t* bump2, uint8_t* bump3, uint8_t* bump4,
    uint8_t* bump5, uint8_t* bump6, uint8_t* bump7, uint8_t* bump8)
{
  sp.serialFlush();

  /* Request. */
  sp.writeSerial(0x66);

  unsigned char bumps = sp.readSerial();

  /* Front right bumper. */
  uint8_t masked = bumps & 0x80;
  if(masked > 0)
  {
    /* No obstacle. */
    *bump1 = 1;
  }
  else
  { 
    /* Obstacle detected. */
    *bump1 = 0;
  }

  /* Left front bumper. */
  masked = bumps & 0x40;
  if(masked > 0)
  {
    *bump2 = 1;
  }
  else
  {
    *bump2 = 0;
  }

  /* Right rear bumper. */
  masked = bumps & 0x20;
  if(masked > 0)
  {
    *bump3 = 1;
  }
  else
  {
    *bump3 = 0;
  }

  /* Mù¾e být po¾ito pro tlaèítko spou¹tìjící poèí¾ání pozice. */    
  masked = bumps & 0x10;
  if(masked > 0)
  {
    *bump4 = 1;
  }
  else
  {
    *bump4 = 0;
  }

  /* Rear bumper */
  masked = bumps & 0x08;
  if(masked > 0)
  {
    *bump5 = 1;
  }
  else
  {
    *bump5 = 0;
  }

  /* Left read bumper. */
  masked = bumps & 0x04;
  if(masked > 0)
  {
    *bump6 = 1;
  }
  else
  {
    *bump6 = 0;
  }	

  /* Right front bumper. */
  masked = bumps & 0x02;
  if(masked > 0)
  {
    *bump7 = 1;
  }
  else
  {
    *bump7 = 0;
  }

  /* Left front bumper. */
  masked = bumps & 0x01;
  if(masked > 0)
  {
    *bump8 = 1;
  }
  else
  {
    *bump8 = 0;
  }	
}

