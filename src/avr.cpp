/** The AVR class is responsible of the communication with the robot board.
 */

#include <morbot/avr.h>

/* Pulse count per meter */
const double M_TICKS = 1164.0; 
//const double RAD_TICKS = 250.8;  //ticks per rad

AVR::AVR()
{
}

AVR::~AVR()
{
  sp_.closeSerial();
}

/** Open the serial port and set parameters.
 */
bool AVR::openSerial(const char* port)
{
  return sp_.openSerial(port) && sp_.serialFlush();
}

/** Turn on the motors.
 *
 * @return True if the message was sent correctly.
 */
bool AVR::setMotorsOn()
{
  if (sp_.writeSerial(0xA5) == -1)
  {
    return false;
  }
  return true;
}

/** Turn off motors.
 *
 * @return True if the message was sent correctly.
 */
bool AVR::setMotorsOff()
{
  if (sp_.writeSerial(0x99) == -1)
  {
    return false;
  }
  return true;
}

/** Configure the number of ticks / 2 rad motor turn.
 *
 * @return True if the message was sent correctly.
 */
bool AVR::setConstant(float c)
{
  if (sp_.writeSerial('c') == -1)
  {
    return false;
  }
  if (sp_.writeSerialFloat(c) != sizeof(float))
  {
    return false;
  }
  return true;
}

/** Set up the robot linear and angular velocities.
 *
 * @return True if the message was sent correctly.
 */
bool AVR::setSpeedAndTurn(signed char speed, signed char turn)
{
  if (sp_.writeSerial(0x96) == -1)
  {
    return false;
  }
  if (sp_.writeSerial((unsigned char)speed) == -1)
  {
    return false;
  }
  if (sp_.writeSerial((unsigned char)turn) == -1)
  {
    return false;
  }
  if (sp_.writeSerial(0x96) == -1)
  {
    return false;
  }
  return true;
}

/** Set the robot position.
 *
 * @return True if the message was sent correctly.
 */
bool AVR::setPosition(float posX, float posY, float posYaw)
{
  posX *= M_TICKS;
  posY *= M_TICKS;
  /* Normalization of coordinate ranges. */
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

  if (sp_.writeSerial(0xAA) == -1)
  {
    return false;
  }
  if (sp_.writeSerialInt((int16_t)posX) != 2)
  {
    return false;
  }
  ;
  if (sp_.writeSerialInt((int16_t)posY) != 2)
  {
    return false;
  }
  ;
  if (sp_.writeSerialInt((int16_t)(posYaw * 10000.0)) != 2)
  {
    return false;
  }
  return true;
}

/** Get the robot position.
 *
 * @return True if everything went allright.
 */
bool AVR::getPosition(float* px, float* py, float* pa)
{
  if (!sp_.serialFlush())
  {
    return false;
  }
  /* Request. */
  if (sp_.writeSerial(0x55) == -1)
  {
    return false;
  }

  int16_t x;
  int16_t y;
  int16_t a;

  if (!sp_.readSerialInt(&x))
  {
    std::cerr << "Error reading x\n";
    *px = 0.0;
    *py = 0.0;
    *pa = 0.0;
    return false;
  }
  if (!sp_.readSerialInt(&y))
  {
    std::cerr << "Error reading y\n";
    *px = 0.0;
    *py = 0.0;
    *pa = 0.0;
    return false;
  }
  if (!sp_.readSerialInt(&a))
  {
    std::cerr << "Error reading yaw\n";
    *px = 0.0;
    *py = 0.0;
    *pa = 0.0;
    return false;
  }

  /* Conversion from ticks to meters and radians. */
  *px = ((float)x) / M_TICKS;
  *py = ((float)y) / M_TICKS;
  *pa = ((float)a) / 10000.0;
  return true;
}

/** Get the four infrared sensor readings.
 *
 * @return True if everything went allright.
 */
bool AVR::getIRs(float* sh1, float* sh2, float* sh3, float* sh4)
{
  if (!sp_.serialFlush())
  {
    return false;
  }

  /* Request. */
  if (sp_.writeSerial(0x69) == -1)
  {
    return false;
  }

  unsigned char raw;
  if (!sp_.readSerial(&raw))
  {
    return false;
  }
  *sh1 = static_cast<float>(raw);

  if (!sp_.readSerial(&raw))
  {
    return false;
  }
  *sh2 = static_cast<float>(raw);

  if (!sp_.readSerial(&raw))
  {
    return false;
  }
  *sh3 = static_cast<float>(raw);

  if (!sp_.readSerial(&raw))
  {
    return false;
  }
  *sh4 = static_cast<float>(raw);

  return true;
}

/** Get the two sonar readings (in meters).
 *
 * @return True if everything went allright.
 */
bool AVR::getSonars(float* sonar1, float* sonar2)
{
  if (!sp_.serialFlush())
  {
    return false;
  }

  /* Request. */
  if (sp_.writeSerial(0x5A) == -1)
  {
    return false;
  }

  unsigned char raw;
  if (!sp_.readSerial(&raw))
  {
    return false;
  }
  /* Raw sonar readings are in centimeters. Convert to meters. */
  *sonar1 = static_cast<float>(raw) / 100.0;

  if (!sp_.readSerial(&raw))
  {
    return false;
  }
  *sonar2 = static_cast<float>(raw) / 100.0;
  return true;
}

/** Get the eight bumper readings.
 */
bool AVR::getBumpers(uint8_t* bump1, uint8_t* bump2, uint8_t* bump3, uint8_t* bump4,
    uint8_t* bump5, uint8_t* bump6, uint8_t* bump7, uint8_t* bump8)
{
  if (!sp_.serialFlush())
  {
    return false;
  }

  /* Request. */
  if (sp_.writeSerial(0x66) == -1)
  {
    return false;
  }

  unsigned char bumps;
  if (!sp_.readSerial(&bumps))
  {
    return false;
  }

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

  /* Left rear bumper. */
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
  return true;
}

