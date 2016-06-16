/**
 * \author Alexander Entinger, MSc / LXRobotics
 */

#ifndef HIGHPOWER_MOTORSHIELD_H_
#define HIGHPOWER_MOTORSHIELD_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

typedef enum 
{
  FWD = 0, BWD = 1
} EDirection;

/**************************************************************************************
 * CLASS DEFINITION
 **************************************************************************************/

class HighpowerMotorshield 
{

public:

  static void begin();

  static void setPwm(uint8_t const pwm);

  static void setDirection(EDirection const dir);

};

#endif /* HIGHPOWER_MOTORSHIELD_H_ */
