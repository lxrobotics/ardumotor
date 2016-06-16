/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Arduino.h>

#include <ros.h>

#include <ardumotor/RPM.h>

#include "HighpowerMotorshield.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void RPMCallbackFunction(const ardumotor::RPM &msg);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ros::NodeHandle node_handle;

ros::Subscriber<ardumotor::RPM> rpm_subscriber("/rpm", &RPMCallbackFunction);


/**************************************************************************************
 * ARDUINO FRAMEWORK FUNCTIONS
 **************************************************************************************/

void setup() 
{
  /* Setup the connection to the ROS */

  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();

  while(!node_handle.connected())
  {
    node_handle.spinOnce();
  }

  /* Initialize the motorshield library */

  HighpowerMotorshield::begin();

  /* Subscribe to the rpm topic */

  node_handle.subscribe(rpm_subscriber);
}

void loop() 
{
  node_handle.spinOnce();
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void RPMCallbackFunction(const ardumotor::RPM &msg)
{

}
