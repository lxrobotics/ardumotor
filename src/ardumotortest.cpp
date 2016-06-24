/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <ros/ros.h>

#include <ardumotor/RPM.h>

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void handleSetRPM      (ros::Publisher &rpm_publisher);
void handleExit        ();
void handleInvalidValue();

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ardumotortest");

  ros::NodeHandle node_handle;

  /* Create the publisher for sending RPM messages to the arduino */

  ros::Publisher rpm_publisher = node_handle.advertise<ardumotor::RPM>("/rpm", 10);

  /* Provide a crude menu for selecting which service one wants to invoke */

  char cmd = 0;

  do
  {
    std::cout << std::endl;
    std::cout << "set the [r]pm value of a channel" << std::endl;
    std::cout << "        [q]uit"                   << std::endl;
    std::cout << ">>"; std::cin >> cmd;

    switch(cmd)
    {
    case 'r': handleSetRPM      (rpm_publisher);  break;
    case 'q': handleExit        ();               break;
    default:  handleInvalidValue();               break;
    }
  } while(cmd != 'q');

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void handleSetRPM(ros::Publisher &rpm_publisher)
{
  std::cout << "Enter the RPM value: ";
  int rpm = 0;
  std::cin >> rpm;

  ardumotor::RPM msg;
  msg.rpm = rpm;

  rpm_publisher.publish(msg);
}

void handleExit()
{
  std::cout << "Exiting function ardumotortest" << std::endl;
}

void handleInvalidValue()
{
  std::cout << "Invalid input value" << std::endl;
}

