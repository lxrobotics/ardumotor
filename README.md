# ardumotor
Control a direct current motor with the Arduino and the LXRobotics Highpower Motorshield.

## Interface

### Subscribed Topics
* **`/rpm`** ([ardumotor/RPM])

  Set desired RPM of the motor.
  
### Published Topics
*None*

### Services
*None*

### Parameters
* **`port`** / std::string / Default: **/dev/ttyACM0**

  The device node under which the Arduino registers itself with the Arduino Uno.

* **`baud`** / int / Default: **115200**

  The baud rate which is used for the communication between the PC and the Arduino Uno.

## Installation

### Dependencies
```
sudo apt-get install arduino ros-indigo-rosserial ros-indigo-rosserial-arduino
```

### Building
```
cd ~/catkin_ws/src
git clone https://github.com/lxrobotics/ardumotor.git
cd ..
catkin_make
```

### Download
* Set the port under which the Arduino is connected to the PC by modifying [firmware/CMakeLists]
```
PORT /dev/ttyUSB0
```
* Download to the Arduino
```
catkin_make ardumotor_firmware_ardumotor-upload
```

### Start
* Set the port under which the Arduino is connected to the PC by modifying [launch/ardumotor.launch]
```
<param name="port" value="/dev/ttyUSB0"/> 
```
* Start via roslaunch
```
roslaunch ardumotor ardumotor.launch
```

[ardumotor/RPM]: https://github.com/lxrobotics/ardumotor/blob/master/msg/RPM.msg
[firmware/CMakeLists]: https://github.com/lxrobotics/ardumotor/blob/master/firmware/CMakeLists.txt
[launch/ardumotor.launch]: https://github.com/lxrobotics/ardumotor/blob/master/launch/ardumotor.launch
