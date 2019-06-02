# ROS u-blox ZED-F9P Driver

## Description
This is a simple driver that converts the NMEA strings provided by the ZED-F9P
board to standard ROS messages. Currently, it provides lattitude, longitude,
altitude, heading, and speed (along with a diagonal positional
covariance matrix).

## Supported Sentences
Currently supported NMEA sentence types are:
* GGA
* GST
* VGT

_Support for more sentences may be added in the future_

## Dependencies
* ROS
* PySerial (Python Serial)

## Usage
1. Setup your GPS receiver to run on a serial port using uBlox config messages
or through u-Center. (Ensure that you enable supported/desired sentences.)
2. Use the config file (`config/gps.yaml`) to choose your port, baudrate, and
timeout. Your timeout should correspond roughly to the configured frequency.
3. Launch the node using `roslaunch ros_f9p_driver gps.launch` and the
driver will begin to output messages on the topics.
  * **/gps/fix [NavSatFix]** : Status, position, and covariance data
  * **/gps/speed [Float64]** : Speed in Km/h
  * **/gps/heading [Float64]** : Heading in degrees from True North
