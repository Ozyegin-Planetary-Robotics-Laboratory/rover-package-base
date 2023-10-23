# Locator for OzU Rover

This program listens to the specified serial UART port, parses NMEA data, and publishes Latitude/Longitude information to the topic "gps".

## About

This code is based on an older C class I wrote for STM32 platform. The parser was designed to be memory-efficient and has been previously tested with GYNEO6 GPS.

Processing of NMEA messages is done char-by-char. The parser holds a state that resets when `$` character is received. For now, the program only parses `GPGLL` messages and ignores any other NMEA content.

## Requirements

* ROS Melodic+

## Building

`catkin_make`

If you only want to compile this package, then

`catkin_make --only-pkg-with-deps neo6m-ros`

## Usage
    
`rosrun neo6m-ros gps_interface PORT`

`PORT` is the serial port that GPS UART is linked to. Such as /dev/ttyTHS0
