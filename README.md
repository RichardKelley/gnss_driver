GNSS
====

## Introduction
This is a port of the Apollo GNSS driver to pure ROS. We removed
dependencies on the protocol buffer messaging infrastructure used by
Apollo.

This driver has been tested to work with a Novatel ProPak 6 on the
University of Nevada, Reno's MKZ platform from AutonomouStuff.

This work was supported by the Nevada Governor's Office of Economic
Development and the State of Nevada's Knowledge Fund.

## Purpose
- Integrate Apollo's framework into the ROS framework through remove of protocol buffer dependencies for messages and instead use the ROS message framework to publish and subscribe to sensor topics. 
- Publish, subscribe, and log gnss data including gps, imu, ins, and wheel encoder.
- Stream RTK correction to the GPS receiver if available.

## Design
This GNSS driver has two nodelets: a data stream nodelet and a parser nodelet. The stream nodelet communicates between the host PC and the device and grabs the RTK data from a NTRIP caster. The parser nodelet subscribes the raw data from the stream nodelet, parses the data, and publishes ROS messages.

## Input
- GPS data, e.g. velocity latency, position and velocity and their standard deviations
- INS data, e.g. position, linear and angular velocity and covariance
- IMU pose data, e.g. orientation, velocity, & acceleration

## Output
- data stream status
- gnss status
- ins status
- imu data
- localization data

## Configuration
We use protocol buffers to store the driver configuration. The configuration file is stored in path `share/gnss_driver/conf/` which is quoted by gnss_driver.launch. File gnss_driver.launch is stored in path `share/gnss_driver/launch/`.
When using the gnss_driver, the follwing soudl be attended:
- To use the location with UTM projection, check the zone id configuration in gnss_driver.launch
- Check the the lever arm distance
- Confirm the imu install method, as this affects the vehicle's frame and orientation compute

