### EtherCAT Control Software
  Welcome to EtherCAT user space application by Veysi ADIN & Chunwoo Kim.This repository contains EtherCAT based control software using CoE and CiA402 standard to control motors and receive sensor data, by wrapping IgH EtherCAT library functions. 
  This implementation can be used with any robotic systems supporting EtherCAT protocol with small modifications. It contains EtherCAT real-time thread with priority of 98. And USB communication with Xbox Controller. You can use different type of user input device as well with small modifications.
  
 Please check prerequisites, guides, links and documentations before installation, or using this control software. To be able to build this control software you will need to install IgH EtherCAT library, and if you need real-time performance you will need to install RT_PREEMPTH patch or Xenomai kernel. You can follow links below for the installation of required library and RT_PREEMPT installation.

## Prerequisites
- [RT_Preempt Linux and IgH EtherCAT Implementation](https://github.com/veysiadn/IgHEtherCATImplementation)
- If you want to use [Xenomai-Installation](https://github.com/veysiadn/xenomai-install)

Once you install the prerequisites you're ready to build the control software.

## Implementation
  
```sh
mkdir project-ws 
cd project-ws
git clone https://github.com/veysiadn/ecat_userspace ethercat-control
cd ethercat-control
cmake .
make
```
If there is no error during the compilation process you are ready to run the executable.
```
sudo ./ecat_node
```
If it is running your implementation is succesful, and now you are ready to customize the software based on your application.
By default number of connected slaves are defined as one, therefore if you don't have any slave to connect to your Ethernet port software will raise an error about the situation, but don't worry you can customize the software based on your needs.
## Getting Started
## STEP 1 : 
  You should start your customization from ecat_globals.hpp file. In that file you can specify : 
  - Number of connected slaves and servo drives,
  - Control operation mode : Velocity Mode, Position Mode, Torque Mode, Cyclic Synchronous Velocity,Position and Torque modes are supported. 
  - Control Frequency
  - Enable/Disable Distributed Clock
  - Motors encoder resolution : note that if you are using different type of motors you might need to create different definitions for each motor.
  - Motor Gear Ratio :  note that if you are using different type of motors you might need to create different definitions for each motor.
  - Custom Slave : If you have different slave than the CiA402 supported servo drive you will need to define custom slave and PDO mapping for that custom slave.
  - Keep in mind that this software addresses connected slaves based on physical position. For example the 0th slave will be the first slave that is connected to your Ethernet port. 
  - Custom slave must be in the end of slave chain.
## STEP 2 : 
  Once you did your initial configuration in the ecat_globals.hpp file. You can modify user input method in main.cpp file.
  - Currently this software uses XboxController left and right axis to send control commands the connected motors, if you want to use different input you can remove Xbox related control parameters and add your own.
  - If you want to use Xbox Controller for testing keep in mind that each axis in controller generated data in the range of -32768 ~ 32768.
## STEP 3 : 
  - Change configuration parameters for your motor in ecat_node.cpp file based on your operation mode. For example if you want to use velocity mode you can change parameters in the SetProfileVelocityParametersAll(ProfileVelocityParam& P) function based on your needs.
  - Change control parameters based on your selected operation mode in ecat_lifecycle.cpp file in the Update functions. For example if you want to use velocity mode in your control loop, you can change function content of UpdateVelocityModeParameters();

Once you did those changes you will need recompile the software using make and you can test the executable. 

## NOTE 
  - This software heavily tested on Maxon EPOS drivers, therefore if you want to use different servo driver you will need to check PDO mapping of your slave, or you can do custom PDO mapping by using MapCustomPdo function defined in ecat_node.cpp file. 
## Guides

- [EtherCAT](https://www.ethercat.org/en/technology.html)
- [Etherlab Webpage](https://www.etherlab.org/en/ethercat/index.php)
- [IgH EtherCAT Library Documentation](https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf)
- [Real-time Linux](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/start)
- [Real-time Background](https://design.ros2.org/articles/realtime_background.html)
- [Article on EtherCAT-RT PREEMPT- Xenomai](https://www.ripublication.com/ijaer17/ijaerv12n21_94.pdf)

#### Good Luck ⚡
