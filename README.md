### Welcome to EtherCAT user space Application by Veysi ADIN & Chunwoo Kim.
 
  This repository contains EtherCAT based control software for using CoE and CiA402 standard to control motors and sensor information using EtherCAT protocol, by wrapping IgH    EtherCAT library functions. 
  This implementation can be used with any robotic system with small modifications.Contains EtherCAT real-time thread with priority of 98. And USB communication with 
  Xbox Controller.
  
  Please check guides, links and documentations before installation, or using this control software.

## Guides

- [EtherCAT](https://www.ethercat.org/en/technology.html)
- [Etherlab Webpage](https://www.etherlab.org/en/ethercat/index.php)
- [IgH EtherCAT Library Documentation](https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf)
- [Real-time Linux](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/start)
- [Real-time Background](https://design.ros2.org/articles/realtime_background.html)
- [Article on EtherCAT-RT PREEMPT- Xenomai](https://www.ripublication.com/ijaer17/ijaerv12n21_94.pdf)

## Prerequisites
- [RT_Preempt Linux and IgH EtherCAT Implementation](https://github.com/veysiadn/IgHEtherCATImplementation)
- If you want to use [Xenomai-Installation](https://github.com/veysiadn/xenomai-install)

## Implementation
  
```sh
mkdir project-ws 
cd project-ws
git clone https://github.com/veysiadn/ecat_userspace ethercat-control
cd ethercat-control
cmake .
make
sudo ./ecat_node
```
#### Keep in mind that you can change and tinker with the program, don't forget to make changes on Makefile for customization, additionally, you can check "compile.txt" file.

#### Good Luck ⚡
