^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tmcl_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-10-10)
------------------
* Removed obsolete modules and updated analog.com references in README
*  Updated to:

  - Remove obsolete modules
  - Update analog.com pages in README.md
  - Update CMakeLists.txt to make sure other drivers that has same names as socket_can_wrapper will not cause conflicts

* Contributors: mmaralit-adi, jmacagba

1.0.0 (2023-09-27)
------------------
* Adding v1.0.0 of tmcl_ros2, ROS2, Humble

  - Contains official ROS2 Driver for Trinamic Motor Controllers (TMC) that uses Trinamic Motion Control Language (TMCL) protocol
  - Supported TMC boards: TMCM-1636, TMCM-1617, TMCM-1241, TMCM-1260, TMCM-6214
  - Supported communication interface and interface driver: CAN (SocketCAN)
  - Supported ROS and OS distro: Humble (Ubuntu 22.04)
  - Supported platform: Intel x86 64-bit (amd64)

* Contributors: mmaralit-adi, jmacagba
