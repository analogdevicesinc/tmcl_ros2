^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tmcl_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2025-07-31)
------------------
* Merge pull request `#9 <https://github.com/analogdevicesinc/tmcl_ros2/issues/9>`_ from villyjayt/humble
  Variable ap_index_bit_width updates + minor updates
* Variable ap_index_bit_width updates + minor updates
  NOTE: This commit is still compatible with the current list of supported TMCM boards.
  1. Adding variable ap_index_bit_width functionality
  2. Adding support for TMCM-1690
  3. Updating scripts
* Merge pull request `#4 <https://github.com/analogdevicesinc/tmcl_ros2/issues/4>`_ from analogdevicesinc/read_me_supported_modules
  Update README.md
* Update README.md
  Added supported modules in readME
* Contributors: Cacar, Christian Joseph Acar, Jamila Macagba, Vtolent3

2.0.3 (2024-11-13)
------------------
* Fix handling of incorrect parameter type and update header files to .hpp
* Contributors: Jamila Macagba

2.0.2 (2024-11-05)
------------------
* Added support for TMCM-1316
* Contributors: Jamila Macagba

2.0.1 (2024-07-15)
------------------
* Added support for TMCM-2611
  Added support for TMCM-2611
  Co-Authored-By: Christian Joseph Acar <124771470+CAcarADI@users.noreply.github.com>
  Co-Authored-By: Jamila Macagba <124771486+jmacagba@users.noreply.github.com>
* Update README.md
  Update README.md with mention of adi_tmcl and tmcl_ros2
  Co-Authored-By: Jamila Macagba <124771486+jmacagba@users.noreply.github.com>
* Contributors: mmaralit-adi

2.0.0 (2023-11-21)
------------------
* Updated package name and TF values
* Updated to:

  - Change package name from "tmcl_ros2" to "adi_tmcl" in preparation for ROS release
  - Change TF values (default)

* Contributors: mmaralit-adi, jmacagba

1.0.2 (2023-10-27)
------------------
* Update LICENSE file
* Contributors: Jamila Macagba

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
