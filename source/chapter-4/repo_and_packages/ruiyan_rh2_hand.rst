Ruiyan RH2 Hand Packages
--------------------------

A collection of ROS 2 packages for controlling and interfacing with the Ruiyan RH2 robotic hand, including:

- ruiyan_rh2_controller: ROS2 package that provides the adaption of the controller for the RuiYan RH2 robotic hand for ARM64 target systems.
- ruiyan_rh2_dexhand: A ROS2 package that converts the ``sensor_msgs/JointState`` to ``rh6_cmd/Rh6Cmd`` as this message is required by the ``rh6_ctrl`` node for controlling the RuiYan RH2 Dexterous Hand.
- ruiyan_rh2_urdf: URDF package for the Ruiyan RH2 dexterous hand.

.. note::

    For cross-compilation by using Yocto SDK, please install ``ruiyan_rh2_controller/rh6_ctrl/lib/libRyhandArm64.so`` to the SDK sysroot using the following command:

    .. code-block:: bash

        $ sudo cp ruiyan_rh2_controller/rh6_ctrl/lib/libRyhandArm64.so $ROS2_SDK_SYSROOT/usr/lib/

    Before using the RuiYan RH2 Dexhand, ensure that the hand is properly initialized using the provided setup script located in the ``ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` or in the ``install/ruiyan_rh2_dexhand/share/ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` after installation.

1. Overview

This collection of packages provides comprehensive support for the Ruiyan RH2 robotic hand, including hardware interfaces, controllers, and robot descriptions. It enables users to easily set up and control the Ruiyan RH2 hand in ROS 2 environments.

For more details about each package, refer to their respective ``README.md files`` in the Ruiyan RH2 Hand packages.

2. License

Those packages are licensed under the Apache License 2.0.

3. CHANGELOG

- v1.0.0 (2025-31-10): Initial release of the collection of Ruiyan RH2 Hand packages.