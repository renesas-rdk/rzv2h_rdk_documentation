Appendix
-----------------------------

.. _abi_mismatch:

What is ABI mismatch error?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ABI (Application Binary Interface) mismatch occurs when the compiled application expects certain library versions or system interfaces that differ from those available on the target system.

It can lead to runtime errors or crashes when the application is executed on the target device.

In the context of cross-building ROS2 applications for the Renesas RZ/V2H RDK platform by using the Yocto SDK, an ABI mismatch error may arise if there are discrepancies between the versions of ROS2 packages and libraries used during the cross-compilation process **(installed in the SDK sysroot)** and those present on the target RZ/V2H RDK board **(installed over apt repository)**.

Example output indicating ABI mismatch:

  .. code-block:: console

    [ros2_control_node-3] Stack trace (most recent call last):
    [ros2_control_node-3] #17   Object "/usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1", at 0xffffffffffffffff, in
    [ros2_control_node-3] #16   Object "/opt/ros/jazzy/lib/controller_manager/ros2_control_node", at 0xaaaad5ad5d6f, in _start
    [spawner-5] [INFO] [1756346496.296861080] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
    [ros2_control_node-3] #15   Source "../csu/libc-start.c", line 360, in __libc_start_main_impl [0xffff9b068597]
    [ros2_control_node-3] #14   Source "../sysdeps/nptl/libc_start_call_main.h", line 58, in __aarch64_ldadd4_relax [0xffff9b0684c3]
    [ros2_control_node-3] #13   Object "/opt/ros/jazzy/lib/controller_manager/ros2_control_node", at 0xaaaad5ad4e07, in main
    [ros2_control_node-3] #12   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0xffff9b6f342b, in rclcpp::executors::MultiThreadedExecutor::spin()
    [ros2_control_node-3] #11   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0xffff9b6f30b3, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
    [ros2_control_node-3] #10   Object "/opt/ros/jazzy/lib/librclcpp.so", at 0xffff9b6dd8a3, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
    ..............................
    [ros2_control_node-3] #0    Object "/opt/ros/jazzy/lib/libhardware_interface.so", at 0xffff9af7e26c, in
    [ros2_control_node-3] Bus error (Invalid address alignment [0x9])

How to avoid ABI mismatch error?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO: Add instructions on how to avoid ABI mismatch error.

- Using script to check for version mismatches between SDK and target.
- Update the SDK sysroot to match the target system libraries.

.. _common_ros2_workspace_structure:

Common ROS 2 topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. A typical ROS 2 workspace has the following structure:

.. code-block:: text

    ros2_ws/
    ├── src/                  # Source code for ROS 2 packages
    │   ├── package_1/
    │   ├── package_2/
    │   └── ...
    ├── build/                # Build output directory (generated)
    ├── install/              # Installation directory (generated)
    └── log/                  # Log files (generated)

2. Code style should follow the official documentation: `Code style and language versions <docs.ros.org/en/jazzy/Contributing/Code-Style-Language-Versions.html>`_.
3. Organizing Files and Folders Inside a ROS 2 Package: `Package Organization For a ROS Stack [Best Practices] <https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/>`_.