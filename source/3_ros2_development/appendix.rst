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

Using the `check_cross_build_versions.sh <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/common_utils/-/blob/main/cross_build/yocto_sdk/check_cross_build_versions.sh?ref_type=heads>`_ script to check for version mismatches between SDK and target.

.. note::

    Make sure to update the ``PROJECT_ROOT`` variable in the script to point to your ROS2 workspace ``src`` directory before running it.

If the script reports any mismatches, you may need to take one of the following actions:

- Try to run the application on the target system and see if it works despite the version mismatches.
- Rebuild the Yocto SDK with the correct versions of ROS2 packages that match those on the target system.
- Manual update the ROS2 packages on the SDK sysroot to match the target system versions. Refer to the next section for detailed steps.

.. _update_ros2_packages_sdk:

How to update/add the Yocto SDK with the correct ROS2 package versions with eSDK?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The procedure below outlines the steps to update or add ROS2 packages in the Yocto SDK sysroot to resolve ABI mismatch issues or missing packages.

If you plan to update the version of an existing ROS2 package, please follow the steps under the **Optional** section. Otherwise, you can skip to the main steps.

.. note::

    This section assumes that you have already set up the Yocto eSDK for RZ/V2H RDK as per the instructions in the :ref:`RZ/V2H eSDK Setup <esdk_setup>` section.

- Open **a new terminal** and source the Yocto eSDK environment:

  .. code-block:: bash

     $ source ~/poky_sdk/environment-setup-cortexa55-poky-linux

  Please replace the path with the actual path to your eSDK working folder.

- **Optional:** If you want to update version of an existing ROS2 package in the SDK sysroot, please edit the correct recipe file in the ``~/poky_sdk/layers/meta-ros/meta-ros2-jazzy``.

  For example, to update the ``pinocchio`` package:

  1. Find the corresponding recipe file located at: ``~/poky_sdk/layers/meta-ros/meta-ros2-jazzy``

  .. code-block:: bash
     :emphasize-lines: 1,3,7

      ubuntu@ros-xbuild:~/poky_sdk/layers/meta-ros/meta-ros2-jazzy$ find . -name "*pinocchio*"
      ./generated-recipes/pinocchio
      ./generated-recipes/pinocchio/pinocchio_3.6.0-1.bb
      ./generated-recipes/kinematics-interface-pinocchio
      ./generated-recipes/kinematics-interface-pinocchio/kinematics-interface-pinocchio_0.0.1-1.bb
      ./recipes-bbappends/pinocchio
      ./recipes-bbappends/pinocchio/pinocchio_%.bbappend
      ubuntu@ros-xbuild:~/poky_sdk/layers/meta-ros/meta-ros2-jazzy$

  2. Edit the recipe file.

  For example, to update to version ``3.7.0``, modify the relevant fields:

  - Find the correct new ``SRCREV`` for newer version available at the package's official repository.

  .. tip::

      You can find the correct ``SRCREV`` by visiting the package's release page which is ``SRC_URI`` combine with ``ROS_BRANCH``.

      The ``SRCREV`` is usually the commit hash or tag corresponding to the desired version.

      For example, for ``pinocchio``, you can check its GitHub releases for ROS2 Jazzy: `Pinocchio Releases Jazzy <https://github.com/ros2-gbp/pinocchio-release/tree/release/jazzy/pinocchio>`_.

  - Update the ``SRCREV`` field in the recipe file - ``pinocchio_3.6.0-1.bb`` accordingly.

  - Rename the recipe file to reflect the new version, e.g., ``pinocchio_3.7.0-1.bb``. Also, update the name of bbappend files if necessary.

- Build the package.

.. code-block:: bash

    $ devtool modify pinocchio
    $ devtool build pinocchio

- Install the package into the SDK sysroot:

.. code-block:: bash

    $ cd ~/poky_sdk/workspace/sources/pinocchio/oe-workdir/image/opt/ros/jazzy
    $ sudo cp -r * /opt/poky/5.1.4/sysroots/cortexa55-poky-linux/opt/ros/jazzy/

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