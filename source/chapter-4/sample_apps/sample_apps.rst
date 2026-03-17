.. _sample_apps:

Sample Applications
-------------------

This section introduces sample ROS 2 applications developed for the Renesas RZ/V2H RDK platform, demonstrating various functionalities and use cases.

.. _sample_apps_prerequisites:

Prerequisites for Running Sample Applications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before running any sample application, ensure that you have completed the following common setup steps:

#. Set up the RZ/V2H RDK board as per :ref:`RZ/V2H RDK board setup <quick_setup_rdk_guide>`.

#. Complete the :ref:`Development Guide <development_guide>` steps to set up the cross-compilation environment, including setting up the Docker container and VS Code workspace.

#. Collect all required packages for the target sample application in the ``ros2_ws/src/`` directory inside the cross-compile Docker container.

   Refer to the **RZ/V ROS 2 Packages Used** section of each sample application for the list of required packages.

#. Cross-compile the ROS 2 workspace and deploy the workspace to the RZ/V2H RDK board.

#. Install the required dependencies on the RZ/V2H RDK board.

   .. code-block:: bash

      rosdep install --from-paths <path/to>/install/*/share -y -r --ignore-src

   Replace ``<path/to>/install/`` with the actual path to the ``install/`` directory on your RZ/V2H RDK board.

#. (Optional) If you have the real robot hardware, set up the robot arm or hand according to the instructions provided in the respective sample application sections.

The following figure shows the hardware setup of the RZ/V2H RDK board and peripherals used in the sample applications:

.. figure:: ../../images/hardware_connect_demo.png
   :alt: Hardware setup for sample applications on the RZ/V2H RDK board
   :width: 600px
   :align: center

   Hardware setup for sample applications on the RZ/V2H RDK board.

.. tip::

   A physical robot arm or hand is not required for the sample applications.
   You can use Foxglove Studio to visualize the robot state and control the robot in a simulated environment.
   See :ref:`Foxglove Studio Visualization <foxglove_visualization>` for more details.

List of Sample Applications
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow the instructions in the respective sample application sections to run each application on the RZ/V2H RDK platform.

.. toctree::
   :maxdepth: 1

   arm_teleoperation
   dexhand
   rock_paper_scissors
   static_object_detection
   hand_landmark