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

   Create and enter the container from the Docker image `ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:latest <https://github.com/orgs/renesas-rdk/packages/container/package/rzv2h_ubuntu_xbuild>`_ image.

   Run the setup script to create the Docker-based cross-compilation environment.

   .. code-block:: bash

      wget https://github.com/renesas-rdk/ros2_demo_workspace/raw/refs/heads/main/common_utils/setup_rdk_docker.sh
      chmod +x setup_rdk_docker.sh

   Execute the script and follow the guide to complete the setup.

   .. code-block:: bash

      ./setup_rdk_docker.sh

   Enter the Docker container:

   .. code-block:: bash

      docker exec -it container_name bash

   Replace ``container_name`` with your created Docker container name.

   After this step, we assume that you already know about how to cross build the ROS 2 application on RZ/V2H RDK board as well as how to deploy and run it.

   The next instruction only focuses on demo-specific operation, not cover the general setup anymore.

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