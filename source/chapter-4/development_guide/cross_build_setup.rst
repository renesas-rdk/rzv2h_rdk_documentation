.. _requirements_ros2_cross_build:

Cross compilation Environment Setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section provides instructions on how to set up the cross-compilation environment for building ROS 2 applications for the Renesas RZ/V2H RDK platform.

Hardware requirements
"""""""""""""""""""""

- Ubuntu 24.04 host machine.

- The following image show the expected setup for cross-building applications for the RZ/V2H RDK platform:

  .. figure:: ../../images/cross_build_setup.png
     :align: center
     :alt: Cross Build Hardware Setup Diagram
     :width: 600px

     Cross Build Hardware Setup Diagram

  Make sure your board and host machine are properly set up and connected to the same network to enable communication between them during development and deployment.

Software requirements on the host machine
"""""""""""""""""""""""""""""""""""""""""

This section describes the required software on the host machine and how to set up and access the Docker-based cross-compilation environment.

Required software
~~~~~~~~~~~~~~~~~

- `Docker <https://docs.docker.com/engine/install/ubuntu/>`_ installed on the host machine.
- `VS Code <https://code.visualstudio.com/download>`_ for code editing and development.
- `Remote - Containers extension <https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers>`_ installed in VS Code for developing inside the Docker container.

Docker environment setup
~~~~~~~~~~~~~~~~~~~~~~~~

#. Pull the latest Docker image provided by Renesas RDK for cross-development. The image includes the required software and toolchain files for cross-compilation.

   .. code-block:: bash

      docker pull ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:latest

#. Create the mount point for sharing the ROS 2 workspace between the host machine and the Docker container.

   .. code-block:: bash

      export ROS2_WS=~/ros2_ws
      mkdir -p $ROS2_WS

   Replace ``$ROS2_WS`` with the actual path where you want to create your ROS 2 workspace on the host machine.

#. Create and run a Docker container from the pulled image to set up the cross-compilation environment.

   .. code-block:: bash

      docker run -it --name ros2_cross_build_container -v $ROS2_WS:/home/ubuntu/ros2_ws ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:latest

   After this step, the Docker container is ready to use, and the ``ros2_ws`` directory is shared with the host machine for reuse.

#. Access the Docker container from a terminal.

   .. code-block:: bash

      docker exec -it ros2_cross_build_container bash

#. Inside the container, go to your ROS 2 workspace.

   .. code-block:: bash

      cd ~/ros2_ws

Accessing the Docker container from VS Code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After the Docker container is running, you can connect to it directly from VS Code using the Remote - Containers extension.

#. Open VS Code on the host machine.
#. Open the Command Palette ``(Ctrl+Shift+P)``.
#. Run **Remote-Containers: Attach to Running Container...**
#. Select ``ros2_cross_build_container`` from the list of running containers.
#. Once connected, open the ``/home/ubuntu/ros2_ws`` directory inside the container.

This allows you to edit source files, use the integrated terminal, and work directly inside the cross-compilation environment from VS Code.

.. tip::

   Environment variables set in the Docker container:

   - ``$ROS2_WS``: Default ROS 2 workspace directory, set to ``/home/ubuntu/ros2_ws``. You can use this variable to navigate to your ROS 2 workspace inside the container.
   - ``$TOOLCHAINS_WS``: Directory for cross-compilation toolchain files.