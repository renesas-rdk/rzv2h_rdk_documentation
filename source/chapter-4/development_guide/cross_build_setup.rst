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

#. Run the setup script to create the Docker-based cross-compilation environment.

   .. code-block:: bash

      wget https://github.com/renesas-rdk/ros2_demo_workspace/raw/refs/heads/main/common_utils/setup_rdk_docker.sh
      chmod +x setup_rdk_docker.sh

   Then run it:

   .. code-block:: bash

      ./setup_rdk_docker.sh

   The setup script performs the following tasks:

   - checks whether the Docker image already exists locally
   - compares the local image digest with the remote image digest when ``docker buildx`` is available
   - pulls the image only if needed or if the remote image differs
   - creates the ROS 2 workspace mount point on the host machine
   - creates and starts the Docker container
   - optionally prepares the ROS 2 workspace inside the container

   The script supports command-line options for non-interactive use:

   .. code-block:: bash

      ./setup_rdk_docker.sh [options]

   .. list-table:: Command-Line Options
      :widths: 35 65
      :header-rows: 1

      * - Option
        - Description
      * - ``-i, --image IMAGE``
        - Docker image to use (default: ``ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:latest``)
      * - ``-n, --container-name NAME``
        - Container name (default: ``ros2_cross_build_container``)
      * - ``-w, --workspace PATH``
        - Host ROS 2 workspace path (default: ``$HOME/ros2_ws``)
      * - ``-y, --yes``
        - Non-interactive mode; accept yes for all confirmations
      * - ``--pull``
        - Automatically pull or update the image if needed
      * - ``--create``
        - Automatically create and start the container if needed
      * - ``--prep``
        - Automatically prepare the workspace inside the container
      * - ``--shell``
        - Open a shell inside the container after setup
      * - ``-h, --help``
        - Show usage information

   Example of fully non-interactive usage:

   .. code-block:: bash

      ./setup_rdk_docker.sh -n my_container -w ~/ros2_ws -y --pull --create --prep

   .. note::

      The script can be reused to create additional containers. Each container
      must have a unique name. If a container with the specified name already
      exists, the script exits with an error and suggests removing the existing
      container or choosing a different name.

#. Provide the required information when prompted.

   When run without command-line options, the script asks for:

   - the Docker container name (default: ``ros2_cross_build_container``)
   - the ROS 2 workspace path on the host machine (default: ``$HOME/ros2_ws``)

   If you press ``Enter`` without typing a value, the default value is used.

   Example:

   .. code-block:: bash

      Enter container name [default: ros2_cross_build_container]:
      Enter ROS 2 workspace path on host [default: /home/user/ros2_ws]:

   The script then displays the configuration summary and asks for confirmation
   before proceeding.

#. Prepare the ROS 2 workspace inside the Docker container.

   During execution, the script prompts whether to prepare the ROS 2 workspace inside the container.

   This step is required the first time you use a workspace with the container and when mounting
   a new workspace path.

   If you choose to run it, the script executes:

   .. code-block:: bash

      docker exec "$CONTAINER_NAME" bash -c "
          sudo chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws &&
          cp -r /home/ubuntu/toolchains/.vscode /home/ubuntu/ros2_ws/ &&
          cp /home/ubuntu/toolchains/.clang-format /home/ubuntu/ros2_ws/
      "

#. Access the Docker container from a terminal.

   After the script completes, it offers to open a shell inside the container.
   If you decline or if the script finishes without opening a shell, access the
   container with:

   .. code-block:: bash

      docker exec -it <container_name> bash

   Replace ``<container_name>`` with the name you chose during setup
   (default: ``ros2_cross_build_container``).

#. Inside the container, go to your ROS 2 workspace.

   .. code-block:: bash

      cd ~/ros2_ws

Accessing the Docker container from VS Code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After the Docker container is running, you can connect to it directly from VS Code using the Remote - Containers extension.

#. Open VS Code on the host machine.
#. Open the Command Palette ``(Ctrl+Shift+P)``.
#. Run **Remote-Containers: Attach to Running Container...**
#. Select ``$CONTAINER_NAME`` from the list of running containers.
#. Once connected, open the ``/home/ubuntu/ros2_ws`` directory inside the container.

This allows you to edit source files, use the integrated terminal, and work directly inside the cross-compilation environment from VS Code.

.. tip::

   Environment variables set in the Docker container:

   - ``$ROS2_WS``: Default ROS 2 workspace directory, set to ``/home/ubuntu/ros2_ws``. You can use this variable to navigate to your ROS 2 workspace inside the container.
   - ``$TOOLCHAINS_WS``: Directory for cross-compilation toolchain files.