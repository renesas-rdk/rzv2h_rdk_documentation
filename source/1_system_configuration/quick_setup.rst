Overview
--------------------------------
The RZ/V2H RDK uses a Linux kernel **version 6.10**, which is built from the **Yocto Styhead Project**.
This section provides instructions on how to customize and rebuild various components of the system by using the **Yocto Extensible SDK (eSDK)** environment.

The Yocto Project eSDK provides tools that allow developers to:

- Add new applications and libraries to an image
- Modify and rebuild the source of existing components
- Test software changes directly on the target hardware

To begin working with the Extensible Software Development Kit (eSDK) in Yocto, consult the official documentation provided by the Yocto Project.

This guide offers comprehensive instructions on configuring and utilizing the eSDK effectively.

Access the official eSDK documentation by following this URL: `Using the Extensible SDK <https://docs.yoctoproject.org/5.1.4/sdk-manual/extensible.html#using-the-extensible-sdk>`_.

Prerequisites
--------------

Before proceeding, ensure that the following prerequisites are in place:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - **Item**
     - **Description / Link**
   * - **Docker**
     - Must be installed on the Host PC.
       Refer to the `Docker Official Installation Guide <https://docs.docker.com/engine/install/>`_.
   * - **Yocto eSDK for RZ/V2H RDK**
     - Required for extensible development and rebuilding components.
       Obtain the installer:
       ``poky-glibc-x86_64-renesas-core-image-weston-cortexa55-rz-cmn-toolchain-ext-5.1.4.sh``
       *(TODO: Add link to Yocto eSDK package)*
   * - **Yocto SDK for RZ/V2H RDK**
     - Standard SDK tool-chain for building applications.
       Obtain the installer:
       ``poky-glibc-x86_64-renesas-core-image-weston-cortexa55-rz-cmn-toolchain-5.1.4.sh``
       *(TODO: Add link to Yocto SDK package)*
   * - **RZ/V2H RDK X Compile Docker**
     - Preconfigured cross-compilation Docker environment.
       Refer to the `RZ/V2H RDK X Compile Docker repository`.
       *(TODO: Add repository link)*

Quick set up guide
--------------------------------

Common docker environment setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To streamline the development process, it is recommended to use a Docker container that has been preconfigured for cross-compiling applications as well as eSDK development for the RZ/V2H RDK board.

This section provides a quick guide to setting up the Docker environment.

Obtain all files from the Prerequisites section, and following the step below to set up the Docker environment:

1. Clone the `x_compilation_docker` repository to your Host PC.

   .. code-block:: bash

       renesas@builder-pc:~$ git clone https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/x_compilation_docker.git

2. Copy both **Yocto SDK** installers to the Docker build context directory. Please replace the paths below with your actual file locations.

    .. code-block:: bash

        renesas@builder-pc:~$ cp poky-glibc-x86_64-renesas-core-image-weston-cortexa55-rz-cmn-toolchain-5.1.4.sh ~/x_compilation_docker/

3. Build the Docker Container

    Navigate to the x_compilation_docker directory and build the Docker image using the following command:

    .. code-block:: bash

         renesas@builder-pc:~$ cd x_compilation_docker/
         renesas@builder-pc:~/x_compilation_docker$ ./setup_ros2_cross_env.sh <path_to_ros2_workspace> [name_of_docker_container]

- <path_to_ros2_workspace>: Path to your ROS2 workspace, mounted inside the container.
- [name_of_docker_container]: Optional container name (default: rzv2h_ros_xbuild).

After complete this step, the Docker image (name: rzv2h_ros_xbuild:latest) and container will be created.

**What does this script do?**

- Copying the tool-chain scripts into the build directory lets the Dockerfile install the Yocto SDK inside the image.
- The setup script usually runs docker build (to produce the image) and docker run (to create/start a container) with proper mounts, environment variables and volumes.
- Inside the container the image will have the cross compilers, sysroot and a configured environment (CMake toolchain file, sourced toolchain setup) so colcon/CMake can cross-compile ROS2 for the target.

4. Enter the Docker Container

   Use the following command to enter the Docker container:

   .. code-block:: bash

       renesas@builder-pc:~$ docker exec -it [name_of_docker_container] /bin/bash

.. _esdk_setup:

eSDK Setup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To get started, extract the eSDK and install the tool-chain.

Please replace the paths below with your actual file locations.

.. tip::

   There is no requirement to use the Yocto eSDK within the Docker environment.

   However, using Docker can simplify the setup process and ensure a consistent build environment.
   Since Yocto SDK/eSDK installations can be complex, Docker provides a preconfigured environment that minimizes setup time and potential configuration issues.

   Additional, ensure that the **Yocto eSDK installer file** has been **copied into the container**
   so it can be executed from within that environment.


   .. code-block:: bash

       renesas@builder-pc:~$ docker cp poky-glibc-x86_64-renesas-core-image-weston-cortexa55-rz-cmn-toolchain-ext-5.1.4.sh [name_of_docker_container]:~

To set up your environment:

1. Install the Yocto eSDK tool-chain.

For example, to install the tool-chain, run the following command.

.. code-block:: bash

    renesas@docker-pc:~$ sh ./poky-glibc-x86_64-renesas-core-image-weston-cortexa55-rz-cmn-toolchain-ext-5.1.4.sh

.. note::

   You **cannot install the eSDK as the root user** because BitBake does not run with root privileges.
   Attempting to install the extensible SDK as root will cause the installation to fail or behave unpredictably.


If the installation is successful, the following messages will appear in the terminal output.

.. code-block:: bash

    renesas@docker-pc:~$ sh ./poky-glibc-x86_64-renesas-core-image-weston-cortexa55-rz-cmn-toolchain-ext-5.1.4.sh
    Poky (Yocto Project Reference Distro) Extensible SDK installer version 5.1.4
    ==========================================================================
    Enter target directory for SDK (default: ~/poky_sdk):
    You are about to install the SDK to "/home/renesas/poky_sdk/5.1.4". Proceed [Y/n]? Y
    Extracting SDK..............done
    Setting it up...
    Extracting buildtools...
    Preparing build system...
    Parsing recipes: 100% |##########################################| Time: 0:00:52
    Initialising tasks: 100% |#######################################| Time: 0:00:00
    Checking sstate mirror object availability: 100% |###############| Time: 0:00:00
    Loading cache: 100% |############################################| Time: 0:00:00
    Initialising tasks: 100% |#######################################| Time: 0:00:00
    done
    SDK has been successfully set up and is ready to be used.
    Each time you wish to use the SDK in a new shell session, you need to source the
    environment setup script e.g.
    $ . /home/renesas/poky_sdk/5.1.4/environment-setup-cortexa55-poky-linux

2. Set up cross-compile environment. The following command assumes that you installed the SDK in `~/poky_sdk/5.1.4`

.. code-block:: bash

    renesas@docker-pc:~$ source ~/poky_sdk/5.1.4/environment-setup-cortexa55-poky-linux
    SDK environment now set up; additionally you may now run devtool to perform development tasks.
    Run devtool --help for further details.

.. note::

   The user needs to run the above command once for each shell session.