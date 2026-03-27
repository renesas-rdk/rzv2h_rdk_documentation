.. _rzv2h_ai_sdk_apps:

Cross-build Renesas AI Applications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes how to cross-compile Renesas AI Applications for the RZ/V2H RDK platform using the Renesas RDK Docker cross-compilation environment.

The Renesas AI Applications are sets of application source code, pre-built application binaries, and pre-trained AI model objects, which allow users to run AI applications on the RZ/V2H RDK.

For more details, visit the `Renesas AI Applications <https://renesas-rz.github.io/rzv_ai_sdk/latest/about-applications.html>`_ page.

.. important::

   #. The instructions in this section apply only to porting and building the Renesas AI Applications on the RZ/V2H RDK platform by using the provided cross-compilation environment.
      They do not describe how to compile AI model files.

   #. To compile AI model files, :ref:`follow the Renesas AI model compilation guide <how_to_compile_your_own_model>`, and then use the compiled model files to run the AI applications on the RZ/V2H RDK.

   #. The target board must use :ref:`OpenCVA <opencva>`, not the default OpenCV package from the Ubuntu repository, because the Renesas AI Applications use APIs provided by OpenCVA.

Prerequisites
"""""""""""""

Before building the Renesas AI Applications, ensure that you have completed the following:

- Set up the RZ/V2H RDK with the Ubuntu 24.04 server image (``ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz``) flashed on the microSD card.
  Follow the instructions in the :ref:`Quick Setup Guide <quick_setup_rdk_guide>`.
- Complete the :ref:`Cross compilation environment setup <requirements_ros2_cross_build>` section to set up the Docker-based cross-compilation environment.

Available AI Applications
"""""""""""""""""""""""""

The following AI Applications are available, developed by Renesas and third-party developers:

- `RZ/V AI Applications Repository <https://github.com/renesas-rz/rzv_ai_sdk/tree/main>`_
- `RZ/V Sample Applications Repository <https://github.com/renesas-rz/rzv_sample_apps/tree/main>`_
- `Ignitarium Renesas - RZ/V AI Applications <https://github.com/Ignitarium-Renesas/rzv_ai_apps>`_
- `Computermind Corporation - DRP-AI Demo App <https://github.com/ComputermindCorp/drp-ai-demo-app/tree/main>`_

Setting up the Environment
""""""""""""""""""""""""""

#. Start and access the Docker container:

   .. code-block:: bash

      docker exec -it ros2_cross_build_container bash

#. Connect to the Docker container from VS Code using the **Remote - Containers** extension.

   Once connected, open the ``/drp-ai_tvm`` directory inside the container as your working folder.

   Refer to the :ref:`Cross compilation environment setup <requirements_ros2_cross_build>` section for instructions on how to connect from VS Code.

Install Dependencies into the Sysroot
""""""""""""""""""""""""""""""""""""""

Install the required development libraries for Wayland, GStreamer, and other dependencies into the ARM64 sysroot:

.. code-block:: bash

   rzv2h-chroot apt update
   rzv2h-chroot apt install -y \
      wayland-protocols \
      libwayland-dev \
      wayland-scanner++ \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      libturbojpeg0-dev \
      libjpeg-dev \
      libgles2-mesa-dev \
      libegl1-mesa-dev

.. note::

   The above command installs the necessary development libraries for building the Renesas AI Applications that use Wayland display and GStreamer functionalities.

   For applications with additional dependencies, install those dependencies into the sysroot using ``rzv2h-chroot`` before building the application.

Generate XDG Shell Protocol Files
""""""""""""""""""""""""""""""""""

The AI applications that use Wayland display require XDG shell protocol files.
Generate them inside the sysroot:

.. code-block:: bash

   # Enter the chroot environment
   rzv2h-chroot

   # Generate the xdg-shell-client-protocol.h file
   wayland-scanner client-header \
      /usr/share/wayland-protocols/stable/xdg-shell/xdg-shell.xml \
      /usr/include/xdg-shell-client-protocol.h

   # Generate the xdg-shell-client-protocol.c file
   wayland-scanner private-code \
      /usr/share/wayland-protocols/stable/xdg-shell/xdg-shell.xml \
      /usr/src/xdg-shell-client-protocol.c

   # Exit the chroot environment
   exit

Building Renesas AI Applications
"""""""""""""""""""""""""""""""""

Select the desired AI Application from the repositories mentioned above and clone the repository inside the Docker container.

Based on the repository, follow the specific porting instructions below to build the application.

.. tip::

   The following common steps need to be performed for all applications that use Wayland functions:

   - Update the ``CMakeLists.txt`` file to use the correct include paths and XDG shell protocol source file.
   - Update the Mali GPU library links in the ``CMakeLists.txt`` file.
   - Update the Wayland initialization function in the wayland source file.
   - Compile the application using CMake with the provided toolchain file.

   The above steps are necessary to ensure that the application can compile and run properly on the RZ/V2H RDK platform with the Ubuntu-based sysroot and Mali GPU libraries.

.. _ai_app_cmake_update:

Common Step: Update CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each application has a ``CMakeLists.txt`` file in its ``src/`` directory (e.g., ``/drp-ai_tvm/how-to/sample_app_v2h/app_deeplabv3_cam/src/CMakeLists.txt``).

Find and replace the following lines in the ``CMakeLists.txt`` file:

**Before change:**

.. code-block:: cmake

   include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl)
   list(APPEND SRC ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl/xdg-shell-client-protocol.c)

**After modification:**

.. code-block:: cmake
   :emphasize-lines: 1,2

   include_directories(${CMAKE_SYSROOT}/usr/include/gstreamer-1.0/gst/gl)
   list(APPEND SRC ${CMAKE_SYSROOT}/usr/src/xdg-shell-client-protocol.c)

.. note::

   The old paths reference GStreamer debug source directories (``/usr/src/debug/gstreamer1.0-plugins-base/...``), which are not available in the Ubuntu-based sysroot.
   The new paths use the standard GStreamer include directory and the XDG shell protocol file generated in the previous step.

.. _ai_app_mali_lib_update:

Common Step: Update Mali GPU Library Links
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``target_link_libraries`` section in the ``CMakeLists.txt`` file needs to be updated to use the Mali GPU library paths instead of the generic EGL/GLESv2/wayland-egl libraries.

Add the Mali library path variable and update the link libraries:

**Before change:**

.. code-block:: cmake

   target_link_libraries(${EXE_NAME} PRIVATE
      ...
      EGL
      GLESv2
      wayland-cursor
      wayland-egl
      wayland-client
      ...

**After modification:**

.. code-block:: cmake
   :emphasize-lines: 1,5,6,7

   set(MALI_LIB_PATH "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/mali")

   target_link_libraries(${EXE_NAME} PRIVATE
      ...
      ${MALI_LIB_PATH}/libEGL.so
      ${MALI_LIB_PATH}/libGLESv2.so
      ${MALI_LIB_PATH}/libwayland-egl.so
      wayland-cursor
      wayland-client
      ...

.. note::

   The Ubuntu-based sysroot provides the Mali GPU libraries under ``/usr/lib/aarch64-linux-gnu/mali/`` instead of the standard system library paths.
   Using explicit paths ensures the linker picks up the correct Mali-specific implementations of EGL, GLESv2, and wayland-egl.

Common Step: Update Wayland Initialization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For applications that use Wayland display functions (i.e., applications with a ``wayland.cpp`` file in the ``src/`` folder), update the ``initWaylandDisplay`` function to add ``wl_display_roundtrip`` after ``wl_surface_commit``:

.. code-block:: diff
   :emphasize-lines: 8-10

   diff --git a/src/wayland.cpp b/src/wayland.cpp
   --- a/src/wayland.cpp
   +++ b/src/wayland.cpp
   @@ static int8_t initWaylandDisplay(struct wl_display** wlDisplay, struct wl_surfac
       xdg_surface_add_listener(xdg_surface, &xdg_surface_listener, NULL);

       wl_surface_commit(*wlSurface);
   +
   +    wl_display_roundtrip(*wlDisplay);
   +
       return 0;
   }

Common Step: Compile the Application
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After making the above common modifications, follow the standard CMake build process to compile the application:

.. code-block:: bash

   mkdir build && cd build

   cmake .. -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAINS_WS/cross.cmake \
      -DAPP_NAME=<app_name> \
      -DCMAKE_BUILD_TYPE=Release

   make -j$(nproc)

Replace ``<app_name>`` with the actual application name, for example, ``app_deeplabv3_cam``.

.. warning::

   You must use the toolchain file at ``$TOOLCHAINS_WS/cross.cmake``, which is provided in the Docker container for cross-compilation.

   Do **not** use ``-DCMAKE_TOOLCHAIN_FILE=$TVM_ROOT/apps/toolchain/runtime.cmake``.
   The ``runtime.cmake`` file is not compatible with the RZ/V2H RDK sysroot and library paths, and using it will cause build errors.

Deploying Renesas AI Applications
""""""""""""""""""""""""""""""""""

After building the application, deploy the generated binaries and necessary files to the RZ/V2H RDK board.

The following files need to be copied to the RZ/V2H RDK board for the Wayland display and GStreamer functionalities to work properly:

.. code-block:: text

   sample_deeplabv3_cam/
   ├── app_deeplabv3_cam        # Application binary generated from cross-compilation
   └── deeplabv3_cam            # Model files generated with TVM extension package
       ├── input_0.bin
       ├── mera.plan
       ├── model_subgraphs.json
       ├── preprocess
       │   ├── addr_map.txt
       │   ├── aimac_cmd.bin
       │   ├── aimac_desc.bin
       │   ├── aimac_param_cmd.bin
       │   ├── aimac_param_desc.bin
       │   ├── drp_config.mem
       │   ├── drp_desc.bin
       │   ├── drp_param.bin
       │   ├── drp_param_info.txt
       │   └── weight.bin
       ├── project.mdp
       ├── ref_result_0.bin
       └── sub_0000__CPU_DRP_TVM
           ├── deploy.json
           ├── deploy.params
           └── deploy.so

.. note::

   There is no need to copy the TVM runtime libraries (libmera2_runtime.so,etc.) to the target device.

   The provided Ubuntu image already includes the necessary libraries for running the application.
   Only the generated binaries and model files need to be copied.