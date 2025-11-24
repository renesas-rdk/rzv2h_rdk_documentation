Renesas AI Applications
--------------------------

AI Applications are sets of application source code, pre-build application binary and pre-trained AI model objects, which allow users to run AI Applications easily and quickly.

Users can select the category of applications and access the applications provided on GitHub.

This section introduces the Renesas AI Applications provided by Renesas and third-party developers for the RZ/V2H RDK platform.

For more details, please visit the `Renesas AI Applications <https://renesas-rz.github.io/rzv_ai_sdk/latest/about-applications.html>`_ page.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^

Before running the Renesas AI Applications, ensure that you have completed the following prerequisites:

- Set up the RZ/V2H RDK with **renesas-core-image-weston.wic.gz** image flashed on the microSD card. Follow the instructions in the :ref:`Quick Setup Guide <quick_setup_rdk_guide>` to set up the environment.
- Building the DRP-AI TVM toolchain docker, follow the instructions provided in the `rzv_ai_toolchain_docker repository <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/ai/rzv_ai_toolchain_docker/-/tree/master?ref_type=heads>`_.

.. important::

    Please use the RZ/V2H SDK when installing the DRP-AI TVM extension package. **Do not use SDKs provided by other platforms.**

- Install the **libtvm_runtime.so** library to the target RZ/V2H RDK platform:

.. code-block:: bash

    $ wget https://github.com/renesas-rz/rzv_drp-ai_tvm/raw/refs/heads/v2.5.1/obj/build_runtime/V2H/libtvm_runtime.so
    $ mv libtvm_runtime.so /usr/lib/
    $ ldconfig

Available AI Applications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following AI Applications are available, developed by Renesas and third-party developers:

    - `RZ/V AI Applications Repository <https://github.com/renesas-rz/rzv_ai_sdk/tree/7362488f2b59c46c337c797e9412acef6ed9dd7c>`_.
    - `RZ/V Sample Applications Repository <https://github.com/renesas-rz/rzv_sample_apps/tree/main>`_.
    - `Ignitarium Renesas - RZ/V AI Applications <https://github.com/Ignitarium-Renesas/rzv_ai_apps>`_.
    - `Computermind corporation - DRP-AI Demo App <https://github.com/ComputermindCorp/drp-ai-demo-app/tree/main>`_.

Building Renesas AI Applications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Select the desired AI Application from the repositories mentioned above and clone the repository to your DRP-AI TVM toolchain docker.

Based on the repository, follow the specific porting instructions bellow to build the application.

.. tip::

    The following common steps need to be performed for all the applications that use wayland functions:

    - Update the wayland initialization function in the wayland source file.
    - Update the CMakeLists.txt file to include the correct GStreamer version and wayland source file.

.. _commonmark:

Common step
""""""""""""""

Inside the DRP-AI toolchain docker, update the ``/drp-ai_tvm/how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp`` file to include ``wl_display_roundtrip`` after ``wl_surface_commit`` as shown below:

.. code-block:: diff
    :emphasize-lines: 9,10,11

    diff --git a/how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp b/how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp
    index 20a9236..10cc132 100644
    --- a/how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp
    +++ b/how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp
    @@ -293,6 +293,9 @@ static int8_t initWaylandDisplay(struct wl_display** wlDisplay, struct wl_surfac
        xdg_surface_add_listener(xdg_surface, &xdg_surface_listener, NULL);

        wl_surface_commit(*wlSurface);
    +
    +    wl_display_roundtrip(*wlDisplay);
    +
        return 0;
    }

How to port and build `rzv_drp-ai_tvm/how-to/sample_app_v2h <https://github.com/renesas-rz/rzv_drp-ai_tvm/tree/main/how-to/sample_app_v2h>`_ applications
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The following step by step guide shows how to port ``rzv_drp-ai_tvm/how-to/sample_app_v2h/app_yolox_cam`` application as an example.

You can follow the same steps to port other applications available in the ``rzv_drp-ai_tvm/how-to/sample_app_v2h`` folder.

Inside the DRP-AI toolchain docker, navigate to the ``/drp-ai_tvm/how-to/sample_app_v2h/app_yolox_cam/src`` folder.

- Update the ``CMakeLists.txt`` file:

    **Before change (line 21-31 of CMakeLists.txt file):**

    .. code-block:: cmake

        if(NOT EXISTS "$ENV{SDK}/sysroots/cortexa55-poky-linux")
            list(REMOVE_ITEM SOURCE ${TVM_ROOT}/../how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp)
            add_definitions(-DV2H)
            message("-- For Dunfell or older")
        else()
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${TVM_ROOT}/../how-to/sample_app_v2h/common_files/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- For Scarthgap or lator")
        endif()

    **After modification:**

    .. code-block:: cmake
        :emphasize-lines: 6,8

        if(NOT EXISTS "$ENV{SDK}/sysroots/cortexa55-poky-linux")
            list(REMOVE_ITEM SOURCE ${TVM_ROOT}/../how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp)
            add_definitions(-DV2H)
            message("-- For Dunfell or older")
        else()
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${TVM_ROOT}/../how-to/sample_app_v2h/common_files/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- For Scarthgap or lator")
        endif()

- Refer to the `README.md <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/how-to/sample_app_v2h/app_yolox_cam/README.md#object-detection-yolox>`_ file in the ``app_yolox_cam`` folder for further build instructions to compile and run the application on the RZ/V2H RDK platform.

How to port and build Renesas AI Applications: `RZ/V AI Applications Repository <https://github.com/renesas-rz/rzv_ai_sdk/tree/7362488f2b59c46c337c797e9412acef6ed9dd7c>`_
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Clone the `rzv_ai_sdk` repository to your DRP-AI TVM toolchain docker.

.. code-block:: bash

    $ cd /drp-ai_tvm/data
    $ git clone https://github.com/renesas-rz/rzv_ai_sdk.git
    $ cd rzv_ai_sdk
    $ git checkout 7362488f2b59c46c337c797e9412acef6ed9dd7c

The following step by step guide shows how to port Renesas AI Applications `Q01_footfall_counter <https://github.com/renesas-rz/rzv_ai_sdk/tree/7362488f2b59c46c337c797e9412acef6ed9dd7c/Q01_footfall_counter>`_ as an example.

You can follow the same steps to port other Renesas AI Applications available in the repository.

- Update the ``rzv_ai_sdk/Q01_footfall_counter/src/CMakeLists.txt`` file:

    **Before change (line 16-36 of CMakeLists.txt file):**

    .. code-block:: cmake

        if(PRODUCT STREQUAL "V2H")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.h)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland_scarthgap.cpp)
            add_definitions(-DV2H)
            message("-- PRODUCT [V2H]")
        elseif(PRODUCT STREQUAL "V2N")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.h)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- PRODUCT [V2N]")
        else()
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.h)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland_scarthgap.cpp)
            add_definitions(-DV2L)
            message("-- PRODUCT [V2L]")
        endif()

    **After modification:**

    .. code-block:: cmake
        :emphasize-lines: 1,4,6

        if(PRODUCT STREQUAL "V2H")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.h)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- PRODUCT [V2H]")
        endif()

    .. important::

        Because this application don't use wayland functions, there is no need to update the ``rzv_ai_sdk/Q01_footfall_counter/src/wayland_scarthgap.cpp`` file.

        However, if you are porting other applications that use wayland functions (``src`` folder have the ``wayland_scarthgap.cpp`` file), please make sure to update the corresponding wayland file as mentioned in the :ref:`Common Step <commonmark>` section above.

        For example, with **rzv_ai_sdk/Q04_fish_classification** application, we have to update the ``/drp-ai_tvm/data/rzv_ai_sdk/Q04_fish_classification/src/wayland_scarthgap.cpp`` file.

- Continue with the **step 3** in the build instructions provided in the repository to compile and run the application on the RZ/V2H RDK platform: `Q01_footfall_counter - Application File Generation <https://github.com/renesas-rz/rzv_ai_sdk/tree/7362488f2b59c46c337c797e9412acef6ed9dd7c/Q01_footfall_counter#application-file-generation>`_.

How to port and build and build RZ/V Sample Applications: `RZ/V Sample Applications Repository <https://github.com/renesas-rz/rzv_sample_apps/tree/main>`_.
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Clone the `rzv_sample_apps` repository to your DRP-AI TVM toolchain docker.

.. code-block:: bash

    $ cd /drp-ai_tvm/data
    $ git clone https://github.com/renesas-rz/rzv_sample_apps.git
    $ cd rzv_sample_apps
    $ git checkout d760587ea766830d8ca224fc67a05941194358fd

The following step by step guide shows how to port RZ/V Sample Applications `rzv_sample_apps <https://github.com/renesas-rz/rzv_sample_apps/tree/d760587ea766830d8ca224fc67a05941194358fd>`_ as an example.

You can follow the same steps to port other RZ/V Sample Applications available in the repository.

- Update the ``rzv_sample_apps/S01_face_mosaic/src/CMakeLists.txt`` file:

    **Before change (line 16-24 of CMakeLists.txt file):**

    .. code-block:: cmake

        if(V2H STREQUAL "ON")
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/PreRuntime.cpp)
            add_definitions(-DV2H)
            message("-- PRODUCT [V2H]")
        else()
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/PreRuntimeV2H.cpp)
            add_definitions(-DV2L)
            message("-- PRODUCT [V2L]")
        endif()

    **After modification:**

    .. code-block:: cmake
        :emphasize-lines: 3,4,5

        if(V2H STREQUAL "ON")
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/PreRuntime.cpp)
            add_definitions(-DV2H -DV2N)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            message("-- PRODUCT [V2H]")
        else()
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/PreRuntimeV2H.cpp)
            add_definitions(-DV2L)
            message("-- PRODUCT [V2L]")
        endif()

- Replace the ``rzv_sample_apps/S01_face_mosaic/src/wayland.cpp`` and the ``rzv_sample_apps/S01_face_mosaic/src/wayland.h`` files

    Please copy the ``/drp-ai_tvm/how-to/sample_app_v2h/common_files/wayland_scarthgap.cpp`` and ``/drp-ai_tvm/how-to/sample_app_v2h/common_files/wayland_scarthgap.h`` files respectively to replace them.

- Continue with the **step 3** in the build instructions provided in the repository to compile and run the application on the RZ/V2H RDK platform: `S01_face_mosaic - Application File Generation <https://github.com/renesas-rz/rzv_sample_apps/tree/d760587ea766830d8ca224fc67a05941194358fd/S01_face_mosaic#application-file-generation>`_.

How to port and build and build Ignitarium Renesas RZ/V AI Applications: `Ignitarium Renesas - RZ/V AI Applications <https://github.com/Ignitarium-Renesas/rzv_ai_apps>`_.
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Clone the `Ignitarium-Renesas/rzv_ai_apps` repository to your DRP-AI TVM toolchain docker.

.. code-block:: bash

    $ cd /drp-ai_tvm/data
    $ git clone https://github.com/Ignitarium-Renesas/rzv_ai_apps.git
    $ cd rzv_ai_apps
    $ git checkout d607cba6e24ffd8a5093ecab45f7bedd5e010b5d

The following step by step guide shows how to port Renesas AI Applications `01_Head_count <https://github.com/Ignitarium-Renesas/rzv_ai_apps/tree/e06af580066307e6fa348b134e096f896f786425/01_Head_count>`_ as an example.

You can follow the same steps to port other Renesas AI Applications available in the repository.

- Update the ``rzv_ai_apps/01_Head_count/src/CMakeLists.txt`` file:

    **Before change (line 16-39 of CMakeLists.txt file):**

    .. code-block:: cmake

        if(PRODUCT STREQUAL "V2H")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.h)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/PreRuntime.cpp)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland_scarthgap.cpp)
            add_definitions(-DV2H)
            message("-- PRODUCT [V2H]")
        elseif(PRODUCT STREQUAL "V2N")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.h)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/PreRuntime.cpp)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- PRODUCT [V2N]")
        else()
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.h)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/PreRuntimeV2H.cpp)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland_scarthgap.cpp)
            add_definitions(-DV2L)
            message("-- PRODUCT [V2L]")
        endif()

    **After modification:**

    .. code-block:: cmake
        :emphasize-lines: 1,4,7

        if(PRODUCT STREQUAL "V2H")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.h)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/PreRuntime.cpp)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- PRODUCT [V2H]")
        endif()

- Update the function ``initWaylandDisplay`` in the ``rzv_ai_apps/01_Head_count/src/wayland_scarthgap.cpp`` file, please add ``wl_display_roundtrip`` after ``wl_surface_commit`` as shown below:

    .. code-block:: diff
        :emphasize-lines: 9,10,11

        diff --git a/01_Head_count/src/wayland_scarthgap.cpp b/01_Head_count/src/wayland_scarthgap.cpp
        index 0fb92a4..b0a6d11 100644
        --- a/01_Head_count/src/wayland_scarthgap.cpp
        +++ b/01_Head_count/src/wayland_scarthgap.cpp
        @@ -323,6 +323,9 @@ static int8_t initWaylandDisplay(struct wl_display** wlDisplay, struct wl_surfac
            xdg_surface_add_listener(xdg_surface, &xdg_surface_listener, NULL);

            wl_surface_commit(*wlSurface);
        +
        +    wl_display_roundtrip(*wlDisplay);
        +
            return 0;
        }

- Continue with the **step 3** in the build instructions provided in the repository to compile and run the application on the RZ/V2H RDK platform: `01_Head_count - Application File Generation <https://github.com/Ignitarium-Renesas/rzv_ai_apps/tree/e06af580066307e6fa348b134e096f896f786425/01_Head_count#application-file-generation>`_.

How to port and build Computermind corporation DRP-AI Demo App: `Computermind corporation - DRP-AI Demo App <https://github.com/ComputermindCorp/drp-ai-demo-app/tree/main>`_.
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Clone the `ComputermindCorp/drp-ai-demo-app` repository to your DRP-AI TVM toolchain docker.

.. code-block:: bash

    $ cd /drp-ai_tvm/data
    $ git clone https://github.com/ComputermindCorp/drp-ai-demo-app.git
    $ cd drp-ai-demo-app
    $ git checkout fe24fc8e8b94028f8cf60cd670e5df12665ed2d9

The following step by step guide shows how to port Computermind corporation DRP-AI Demo App `C01_river_area_monitoring <https://github.com/ComputermindCorp/drp-ai-demo-app/tree/fe24fc8e8b94028f8cf60cd670e5df12665ed2d9/C01_river_area_monitoring>`_ as an example.

You can follow the same steps to port other Computermind corporation DRP-AI Demo Apps available in the repository.

- Update the ``drp-ai-demo-app/C01_river_area_monitoring/src/CMakeLists.txt`` file:

    **Before change (line 20-34 of CMakeLists.txt file):**

    .. code-block:: cmake

        if(PRODUCT STREQUAL "V2H")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v230/*.h)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland_scarthgap.cpp)
            add_definitions(-DV2H)
            message("-- PRODUCT [V2H]")
        elseif(PRODUCT STREQUAL "V2N")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.h)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.22.12/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- PRODUCT [V2N]")
        endif()

    **After modification:**

    .. code-block:: cmake
        :emphasize-lines: 1,4,6

        if(PRODUCT STREQUAL "V2H")
            include_directories(${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251)
            file(GLOB SOURCE *.cpp *.h ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.cpp ${CMAKE_SOURCE_DIR}/drp-ai_tvm_v251/*.h)
            include_directories(${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl)
            list(REMOVE_ITEM SOURCE ${CMAKE_SOURCE_DIR}/wayland.cpp)
            list(APPEND SOURCE ${CMAKE_SYSROOT}/usr/src/debug/gstreamer1.0-plugins-base/1.24.10/gst-libs/gst/gl/xdg-shell-client-protocol.c)
            add_definitions(-DV2H -DV2N)
            message("-- PRODUCT [V2H]")
        endif()

- Update the function ``initWaylandDisplay`` in the ``drp-ai-demo-app/C01_river_area_monitoring/src/wayland_scarthgap.cpp`` file, please add ``wl_display_roundtrip`` after ``wl_surface_commit`` as shown below:

    .. code-block:: diff
        :emphasize-lines: 9,10,11

        diff --git a/C01_river_area_monitoring/src/wayland_scarthgap.cpp b/C01_river_area_monitoring/src/wayland_scarthgap.cpp
        index 03af867..5c2ff08 100644
        --- a/C01_river_area_monitoring/src/wayland_scarthgap.cpp
        +++ b/C01_river_area_monitoring/src/wayland_scarthgap.cpp
        @@ -293,6 +293,9 @@ static int8_t initWaylandDisplay(struct wl_display** wlDisplay, struct wl_surfac
            xdg_surface_add_listener(xdg_surface, &xdg_surface_listener, NULL);

            wl_surface_commit(*wlSurface);
        +
        +    wl_display_roundtrip(*wlDisplay);
        +
            return 0;
        }

- Continue with the **step 3** in the build instructions provided in the repository to compile and run the application on the RZ/V2H RDK platform: `C01_river_area_monitoring - Application: Build Stage <https://github.com/ComputermindCorp/drp-ai-demo-app/tree/fe24fc8e8b94028f8cf60cd670e5df12665ed2d9/C01_river_area_monitoring#application-build-stage>`_.