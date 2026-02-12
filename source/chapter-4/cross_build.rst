Cross-build the ROS2 Application
------------------------------------------------

This section provides instructions on how to cross-build ROS2 applications for the Renesas RZ/V2H RDK platform.

**What is Cross-building?**

Cross-building is the process of compiling software on a host system to run on a different target system.

The advantages of cross-building is that it allows developers to build applications for embedded systems without needing to compile directly on the target device, which have limited resources causing longer build times.

In this case, we will be cross-building ROS2 applications on a development machine (host) to run on the Renesas RZ/V2H RDK board (target).

We provide two methods for cross-building ROS2 applications for the RZ/V2H RDK platform:

- **Cross-build with Yocto SDK**: Uses the Yocto-provided SDK for cross-compiling ROS 2 applications targeting the RZ/V2H platform.
- **Cross-build with QEMU Docker**: Utilizes a Docker container with QEMU emulation to create a cross-compilation environment for building ROS2 applications.

.. list-table:: Comparison Between Build Methods
   :header-rows: 1
   :widths: 25 40 35

   * - **Method**
     - **Advantages**
     - **Disadvantages**
   * - Yocto SDK
     - Very fast compilation.
     - Possible **ABI version mismatch error**,

       if package versions differ between Yocto SDK and Ubuntu.
   * - QEMU Docker
     - No **ABI mismatch**,

       as the container matches the target runtime.
     - **Slower build time** due to QEMU emulation.
   * - Native build
     - Simplest setup, directly compiles on the RZ/V2H target.
     - **Very slow build time** and requires sufficient storage and memory on the board.

Follow the respective sections for detailed instructions on each cross-build method.

.. tip::

   For most use cases, the **Cross-build with Yocto SDK** method is recommended for its speed and efficiency.

   But if you encounter :ref:`ABI mismatch errors <abi_mismatch>`, consider using the **Cross-build with QEMU Docker** method.

.. toctree::
   :maxdepth: 1

   cross_build_sdk
   cross_build_qemu