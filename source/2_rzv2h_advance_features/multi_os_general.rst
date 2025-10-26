Overview
-------------------

The RZ/V Multi-OS feature allows the RZ/V2H platform to run multiple operating systems concurrently.

This capability is particularly useful for applications that require separation of tasks, such as running a real-time operating system (RTOS) alongside a general-purpose OS like Linux

.. important::

   On our default IPL, the following features are enabled by default:

   - Remoteproc support

   - CM33 and CR8 invocation from U-Boot

.. attention::

   When using the J-Link debugger, ensure that the SW1-6 switch on the RZ/V2H RDK board is set to the "ON" position to enable JTAG debug functionality.

   TODO: Update the image with the SW1-6 is set to "ON".

Useful References
-----------------

The following documents and repositories are recommended for users who want to explore more about multi-OS integration and RZ/V FSP software development:

- `RZ/V Multi-OS Package v3.2.0 <https://www.renesas.com/en/software-tool/rzv-group-multi-os-package>`_: Official software package enabling Linux + FreeRTOS/BareMetal multi-OS solutions on RZ/V2H.
- `RZ/V2H Quick Start Guide for RZ/V Multi-OS Package <https://www.renesas.com/en/document/qsg/rzv2h-quick-start-guide-rzv-multi-os-package>`_: Step-by-step guide to integrate and configure Multi-OS environments.
- `Getting Started with RZ/V Flexible Software Package (FSP) <https://www.renesas.com/en/document/apn/rzv-getting-started-flexible-software-package>`_: Instructions for developing and managing applications using RZ/V FSP.
- `FSP Example Project Usage Guide <https://www.renesas.com/en/document/apn/rzv-getting-started-flexible-software-package>`_: Detailed examples demonstrating peripheral control, communication, and middleware setup.
- `micro_ros_renesas_demos <https://github.com/micro-ROS/micro_ros_renesas_demos/tree/jazzy>`_: Demonstrations and documentation for using Micro-ROS with Renesas platforms, including agent setup and communication examples.

We recommend reviewing these resources to gain a deeper understanding of multi-OS capabilities and how to effectively utilize them in your projects.

Requirement
-------------------

- Ubuntu or Windows machines that can install the e2Studio and Flexible Software Package (FSP) for the Renesas RZ/V MPU Series.

- Segger J-Link (**firmware version 7.96e**): JTAG debugger for flashing firmware and debugging.

- `RZ/V FSP version 3.1 <https://github.com/renesas/rzv-fsp/tree/v3.1.0>`_ installed on the development machine. Follow the instructions in the `RZ/V FSP Getting Started Guide <https://www.renesas.com/en/document/apn/rzv-getting-started-flexible-software-package>`_ to set up the FSP.

Note for integration
----------------------

The peripherals which are NOT enabled enter Module Standby Mode after Linux kernel is booted up. That means the peripherals used on CM33 side might stop working at that time.

To avoid such a situation, Multi-OS Package incorporates the patch below:

- ``0003-Set-OSTM-for-MCPU-and-RCPU.patch``

This patch prevents GTM used in RPMsg demo program from entering Module Standby Mode. If you have any other peripherals which you would like to stop entering Module Standby implicitly, please update the patch as shown below:

.. code-block:: bash

   diff --git a/drivers/clk/renesas/r9a09g057-cpg.c b/drivers/clk/renesas/r9a09g057-cpg.c
   index 0468eb17305b..146d5b444e8c 100644
   --- a/drivers/clk/renesas/r9a09g057-cpg.c
   +++ b/drivers/clk/renesas/r9a09g057-cpg.c
   @@ -302,14 +302,14 @@ static const struct rzv2h_mod_clk r9a09g057_mod_clks[] __initconst = {
      DEF_MOD("rcpu_cmtw1_clkm",		CLK_PLLCLN_DIV32, 4, 0, 2, 0),
      DEF_MOD("rcpu_cmtw2_clkm",		CLK_PLLCLN_DIV32, 4, 1, 2, 1),
      DEF_MOD("rcpu_cmtw3_clkm",		CLK_PLLCLN_DIV32, 4, 2, 2, 2),
   -	DEF_MOD("gtm_0_pclk",			CLK_PLLCM33_DIV16, 4, 3, 2, 3),
   -	DEF_MOD("gtm_1_pclk",			CLK_PLLCM33_DIV16, 4, 4, 2, 4),
   +	DEF_MOD_CRITICAL("gtm_0_pclk",			CLK_PLLCM33_DIV16, 4, 3, 2, 3),
   +	DEF_MOD_CRITICAL("gtm_1_pclk",			CLK_PLLCM33_DIV16, 4, 4, 2, 4),
      DEF_MOD("gtm_2_pclk",			CLK_PLLCLN_DIV16, 4, 5, 2, 5),
      DEF_MOD("gtm_3_pclk",			CLK_PLLCLN_DIV16, 4, 6, 2, 6),
   -	DEF_MOD("gtm_4_pclk",			CLK_PLLCLN_DIV16, 4, 7, 2, 7),
   -	DEF_MOD("gtm_5_pclk",			CLK_PLLCLN_DIV16, 4, 8, 2, 8),
   -	DEF_MOD("gtm_6_pclk",			CLK_PLLCLN_DIV16, 4, 9, 2, 9),
   -	DEF_MOD("gtm_7_pclk",			CLK_PLLCLN_DIV16, 4, 10, 2, 10),
   +	DEF_MOD_CRITICAL("gtm_4_pclk",			CLK_PLLCLN_DIV16, 4, 7, 2, 7),
   +	DEF_MOD_CRITICAL("gtm_5_pclk",			CLK_PLLCLN_DIV16, 4, 8, 2, 8),
   +	DEF_MOD_CRITICAL("gtm_6_pclk",			CLK_PLLCLN_DIV16, 4, 9, 2, 9),
   +	DEF_MOD_CRITICAL("gtm_7_pclk",			CLK_PLLCLN_DIV16, 4, 10, 2, 10),
      DEF_MOD("wdt_0_clkp",			CLK_PLLCM33_DIV16, 4, 11, 2, 11),
      DEF_MOD("wdt_0_clk_loco",		CLK_QEXTAL, 4, 12, 2, 12),
      DEF_MOD("wdt_1_clkp",			CLK_PLLCLN_DIV16, 4, 13, 2, 13),
   --

Please edit the ``r9a09g057-cpg.c`` file accordingly and re-build the Linux Image to ensure the desired peripherals remain active during Multi-OS operation.

We recommend using the eSDK to build the Linux image to do it. For more details, please refer to the :ref:`Custom Linux Kernel and Device Tree <build_kernel>`.

How to create the new project for RZ/V2H RDK
---------------------------------------------

To create a new project for the RZ/V2H RDK board using the RZ/V Flexible Software Package (FSP) and e2Studio, follow these steps:

1. Open e2Studio and create a new Renesas FSP project.
2. Choose **File** > **New** > **Renesas C/C++ Project** > **Renesas RZ** > **Renesas RZ/V C/C++ FSP Project**.
3. Enter the Project name and location.
4. Select the **Custom User Board (for RZ/V2H)**, the Device **R9A09G057H44GBG**, and the target core: **CM33 or CR8**.
5. If you select the CR8 core, please select the appropriate **Preceding Project/Smart Bundle Details**.

   This setup is necessary to ensure that there are no conflicts in hardware resource usage between the CM33 and CR8 cores when both are running on the RZ/V2H platform.

6. Configure the **Build artifact type, RTOS selection and Sub-core selection** as needed.
7. Configure the **Project Templete Selection**, and click on the **Finish** button to create the project.
8. Once the project is created, you can start adding your application code and configuring the necessary peripherals using the FSP.

We also provide a sample project for RZ/V2H RDK board with Multi-OS Package.

Please follow the next section to import the sample project.