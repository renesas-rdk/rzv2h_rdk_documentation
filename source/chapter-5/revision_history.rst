Revision History
----------------

+-----------+---------------+----------------------------------------------------------------+
| Revision  | Date          | Description                                                    |
+===========+===============+================================================================+
| 1.3.0     | Oct 06, 2026  | - Update the Linux kernel to version 6.18.20.                  |
|           |               | - Add support for the RZ/V2H RDK ver101 (8 GB) board; add the  |
|           |               |   Board Versions section and board-specific images and IPL     |
|           |               |   files.                                                       |
|           |               | - Update the build tool (rz-utils) to build the IPL (TF-A and  |
|           |               |   U-Boot) for each board version and Multi-OS mode, and add    |
|           |               |   the PREEMPT_RT kernel build option.                          |
|           |               | - Add remoteproc support for loading CM33/CR8 firmware from    |
|           |               |   Linux, and add the RTOS-RTOS RPMsg examples.                 |
+-----------+---------------+----------------------------------------------------------------+
| 1.2.0     | Sep 11, 2026  | - Add three new sample applications: Dexterous Hand with       |
|           |               |   Tactile Sensors, Vision-Based Grasping, and Queen's Hand     |
|           |               |   Chess Robot.                                                 |
|           |               | - Use the common ``renesas_demo_*`` launch packages for the    |
|           |               |   demo applications.                                           |
+-----------+---------------+----------------------------------------------------------------+
| 1.1.1     | Jul 15, 2026  | - Add more USB Wi-Fi device support.                           |
|           |               | - Update section Cross-Compilation Environment Setup to        |
|           |               |   support multi-arch container; add Windows, macOS and         |
|           |               |   WSL2 setup guides.                                           |
|           |               | - Add workaround method for Known Issue 1.                     |
+-----------+---------------+----------------------------------------------------------------+
| 1.1.0     | May 31, 2026  | - Update Vision-Based Dexterous Hand and Rock Paper            |
|           |               |   Scissors application.                                        |
|           |               | - Update the cross-compilation process.                        |
+-----------+---------------+----------------------------------------------------------------+
| 1.0.0     | Mar 31, 2026  | - Initial release.                                             |
+-----------+---------------+----------------------------------------------------------------+