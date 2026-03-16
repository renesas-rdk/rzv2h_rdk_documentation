RZ/V2H ROS 2 Software Stack
===========================

This section provides ROS 2 demo packages for computer vision applications on Renesas RZ/V MPU platforms, with a primary focus on the RZ/V2H.

The demos showcase AI-accelerated vision features such as object detection, pose estimation, and visualization.

.. note::

   Please follow the pages below to learn more about each demo and set it up on your own.

RZ/V2H ROS 2 Software Stack
---------------------------

The following image shows the software stack of the RZ/V2H RDK, which includes the available demo applications, AI model packages, DRP-AI support, and other related concepts.

.. figure:: ../images/sw_stack.png
   :alt: RZ/V2H Software Stack
   :width: 800px
   :align: center

   RZ/V2H Software Stack

.. note::

   Some packages are under development. Please stay tuned and get the latest updates from the `Renesas RDK GitHub repository <https://github.com/renesas-rdk>`_.

Available Demo Applications
---------------------------

- :ref:`Vision-Based Robotic Arm Teleoperation <arm_teleoperation>`

  Includes required hardware, launch instructions, input method, and control behavior.

- :ref:`Vision-Based Dexterous Hand <dexhand>`

  Includes setup steps for virtual and physical hand control, supported hardware, and usage notes.

- :ref:`Rock Paper Scissors Game <rock_paper_scissors>`

  Includes game launch steps, gesture requirements, and display behavior.

- :ref:`Static Object Detection <static_object_detection>`

  Includes model requirements, camera input setup, and object detection output details.

- :ref:`Static or Camera-Based Hand Landmark Estimation <hand_landmark>`

  Includes detection pipeline details, supported input modes, and landmark visualization behavior.