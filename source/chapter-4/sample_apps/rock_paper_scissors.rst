.. _rock_paper_scissors:
.. _sample_app_rps:

Rock Paper Scissors
^^^^^^^^^^^^^^^^^^^

.. note::

   Available for :ref:`Foxglove <foxglove_visualization>` simulation environment without real robotic hardware!

.. figure:: ../../images/rps.png
   :align: center
   :alt: Rock Paper Scissors Demo
   :width: 600px

   Rock Paper Scissors Demo

The ``renesas_demo_rps`` (Rock Paper Scissors) package provides the following features:

- Rock-Paper-Scissors controller: detects rock, paper, and scissors gestures in real time, executes the game logic, and sends commands to control the robotic hand accordingly.
- Compatible with the Inspire RH56, Inspire RH56E2, and Ruiyan RH2 robotic hands.
- Supports RPS object detection and interpretation.
- Supports simultaneous control of virtual and physical dexterous hands.
- Supports a low-latency always-win mode with a selectable detector.
- Supports visualization through Foxglove Studio.
- Supports publishing a compressed H.264 video stream for low-bandwidth Foxglove viewing.

Quick hardware setup instructions
""""""""""""""""""""""""""""""""""

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`.

#. **Optional**: Connect the dexterous hand to the RZ/V2H RDK board if you want to control the real hand.

   .. note::

      Before using the Ruiyan RH2 Dexhand, ensure that the hand is properly initialized using the provided setup script located in ``ruiyan_rh2_hand_bringup/setup/ruiyan_rh2_init.sh`` or in ``install/ruiyan_rh2_hand_bringup/share/ruiyan_rh2_hand_bringup/setup/ruiyan_rh2_init.sh`` after installation.

#. Connect a compatible USB camera to the RZ/V2H RDK board for Rock Paper Scissors gesture detection.

Quick software setup instructions
"""""""""""""""""""""""""""""""""

.. note::

   All subsequent operations must be executed inside :ref:`the cross-compilation Docker container <development_guide>`, which was set up in the :ref:`common setup step <sample_apps_prerequisites>`.

#. Clone the required source from GitHub by using the ``vcs`` tool inside the Docker container.

   Get the ``ros2_demo_workspace`` repository first:

   .. code-block:: bash

      cd ~/ros2_ws
      git clone https://github.com/renesas-rdk/ros2_demo_workspace.git

   Import the repositories by using the ``vcs`` command:

   .. code-block:: bash

      vcs import < ./ros2_demo_workspace/vcs_manifests/rz-v2h/rock_paper_scissors.target.lock.repos

   It will clone all required repositories to the ``./src`` folder.

#. Cross-compile the ROS 2 workspace.

   Update the APT repository list in the target sysroot.

   .. code-block:: bash

      arm64-chroot apt update

   Install the dependencies to the target board first:

   .. code-block:: bash

      sysroot-rosdep-install

   It will take time if you run this command for the first time.

   Cross-build the application:

   .. code-block:: bash

      cross-colcon-build --packages-up-to renesas_demo_rps

#. Deploy the result to the board and install the runtime dependencies there, as described in
   :ref:`Deploying and Installing Dependencies <sample_apps_deploy>`.

Start the application
"""""""""""""""""""""

#. Application rules:

   - Similar to the traditional game.
   - The user initiates a game by showing the "HI" pose (scissors gesture) in front of the camera.
   - The robotic hand performs a 1-2-3 countdown to signal the start of the round.
   - When the countdown is finished, the player must show a chosen gesture (rock, paper, or scissors) within 2 seconds. If no gesture is detected within this time, the game is aborted.
   - After the player gives a choice, the robotic hand randomly selects and displays rock, paper, or scissors.
   - The game result is then displayed by the robotic hand using the following gestures: OK for a draw, thumbs up when the player wins, and victory when the player loses.
   - Wait 2 seconds after the result is shown before starting a new game.

   In always-win mode the robotic hand skips the random choice and immediately answers with the
   gesture that beats the detected player pose: ``paper`` beats ``rock``, ``scissor`` beats
   ``paper``, and ``rock`` beats ``scissor``. The controller publishes the ``ALWAYS_WIN`` status
   on ``/game_status``.

#. Load the workspace environment on the RZ/V2H RDK board.

   .. code-block:: bash

      cd /home/ubuntu/ros2_ws
      source /opt/ros/jazzy/setup.bash
      source ./install/setup.bash

#. Launch the Rock Paper Scissors application.

   To launch the virtual hands demo (without requiring hand hardware):

   .. code-block:: bash

      # For Inspire RH56 hand
      ros2 launch renesas_demo_rps demo_inspire_rh56_hand_rps.launch.py use_mock_hardware:=true

      # For Inspire RH56 hand + low-latency always win mode
      ros2 launch renesas_demo_rps demo_inspire_rh56_hand_rps_always_win.launch.py use_mock_hardware:=true

      # For Inspire RH56E2 hand
      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps.launch.py use_mock_hardware:=true

      # For Inspire RH56E2 hand + low-latency always win mode
      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps_always_win.launch.py use_mock_hardware:=true

      # For Ruiyan RH2 hand
      ros2 launch renesas_demo_rps demo_ruiyan_rh2_hand_rps.launch.py use_mock_hardware:=true

      # For Ruiyan RH2 hand + low-latency always win mode
      ros2 launch renesas_demo_rps demo_ruiyan_rh2_hand_rps_always_win.launch.py use_mock_hardware:=true

   To launch the physical Inspire RH56 hand control demo:

   .. code-block:: bash

      ros2 launch renesas_demo_rps demo_inspire_rh56_hand_rps.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0

   To launch the low-latency always-win Inspire RH56 demo:

   .. code-block:: bash

      # With YOLOX (default)
      ros2 launch renesas_demo_rps demo_inspire_rh56_hand_rps_always_win.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0

      # With YOLOv8
      ros2 launch renesas_demo_rps demo_inspire_rh56_hand_rps_always_win.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0 detector:=yolov8

   To launch the physical Inspire RH56E2 hand control demo:

   .. code-block:: bash

      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0

   To launch the low-latency always-win Inspire RH56E2 demo:

   .. code-block:: bash

      # With YOLOX (default)
      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps_always_win.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0

      # With YOLOv8
      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps_always_win.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0 detector:=yolov8

   To launch the low-latency always-win Inspire RH56E2 demo with compressed video streaming:

   .. code-block:: bash

      # With YOLOX (default)
      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps_always_win_r365.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0 enable_whip:=false

      # With YOLOv8
      ros2 launch renesas_demo_rps demo_inspire_rh56e2_hand_rps_always_win_r365.launch.py use_mock_hardware:=false video_device:=/dev/video0 serial_port:=/dev/ttyUSB0 detector:=yolov8 enable_whip:=false

   This launch file keeps the same always-win control path and publishes the overlaid camera feed
   as a compressed H.264 stream on ``/hand_camera/compressed_video``, which the Foxglove layout
   reads instead of the raw image topic. Encoding happens on the hardware encoder, so the board
   sends far less data to Foxglove than a raw stream.

   .. important::

      Pass ``enable_whip:=false``. The launch file otherwise also tries to push the encoded stream
      to a network uplink that is not part of the RDK.

   To launch the physical RuiYan RH2 hand control demo:

   .. code-block:: bash

      ros2 launch renesas_demo_rps demo_ruiyan_rh2_hand_rps.launch.py use_mock_hardware:=false video_device:=/dev/video0 can_interface:=can2

   To launch the low-latency always-win RuiYan RH2 demo:

   .. code-block:: bash

      # With YOLOX (default)
      ros2 launch renesas_demo_rps demo_ruiyan_rh2_hand_rps_always_win.launch.py use_mock_hardware:=false video_device:=/dev/video0 can_interface:=can2

      # With YOLOv8
      ros2 launch renesas_demo_rps demo_ruiyan_rh2_hand_rps_always_win.launch.py use_mock_hardware:=false video_device:=/dev/video0 can_interface:=can2 detector:=yolov8

#. For simulation using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

   The input layout file for Foxglove Studio is located at
   ``renesas_demo_rps/config/foxglove/demo_rps.json`` inside the ROS 2 workspace.

   For always-win mode, the input layout file is located at
   ``renesas_demo_rps/config/foxglove/demo_rps_always_win.json``.

   For the compressed-video always-win launch file, import
   ``renesas_demo_rps/config/foxglove/demo_rps_always_win_r365.json`` instead. It uses
   ``/hand_camera/compressed_video`` for the camera panel.

Selecting the detector
""""""""""""""""""""""

The ``detector`` argument exists only in the always-win launch files. The plain ``demo_*_rps``
launch files are fixed to YOLOv8.

.. list-table::
   :header-rows: 1
   :widths: 26 37 37

   * - ``detector``
     - Executable (``rzv_object_detection``)
     - Model type
   * - ``yolox`` (default)
     - ``yolox_rps_detection``
     - ``yolox_s_rps``
   * - ``yolov8``
     - ``yolov8_object_detection``
     - ``yolov8_rps``

Launch arguments
""""""""""""""""

The following table lists the launch arguments accepted by the demo launch files:

.. list-table::
   :header-rows: 1
   :widths: 34 46 20

   * - Argument
     - Meaning
     - Default
   * - ``video_device``
     - Camera device node used for gesture detection.
     - ``/dev/video0``
   * - ``serial_port``
     - Serial port of the physical Inspire RH56 or RH56E2 hand.
     - ``/dev/ttyUSB0``
   * - ``can_interface``
     - CAN interface of the physical Ruiyan RH2 hand.
     - ``can2``
   * - ``hand_speed``
     - Target motor speed for all joints, 0 to 1000.
     - ``1000``
   * - ``hand_side``
     - Which hand to control, ``left`` or ``right``.
     - ``left`` (``right`` in the compressed-video launch file)
   * - ``use_mock_hardware``
     - Set to ``true`` to run in simulation without physical hardware.
     - ``true``
   * - ``detector``
     - AI detector used by the always-win launch files, ``yolox`` or ``yolov8``.
     - ``yolox``

The arguments below exist only in the compressed-video launch file,
``demo_inspire_rh56e2_hand_rps_always_win_r365.launch.py``:

.. list-table::
   :header-rows: 1
   :widths: 34 46 20

   * - Argument
     - Meaning
     - Default
   * - ``camera_image_qos_reliability``
     - Reliability of the camera publisher.
     - ``best_effort``
   * - ``enable_whip``
     - Network uplink for the encoded stream. Set it to ``false`` on the RDK.
     - ``true``
   * - ``compressed_video_topic``
     - Topic carrying the encoded H.264 stream as ``sensor_msgs/msg/CompressedImage``.
     - ``/hand_camera/compressed_video``
   * - ``target_bitrate``
     - H.264 encoder target bitrate, in bits per second.
     - ``2000000``
   * - ``framerate``
     - Target camera capture frame rate.
     - ``30/1``
   * - ``max_input_framerate``
     - Input frame-rate limiter applied before encoding. ``0.0`` disables limiting.
     - ``0.0``
   * - ``overlay_max_annotation_age_ms``
     - Maximum age of an annotation still drawn on the overlay, in milliseconds.
     - ``1000``
   * - ``overlay_anti_aliasing``
     - Enable anti-aliasing when rendering the overlay.
     - ``false``
   * - ``overlay_render_text``
     - Render annotation text on the overlay.
     - ``true``

For more details about the Rock Paper Scissors application, refer to the
`README.md in the renesas_demo_rps package <https://github.com/renesas-rdk/renesas_demo_rps>`_.

- v1.0.0 (2026-03-31): Initial release of the Rock Paper Scissors sample application.
- v1.1.0 (2026-05-31): Added support for the RH56E2 Dexhand, always win mode, and ported the application to ``ros2_control`` framework for improved performance and flexibility.
- v1.2.0 (2026-09-10): Added the compressed-video streaming launch file and moved the package to a build-time platform selection.
