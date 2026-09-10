.. _sample_app_grasping:

Vision-Based Grasping
^^^^^^^^^^^^^^^^^^^^^

The `renesas_vision_based_grasping <https://github.com/renesas-rdk/renesas_vision_based_grasping>`_ package is the top-level launch and configuration package
for the vision-based pick-and-place demo: an Agilex Piper arm with a dexterous hand picks objects
detected by a RealSense camera and drops them in a bin, driven by a BehaviorTree.CPP mission.

Detection runs ``rzv_soft_objects_detection``, a YOLOX soft-object model accelerated by the
DRP-AI IP. Detections are published on ``/yolox_soft_objects_detection/bounding_box``.

The package wires together:

- RealSense D4xx camera bringup through ``realsense2_camera``.
- YOLOX soft-object detection on the DRP-AI IP.
- 3D object-pose extraction from the detections and the aligned depth image
  (``get_object_pose_server``).
- Arm motion servers for pose moves, trajectory planning, and speed control.
- Force-aware and position-only end-effector control servers.
- The behavior-tree engine, the pick-place module plugin, and the mission tree.
- A tree manager exposing start, pause, resume, restart, and stop control.
- Foxglove visualization: bounding-box overlays, an inference-timing overlay, and an H.264
  ``CompressedVideo`` stream of the color image.

The package contains no code of its own; its ``CMakeLists.txt`` installs ``launch/``, ``config/``,
and ``trees/`` only.

Launch Files
""""""""""""

The following table lists the launch files and what each one brings up:

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Launch file
     - Purpose
   * - ``perception_realsense_camera.launch.py``
     - RealSense camera, YOLOX detection through ``rzv_soft_objects_detection``, the Foxglove
       bridge, the bounding-box and inference-timing overlays, and H.264 color-image streaming.
   * - ``behavior_bringup.launch.py``
     - The execute-layer servers (end-effector, move-to-pose, arm speed, object pose, and
       trajectory planning), plus the tree manager and the behavior-tree engine.

Behavior Tree
"""""""""""""

``trees/MainTree.xml`` is the mission the behavior-tree engine loads. ``MainTree`` retries the
pick-and-place mission up to three times, then returns the arm home. If that also fails, it
force-returns home and reports failure.

The mission itself is a loop around one pick-and-place cycle:

.. code-block:: text

   PauseGate -> MoveToHomePhase -> PerceptionAndCompute
             -> PickWithRecovery -> PlaceIfPicked

- ``PauseGate`` sits at the top of the loop body. It subscribes to the tree manager's latched
  ``/bt/tree_paused`` topic and returns RUNNING while paused, so a ``pause`` command holds the
  mission at the start of the next cycle.
- Perception retries indefinitely until a stable detection, then computes the pick, place, and
  rescan poses.
- ``PickWithRecovery`` never fails the loop because of one bad object. On an end-effector failure
  it rescans once and retries the pick; if that also fails the object is skipped. A motion failure
  or an unknown error aborts the loop.
- ``PlaceIfPicked`` runs the place phase only when the pick succeeded. A skipped object goes
  straight home and the loop continues.

The ``MoveToHomePhase``, ``PickPhase``, ``PlacePhase``, and ``MoveToRescan`` subtrees come from
the ``pick_place_module`` plugin, not from this file.

Hardware Setup
""""""""""""""

.. note::

   This demo requires several 3D-printed parts. Download the STL files from the
   `robot_printables GitHub repository <https://github.com/renesas-rdk/robot_printables>`_
   and print them before starting.

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`.

#. Connect an Intel RealSense D4xx depth camera to the RZ/V2H RDK board.

#. Set up the Agilex Piper arm and the dexterous hand using the instructions in their bringup
   packages. The arm and a Ruiyan RH2 hand each attach through their own USB-to-CAN adapter; an
   Inspire hand attaches through a USB-to-serial adapter.

#. Place the objects to be picked within the camera's field of view and the arm's reach, and put
   the drop-off bin in reach as well.

Quick Software Setup Instructions
"""""""""""""""""""""""""""""""""

.. note::

   Run every command below inside the cross-compilation Docker container set up in
   :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`.

#. Get the ``ros2_demo_workspace`` repository, which carries the manifest for each demo:

   .. code-block:: bash

      cd ~/ros2_ws
      git clone https://github.com/renesas-rdk/ros2_demo_workspace.git

#. Import the repositories this demo needs with the ``vcs`` tool:

   .. code-block:: bash

      vcs import < ./ros2_demo_workspace/vcs_manifests/rz-v2h/vision_based_grasping.target.lock.repos

   Every repository the demo needs is cloned into the ``src/`` folder of the workspace, each
   pinned to the revision the manifest locks.

#. Install the demo's build dependencies into the target sysroot:

   .. code-block:: bash

      arm64-chroot apt update
      sysroot-rosdep-install

#. Cross-compile the workspace. ``renesas_vision_based_grasping`` is hardware-neutral, so its
   dependencies do not pull in an arm-and-hand assembly. Build the application together with the
   assembly you are going to run.

   For the Piper arm and an Inspire RH56E2 hand:

   .. code-block:: bash

      cross-colcon-build --packages-up-to \
        renesas_vision_based_grasping \
        piper_arm_inspire_rh56e2_hand_bringup

   For the Piper arm and a Ruiyan RH2 hand:

   .. code-block:: bash

      cross-colcon-build --packages-up-to \
        renesas_vision_based_grasping \
        piper_arm_ruiyan_hand_bringup \
        ruiyan_rh2_hand_bringup

   ``ruiyan_rh2_hand_bringup`` is listed on its own because it installs the ``ruiyan_rh2_init.sh``
   script that initializes the hand's USB-to-CAN adapter.

#. Deploy the result to the board and install the runtime dependencies there, as described in
   :ref:`Deploying and Installing Dependencies <sample_apps_deploy>`.

Running the Demo
""""""""""""""""

Source the workspace, then bring the stack up in this order, each part in its own terminal.

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

#. **Bring up the robot.** Launch the arm and the hand together, using the assembly you built.

   For the Piper arm and an Inspire RH56E2 hand, which attaches through a USB-to-serial adapter:

   .. code-block:: bash

      ros2 launch piper_arm_inspire_rh56e2_hand_bringup \
        piper_arm_inspire_rh56e2_hand_joint_position.launch.py \
        use_mock_hardware:=false camera_mode:=eye_in_hand \
        arm_can_interface:=can2 arm_speed:=40 serial_port:=/dev/ttyUSB0

   For the Piper arm and a Ruiyan RH2 hand, initialize the hand's USB-to-CAN adapter once per
   power cycle first, then launch:

   .. code-block:: bash

      cd ~/ros2_ws
      ./install/ruiyan_rh2_hand_bringup/share/ruiyan_rh2_hand_bringup/setup/ruiyan_rh2_init.sh

      ros2 launch piper_arm_ruiyan_hand_bringup \
        piper_arm_ruiyan_hand_joint_position.launch.py \
        use_mock_hardware:=false camera_mode:=eye_in_hand \
        arm_can_interface:=can2 arm_speed:=40 hand_can_interface:=can3

   ``can2`` and ``can3`` are the interfaces the USB-to-CAN adapters enumerate as, not the onboard
   CAN-FD header. With the adapters plugged in, run ``ip link show | grep can`` to confirm which
   name belongs to the arm and which to the hand, and pass them accordingly. The launch asks for
   your password so it can bring up the arm's CAN interface.

#. **Start the behavior layer.** Match the end-effector mode to the hand you mounted:

   .. code-block:: bash

      # Inspire RH56E2: force-aware end-effector control, the default
      ros2 launch renesas_vision_based_grasping behavior_bringup.launch.py

      # Ruiyan RH2: plain position control
      ros2 launch renesas_vision_based_grasping behavior_bringup.launch.py \
        eef_control_mode:=no_force

   The launch accepts the following arguments:

   .. list-table::
      :header-rows: 1
      :widths: 24 20 56

      * - Argument
        - Default
        - Description
      * - ``eef_control_mode``
        - ``force``
        - Which end-effector server to launch for the mounted hand. ``force`` is the Inspire
          RH56E2 mode-1 force hold; ``no_force`` is plain position control, used by the Ruiyan
          RH2. Both advertise the same ``/gripper_action``, so the trees are hand-agnostic.
      * - ``params_file``
        - empty
        - Override path to a YAML parameter file. Empty uses ``config/params.yaml`` from
          ``renesas_vision_based_grasping``.

#. **Start perception.**

   .. code-block:: bash

      ros2 launch renesas_vision_based_grasping perception_realsense_camera.launch.py

   This starts the RealSense camera, the YOLOX detector with a confidence threshold of 0.7 and an
   IoU threshold of 0.45, the Foxglove bridge, the overlay nodes, and the H.264 streaming node.
   The launch sets ``TVM_NUM_THREADS=3`` for DRP-AI inference and accepts the following arguments:

   .. list-table::
      :header-rows: 1
      :widths: 28 26 46

      * - Argument
        - Default
        - Description
      * - ``yolox_model_type``
        - ``yolox_soft_objects``
        - Name of the soft-objects detection model folder.
      * - ``stream_image_topic``
        - ``/camera/camera/color/image_raw``
        - Raw color image topic compressed for Foxglove streaming.
      * - ``stream_compressed_topic``
        - empty
        - ``foxglove_msgs/CompressedVideo`` output topic. Empty derives
          ``<stream_image_topic>/compressed_video``.
      * - ``stream_framerate``
        - ``15/1``
        - Source frame rate as numerator/denominator. Match it to the RealSense color FPS.
      * - ``stream_bitrate``
        - ``4000``
        - H.264 target bitrate in kbps.
      * - ``stream_keyframe_interval``
        - ``6``
        - Maximum key-frame interval in frames. A lower value joins faster in Foxglove at the
          cost of bitrate.

#. **Start the mission.** Nothing moves until the tree manager activates the engine. Wait for the
   behavior-tree engine to report that it configured the stack:

   .. code-block:: text

      [behavior_tree_engine_node-7] [INFO] [xx.xx] [bt_engine]: Configured stack: Vision-Based Grasping Stack

   Then start the tree:

   .. code-block:: bash

      ros2 service call /bt/tree_control bt_interfaces/srv/TreeControl "{command: start}"

Controlling the Mission
~~~~~~~~~~~~~~~~~~~~~~~

``tree_manager_node`` serves ``/bt/tree_control`` (``bt_interfaces/srv/TreeControl``) with the
commands ``start``, ``pause``, ``resume``, ``restart``, and ``stop``, and drives the ``bt_engine``
lifecycle node accordingly. It also publishes a latched ``std_msgs/Bool`` on ``/bt/tree_paused``,
consumed by the tree's ``PauseGate``, and a latched result string on ``/bt/tree_control_result``.

.. code-block:: bash

   ros2 service call /bt/tree_control bt_interfaces/srv/TreeControl "{command: pause}"
   ros2 service call /bt/tree_control bt_interfaces/srv/TreeControl "{command: resume}"
   ros2 service call /bt/tree_control bt_interfaces/srv/TreeControl "{command: stop}"

Pause takes effect at the next pause gate; in-flight arm motion finishes first.

Configuration
"""""""""""""

``config/params.yaml`` is the single source of truth, and ``behavior_bringup.launch.py`` passes it
to every server and to the behavior-tree engine.

The ``bt_engine:`` section holds the engine settings and the nested plugin parameters:

- ``pick_place_module`` holds motion speeds and tolerances, gripper open and close positions,
  per-object per-finger ``force_threshold`` maps in grams for the force end-effector server's
  mode-1 hold, and the home pose.
- ``grasp_bt_plugins`` holds the action and service names, the workspace limits, the grasp
  geometry, the drop-bin pose, and the per-class grasp profiles.

Object class keys must match the ``yolox_soft_objects`` model names: ``carrot``, ``coke``,
``egg``, ``pp_cup``, and ``sponge``. Unknown classes fall back to ``default``.

``config/realsense/realsense_config.yaml`` holds the camera stream configuration: 640x480 at
15 FPS color YUYV plus depth, with aligned depth enabled and the point cloud and infrared streams
disabled.

Visualization
"""""""""""""

The perception launch starts ``foxglove_bridge`` on the board. Connect Foxglove Studio to:

.. code-block:: text

   ws://<board-ip>:8765

Import the ``config/foxglove/vision_base_grasping.json`` layout. Its image panel reads the H.264
``CompressedVideo`` stream on ``/camera/camera/color/image_raw/compressed_video``, and shows the
bounding-box overlay (``/bbox_visualization``) and the inference-timing overlay
(``/inference_timing_visualization``) as annotations on top of it. The layout also provides
service-call buttons for ``/bt/tree_control``. The stream carries the camera header stamps, so
the video lines up with the detection topics on the Foxglove timeline.

.. seealso::

   :ref:`Foxglove Visualization <foxglove_visualization>` for the general Foxglove setup.

Troubleshooting
"""""""""""""""

RealSense fails with ``VIDIOC_QBUF``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Errors such as the following usually mean that the ``uvcvideo`` module is using the DMA-BUF
allocator required by the generic USB camera GStreamer pipeline, which is incompatible with Intel
RealSense:

.. code-block:: text

   xioctl(VIDIOC_QBUF) failed Last Error: Invalid argument
   Failed to resolve request. Request: Z16 640x480

Check the active allocator:

.. code-block:: bash

   cat /sys/module/uvcvideo/parameters/allocators

If it reports ``1``, stop the perception launch and reload ``uvcvideo`` with the
RealSense-compatible allocator. Also update the persistent setting so that a reboot does not
restore the incompatible value:

.. code-block:: bash

   echo 'options uvcvideo allocators=0' | \
     sudo tee /etc/modprobe.d/rzv2h-uvcvideo.conf
   sudo modprobe -r uvcvideo
   sudo modprobe uvcvideo allocators=0
   cat /sys/module/uvcvideo/parameters/allocators

The final command must report ``0``. If ``modprobe -r`` reports that the module is in use, stop
every process using ``/dev/video*`` before retrying. Relaunch perception and confirm that both
streams produce frames:

.. code-block:: bash

   ros2 topic echo --once /camera/camera/color/image_raw
   ros2 topic echo --once /camera/camera/depth/image_rect_raw

A warning that ``~/.realsense-config.json`` is missing is harmless; the camera is configured by
this package's ``config/realsense/realsense_config.yaml``.

For more details about the vision-based grasping application, refer to the
`README.md in the renesas_demo_vision_based_grasping package <https://github.com/renesas-rdk/renesas_demo_vision_based_grasping>`_.

- v1.0.0 (2026-09-10): Initial release of the Vision-Based Grasping application.
