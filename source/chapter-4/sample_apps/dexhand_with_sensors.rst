.. _sample_app_dexhand_sensors:

Dexterous Hand with Tactile Sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`renesas_demo_dexhand_w_sensors <https://github.com/renesas-rdk/renesas_demo_dexhand_w_sensors>`_
extends the :ref:`Vision-Based Dexterous Hand <dexhand>` demo with a second camera, the
Renesas SSC tactile glove, and grip control that adapts to what the hand is about to pick up.

The demo wires together:

- DRP-AI hand landmark estimation (``rzv_pose_estimation``) and DRP-AI soft-object detection
  (``rzv_soft_objects_detection``).
- Inspire RH56E2 hand and SSC tactile glove bringup.
- Hand-landmark gripper teleoperation: your hand drives the gripper opening.
- Object-aware force-threshold and gripper-profile updates, so a sponge is gripped differently
  from an egg.
- Tactile human-touch gesture detection from the glove.
- Foxglove layouts and a companion panel extension.
- Optional H.264 compressed-video streaming for low-bandwidth Foxglove viewing.

Perception: Two Models, One Container
"""""""""""""""""""""""""""""""""""""

Both AI models run as composable nodes inside a single ``component_container_mt``, which lets them
share the in-process DRP-AI scheduler rather than contending as separate processes:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Stage
     - Role
   * - ``rzv_pose_estimation::HandLandmarkEstimation``
     - Finds the operator's hand in the hand-camera frame and produces the 21 keypoints.
   * - ``rzv_soft_objects_detection::YoloXObjectDetection``
     - Classifies the object in the second camera's frame.

The launch files set ``TVM_NUM_THREADS=2`` for DRP-AI inference. Because the hand-landmark
pipeline is the latency-sensitive half, the soft-object branch is throttled before it reaches the
models: ``soft_objects_camera_ai_fps`` defaults to ``2.0`` in the perception-only demo and
``soft_objects_camera_throttle_fps`` to ``10.0`` in the integrated demo.

Helper Nodes
~~~~~~~~~~~~

The following table lists the helper nodes the demo runs alongside the perception container:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Executable
     - Role
   * - ``object_force_threshold_setter``
     - Decodes the soft-object detections, picks the highest-confidence class above
       ``min_confidence``, and publishes a six-value force-threshold command.
   * - ``object_gripper_mapping_setter``
     - Decodes the same detections and publishes a grasp-profile name (``full_hand``,
       ``three_fingers``, or ``pinch``) for the gripper adapter to switch to.
   * - ``image_throttle_node``
     - Republishes an image stream at a capped frame rate, with explicit input and output QoS.

Force-threshold presets live in ``config/hand/object_force_thresholds.yaml``. The class keys must
match the names of the ``yolox_soft_objects`` model: ``carrot``, ``coke``, ``egg``, ``pp_cup``,
and ``sponge``.

The setter nodes behave as follows:

- Detections below ``min_confidence`` are ignored.
- The highest-confidence remaining detection is treated as the dominant class.
- An empty detection frame keeps the last applied threshold and mapping.
- Unknown classes fall back to the configured defaults.
- Force thresholds are clamped to 0 to 3000 grams per joint.
- Repeated identical classes and mapping filenames are deduplicated.

The object-aware nodes use generic topic names, ``detections`` and ``force_threshold_commands``,
which the launch files remap to the RH56E2 hand topics.

Launch Files
""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 40 38 22

   * - Launch file
     - Purpose
     - Foxglove layout
   * - ``integrated_demo.launch.py``
     - Full local demo: dual-camera perception, gripper teleoperation, object-aware force
       thresholds and gripper mapping, tactile gestures, hand and glove bringup, and the overlays.
     - ``integrated_demo.json``
   * - ``integrated_r365_streaming_demo.launch.py``
     - The integrated demo plus hardware-encoded H.264 streams published as Foxglove
       ``CompressedVideo`` topics.
     - ``integrated_r365_streaming_demo.json``
   * - ``dual_camera_dual_model_demo.launch.py``
     - Perception only: hand landmarks and soft-object detection in one AI container. No hand or
       glove bringup.
     - ``dual_cam_dual_model.json``
   * - ``hand_landmark_gripper_teleop_demo.launch.py``
     - Hand camera to hand landmarks to gripper retargeter to the RH56E2 gripper action adapter.
     - ``hand_landmark_gripper_teleop.json``
   * - ``object_force_threshold_demo.launch.py``
     - Soft-object camera to YOLOX detections to per-class force-threshold and gripper-mapping
       updates.
     - ``object_force_threshold.json``
   * - ``human_touch_gestures_demo.launch.py``
     - SSC tactile glove to the human-touch gesture detector to ``/tactile_gestures/event``.
     - ``human_touch_gestures.json``

Hardware Setup
""""""""""""""

The demo uses the following hardware:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Item
     - Purpose
   * - Inspire RH56E2 dexterous hand
     - The controlled hand. Connects through a USB-to-serial adapter, ``/dev/ttyUSB0`` by
       default.
   * - Renesas SSC tactile glove
     - Per-pad contact sensing, based on the RAA2S470X impedance sensor. Connects over SPI.
   * - USB camera 1 (hand camera)
     - Watches the operator's hand for landmark estimation.
   * - USB camera 2 (soft-object camera)
     - Watches the object to be grasped.

.. important::

   Both cameras are captured by ``v4l2_camera`` as uncompressed 640x480 YUY2 streams, and two of
   those do not fit on a single USB root hub. Plug the two cameras into ports on different
   root hubs to avoid error.

Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`
first. Note the device node of each camera; the launch files take them as separate arguments.

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

      vcs import < ./ros2_demo_workspace/vcs_manifests/rz-v2h/vision_based_dexterous_hand_with_sensors.target.lock.repos

   Every repository the demo needs is cloned into the ``src/`` folder of the workspace, each
   pinned to the revision the manifest locks.

#. Install the demo's build dependencies into the target sysroot:

   .. code-block:: bash

      arm64-chroot apt update
      sysroot-rosdep-install

#. Cross-compile the workspace:

   .. code-block:: bash

      cross-colcon-build --packages-up-to renesas_demo_dexhand_w_sensors

#. Deploy the result to the board and install the runtime dependencies there, as described in
   :ref:`Deploying and Installing Dependencies <sample_apps_deploy>`.

Running the Demo
""""""""""""""""

Source the workspace first:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

Full Integrated Demo
~~~~~~~~~~~~~~~~~~~~

Runs everything together: dual-camera perception, hand and glove bringup, gripper teleoperation,
object-aware force thresholds, tactile gestures, and the Foxglove overlays.

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors integrated_demo.launch.py \
     hand_video_device:=/dev/video0 \
     soft_objects_video_device:=/dev/video2

The launch defaults to the real hand and the real glove, with ``hand_side`` set to ``right``. To
run the same graph fully mocked, override both flags:

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors integrated_demo.launch.py \
     hand_use_mock_hardware:=true \
     glove_use_mock_hardware:=true

Compressed Video Streaming Demo
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``integrated_r365_streaming_demo.launch.py`` runs the integrated graph and adds two encoder
processes, one per camera:

- Channel 1: ``/hand_camera/image_raw`` to ``/hand_camera/compressed_video``
- Channel 2: ``/soft_objects_camera/image_raw_full_rate`` to
  ``/soft_objects_camera/compressed_video``

Each stream renders the latest AI annotations locally and encodes one H.264 stream with the
hardware encoder, which Foxglove reads as a ``CompressedVideo`` topic. Both camera feeds then
reach Foxglove at full frame rate for a fraction of the bandwidth a raw stream would need.

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors integrated_r365_streaming_demo.launch.py \
     hand_video_device:=/dev/video0 \
     soft_objects_video_device:=/dev/video2 \
     enable_ch1_whip:=false \
     enable_ch2_whip:=false

.. important::

   Pass ``enable_ch1_whip:=false`` and ``enable_ch2_whip:=false``. The launch file otherwise also
   tries to push each encoded stream to a network uplink that is not part of the RDK.

This launch file also sets ``FASTDDS_BUILTIN_TRANSPORTS=UDPv4`` to avoid Fast DDS shared-memory
lock issues with the larger streaming graph.

Perception Only
~~~~~~~~~~~~~~~

Brings up the two cameras and both AI components in one container, with its own
``foxglove_bridge``. Neither the hand nor the glove is started.

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors dual_camera_dual_model_demo.launch.py \
     hand_video_device:=/dev/video0 \
     soft_objects_video_device:=/dev/video2

Foxglove debug streams default to 10 FPS on ``/hand_camera/image_raw_debug`` and
``/soft_objects_camera/image_raw_debug``. Disable the target-side Foxglove bridge if another ROS 2
machine runs it:

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors dual_camera_dual_model_demo.launch.py \
     run_foxglove_bridge_on_target:=false

Gripper Teleoperation Only
~~~~~~~~~~~~~~~~~~~~~~~~~~

The default is a real hand camera with fully mocked hand and glove hardware.

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors hand_landmark_gripper_teleop_demo.launch.py \
     hand_video_device:=/dev/video0

For a real hand with a mocked glove:

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors hand_landmark_gripper_teleop_demo.launch.py \
     hand_video_device:=/dev/video0 \
     hand_use_mock_hardware:=false \
     glove_use_mock_hardware:=true \
     hand_serial_port:=/dev/ttyUSB0

The pipeline is:

.. code-block:: text

   USB camera
     -> hand_landmark_estimation
     -> hand_landmark_gripper_retargeter
     -> hand_gripper_action_adapter
     -> inspire_rh56e2_hand_joint_position_controller

Retargeting is tuned by ``gripper_max_width`` (default ``0.06``), ``pinch_open_ratio``
(``1.5``), ``pinch_close_ratio`` (``0.15``), and ``gripper_smooth_factor`` (``0.7``).

Object-Aware Force Threshold Only
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The default uses a real soft-object camera with a mocked hand and a mocked glove.

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors object_force_threshold_demo.launch.py \
     soft_objects_video_device:=/dev/video0

For a real hand with a mocked glove:

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors object_force_threshold_demo.launch.py \
     soft_objects_video_device:=/dev/video0 \
     use_mock_hardware:=false \
     hand_serial_port:=/dev/ttyUSB0

The pipeline is:

.. code-block:: text

   USB camera
     -> rzv_soft_objects_detection
        /soft_objects_detection/bounding_box
     -> object_force_threshold_setter
        /inspire_rh56e2_hand_force_threshold_controller/commands
     -> object_gripper_mapping_setter
        /set_grasp_profile

Tactile Gestures Only
~~~~~~~~~~~~~~~~~~~~~

Runs the glove and the human-touch gesture detector. The default uses a real glove and a mocked
hand, which is useful for exercising gesture detection without moving the physical hand.

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors human_touch_gestures_demo.launch.py

For a real hand and a real glove:

.. code-block:: bash

   ros2 launch renesas_demo_dexhand_w_sensors human_touch_gestures_demo.launch.py \
     hand_use_mock_hardware:=false \
     glove_use_mock_hardware:=false \
     hand_serial_port:=/dev/ttyUSB0

Watch the gesture stream:

.. code-block:: bash

   ros2 topic echo /tactile_gestures/event

With the glove mocked, you can drive fake contact bits by hand. The order is thumb, index, middle,
ring, pinky, palm:

.. code-block:: bash

   # Press the palm pad
   ros2 topic pub --rate 50 /fake_tactile_glove_contact std_msgs/msg/Float64MultiArray \
     "{data: [0,0,0,0,0,1]}"

   # Release everything
   ros2 topic pub --once /fake_tactile_glove_contact std_msgs/msg/Float64MultiArray \
     "{data: [0,0,0,0,0,0]}"

Gesture detection subscribes to ``/tactile_glove_state_broadcaster/names`` and ``/values``, and
publishes ``human_touch_gestures/GestureEvent`` messages. Set ``feedback_enabled:=false`` to keep
gesture detection running without the hand-motion feedback.

Common Launch Arguments
"""""""""""""""""""""""

The following table lists the launch arguments shared by the launch files above. Most of them are
passed through to ``inspire_rh56e2_hand_ssc_glove.launch.py``.

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Argument
     - Meaning
   * - ``hand_video_device`` / ``soft_objects_video_device``
     - Camera device nodes for the hand camera and the soft-object camera.
   * - ``use_mock_hardware``
     - Shared fallback for both the hand and the glove mock modes.
   * - ``hand_use_mock_hardware`` / ``glove_use_mock_hardware``
     - Per-component overrides. An empty value falls back to ``use_mock_hardware``.
   * - ``hand_side``
     - ``left`` or ``right``. The integrated launches default to ``right``, the focused demos to
       ``left``.
   * - ``hand_speed``
     - Hand motor speed, available in the integrated launches. Default ``500``.
   * - ``hand_serial_port``
     - USB-to-serial adapter of the Inspire RH56E2 hand. Default ``/dev/ttyUSB0``.
   * - ``glove_transport``
     - ``spi`` or ``serial``. Default ``spi``.
   * - ``glove_calibration_file``
     - Calibration YAML from ``config/hand/``.
   * - ``gripper_mapping``
     - Multi-profile gripper-to-joint mapping YAML holding all grasp profiles. Default
       ``rh56e2_gripper_joint_mapping.yaml``.
   * - ``landmark_model_type``
     - Hand landmark model. Default ``mediapipe_hand_landmark``.
   * - ``soft_objects_model_type``
     - Soft-object detection model. Default ``yolox_soft_objects``.
   * - ``camera_image_qos_reliability``
     - Camera publisher reliability. Default ``best_effort``.
   * - ``soft_objects_camera_throttle_fps`` / ``soft_objects_camera_ai_fps``
     - Frame-rate cap on the soft-object branch before the models and the Foxglove display.
   * - ``run_foxglove_bridge_on_target``
     - Start ``foxglove_bridge`` on the board. Default ``true``.

Streaming Launch Arguments
""""""""""""""""""""""""""

The arguments below exist only in the compressed-video launch file,
``integrated_r365_streaming_demo.launch.py``:

.. list-table::
   :header-rows: 1
   :widths: 40 26 34

   * - Argument
     - Default
     - Meaning
   * - ``enable_ch1_whip`` / ``enable_ch2_whip``
     - ``true`` / ``true``
     - Network uplink for each encoded stream. Set both to ``false`` on the RDK.
   * - ``ch1_compressed_video_topic`` / ``ch2_compressed_video_topic``
     - ``/hand_camera/compressed_video`` / ``/soft_objects_camera/compressed_video``
     - Foxglove compressed-video topics.
   * - ``ch1_target_bitrate`` / ``ch2_target_bitrate``
     - ``4194304``
     - H.264 target bitrate in bits per second.
   * - ``ch1_framerate`` / ``ch2_framerate``
     - ``30/1``
     - Target camera capture frame rate.
   * - ``ch1_max_input_framerate`` / ``ch2_max_input_framerate``
     - ``0.0``
     - Input frame-rate limiter applied before encoding. ``0.0`` disables limiting.
   * - ``overlay_max_annotation_age_ms``
     - ``1000``
     - Maximum age for reusing low-rate AI annotations on the full-rate encoded streams. ``-1``
       keeps the latest overlay indefinitely.
   * - ``overlay_anti_aliasing``
     - ``false``
     - Smoother but slower overlay drawing.
   * - ``overlay_render_text``
     - ``true``
     - Draw annotation text on the encoded streams.

Note that ``soft_objects_camera_ai_fps`` defaults to ``5.0`` here; the encoded video still uses
the full-rate camera topic.

Visualization
"""""""""""""

The demos start ``foxglove_bridge`` on the board by default. Connect Foxglove Studio to:

.. code-block:: text

   ws://<board-ip>:8765

The shipped layouts subscribe to the throttled ``_debug`` raw image topics, or to the
compressed-video topics in ``integrated_r365_streaming_demo.json``. Pointing panels at the
full-rate raw camera topics instead increases CPU and network load on the board noticeably.

Install the Companion Panel Extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The layouts use custom panels for the hand controls, the tactile pads, the gripper, and the force
thresholds. Install them once:

#. Open Foxglove Studio Desktop.
#. Go to **Settings** > **Extensions** > **Install from file**.
#. Select ``config/foxglove/renesasuxsst.foxglove-dexhand-panels-1.0.0.foxe``.
#. Restart Foxglove Studio, or reload the panel registry.

.. seealso::

   :ref:`Foxglove Visualization <foxglove_visualization>` for the general Foxglove setup.

For more details about the Dexterous Hand with Tactile Sensors application, refer to the
`README.md in the renesas_demo_dexhand_w_sensors package <https://github.com/renesas-rdk/renesas_demo_dexhand_w_sensors>`_.

- v1.0.0 (2026-09-10): Initial release of the Dexterous Hand with Tactile Sensors application.