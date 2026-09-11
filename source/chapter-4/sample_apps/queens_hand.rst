.. _sample_app_queens_hand:

Queen's Hand Chess Robot
^^^^^^^^^^^^^^^^^^^^^^^^

The `renesas_demo_queens_hand <https://github.com/renesas-rdk/renesas_demo_queens_hand>`_ package is the top-level launch, node, and configuration package
for the Queen's Hand chess-playing robot: an Agilex Piper arm fitted with a dexterous hand
physically plays chess against a human, driven by a BehaviorTree.CPP mission with Stockfish as the
game brain.

Perception runs ``rzv_chess_pieces_detection``: a 12-class YOLOv8 detector accelerated by the
DRP-AI IP turns the camera image into a Forsyth-Edwards Notation (FEN) board state. The nodes
exchange the board position as FEN throughout.

.. note::

   The demo has a hardware-free mode. ``behavior_bringup_mock.launch.py`` runs the whole stack
   with a mock arm and a mock perception node, so you can exercise the game logic and the behavior
   tree without an arm, a hand, or a camera.

Nodes
"""""

The package builds two C++ executables and installs five Python scripts:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Executable
     - Role
   * - ``chess_engine_node.py``
     - Authoritative game state, built on python-chess and Stockfish. Serves the ``/chess/``
       services for board setup, move validation, move update, best move, human-move detection,
       and Portable Game Notation (PGN) handling. Publishes the latched ``/chess/board_state``
       FEN and ``/chess/game_state``.
   * - ``chess_board_geometry_node``
     - Board geometry derived from the probe-teach calibration. Serves
       ``/chess/get_square_pose``, ``/chess/get_capture_bin_pose``, ``/chess/get_promotion_pose``,
       and ``/chess/get_piece_height``.
   * - ``chess_game_manager_node``
     - Game control; drives the behavior-tree engine lifecycle. Serves ``/chess/game_control`` and
       publishes the latched ``/chess/game_paused`` and ``/chess/game_control_result``.
   * - ``chess_opponent_node.py``
     - Software Stockfish opponent for the software-only demo. Serves ``/chess/opponent/enable``.
   * - ``mock_perception_node.py``
     - Camera and detector replacement for hardware-free runs. Serves
       ``/chessboard/detect_board_state``, ``/get_detected_piece``, and ``/mock/play_human_move``.
   * - ``board_teach_calibration_node.py``
     - Interactive probe-teach board calibration. Writes ``calibrated_board.yaml``.
   * - ``chess_move_client.py``
     - Command-line helper with the ``move``, ``human``, ``best``, ``state``, ``reset``, ``pause``,
       and ``resume`` subcommands.

Run the Python nodes with the ``.py`` suffix, for example
``ros2 run renesas_demo_queens_hand chess_move_client.py state``.

Launch Files
""""""""""""

The following table lists the launch files and what each one brings up:

.. list-table::
   :header-rows: 1
   :widths: 44 56

   * - Launch file
     - Purpose
   * - ``behavior_bringup.launch.py``
     - Real hardware: the execute-layer servers, chess game logic, board geometry, game manager,
       and behavior-tree engine. Run the robot bringup and the perception launch separately.
   * - ``behavior_bringup_mock.launch.py``
     - Hardware-free: the same stack with a ros2_control mock arm and ``mock_perception_node``
       instead of the camera and detector.
   * - ``chess_vs_stockfish.launch.py``
     - Software only, with no arm at all: the engine, the Stockfish opponent, the board renderer,
       and Foxglove.
   * - ``chess_perception_realsense_camera.launch.py``
     - RealSense camera, ``rzv_chess_pieces_detection``, the board renderer, the Foxglove bridge,
       and the overlays. Run this alongside the real bringup.
   * - ``board_teach_calibration.launch.py``
     - Probe-teach calibration of the physical board.

Foxglove layouts live in ``config/foxglove/``: ``chess_demo.json``, ``game_play.json``, and
``renesas_demo_queens_hand.json``.

Hardware Setup
""""""""""""""

The demo uses the following hardware:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Item
     - Purpose
   * - Agilex Piper 6-DOF arm
     - Moves the pieces. Connects through a USB-to-CAN adapter.
   * - Dexterous hand
     - Grips the pieces. The Ruiyan RH2 is recommended for this demo; the Inspire RH56E2 also
       works. Connects through a second USB-to-CAN adapter.
   * - Intel RealSense depth camera
     - Sees the board. Configured by ``config/realsense/realsense_config.yaml``: 1920x1080 at
       6 FPS color YUYV with manual exposure; depth, infrared, and point cloud disabled.
   * - Chessboard and pieces
     - A standard 8x8 board, placed within the arm's reach and fully inside the camera's view.
   * - Probe tool
     - Mounted on the arm for the board teach calibration described below.

.. note::

   This demo requires several 3D-printed parts. Download the STL files from the
   `robot_printables GitHub repository <https://github.com/renesas-rdk/robot_printables>`_
   and print them before starting.

Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`
first. Mount the camera so that all 64 squares are visible and unobstructed, and place the board
so that the arm can reach every square. Both calibrations below assume that neither the board nor
the camera moves afterwards.

.. important::

   Run the steps in this order. Each one depends on the previous:

   #. Calibrate :ref:`the board to the robot <queens_hand_board_calibration>` once, and again
      whenever the board moves relative to the arm.
   #. Calibrate :ref:`the camera to the board <queens_hand_camera_calibration>` once, and again
      whenever the board or the camera moves.
   #. Bring up the robot.
   #. Start perception.
   #. Start the demo stack.

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

      vcs import < ./ros2_demo_workspace/vcs_manifests/rz-v2h/queens_hand.target.lock.repos

   Every repository the demo needs is cloned into the ``src/`` folder of the workspace, each
   pinned to the revision the manifest locks.

#. Install the demo's build dependencies into the target sysroot:

   .. code-block:: bash

      arm64-chroot apt update
      sysroot-rosdep-install

#. Cross-compile the workspace:

   .. code-block:: bash

      cross-colcon-build --packages-up-to renesas_demo_queens_hand

#. Deploy the result to the board and install the runtime dependencies there, as described in
   :ref:`Deploying and Installing Dependencies <sample_apps_deploy>`.

#. Install the non-ROS runtime dependencies on the RZ/V2H RDK board. The game logic needs the
   Stockfish engine and the python-chess library, which ``rosdep`` does not provide:

   .. code-block:: bash

      bash ./install/renesas_demo_queens_hand/share/renesas_demo_queens_hand/setup/install_dependencies.sh

   Run this once per board.

.. _queens_hand_board_calibration:

Calibrating the Board to the Robot
""""""""""""""""""""""""""""""""""

The arm needs to know where each square is in its own ``base_link`` frame. That mapping comes from
a probe-teach calibration whose output is a YAML file holding the 64 square centroids. The
geometry node reads it through its ``calibration_file`` parameter, which points at
``config/calibrated_board.yaml`` by default.

If the board has not moved since the shipped calibration was taken, skip this section.

#. Mount the probe tool on the Piper arm.

#. Start the calibration node:

   .. code-block:: bash

      ros2 launch renesas_demo_queens_hand board_teach_calibration.launch.py

   It accepts ``can_interface`` (default ``can2``), ``speed`` (default ``50``),
   ``use_mock_hardware`` (default ``false``), and ``output_file`` (default
   ``/tmp/calibrated_board.yaml``).

#. Switch the arm to teaching mode. Enable the hand's teaching mode first, then the arm's.

#. Move the tool center point (TCP) to the center of each square in turn. The node records each
   centroid from the ``base_link`` to TCP transform.

#. Read the result from ``/tmp/calibrated_board.yaml``, where the node writes it. Override the
   path with the ``output_file`` argument to write it elsewhere.

#. Copy the file into the package's ``config/`` directory and rebuild to make it the default. The
   demo stack also picks up on-disk changes to the file automatically.

.. caution::

   Teaching mode leaves the arm in a state that has to be cleared. Power the Piper arm off and
   back on before running the verification below or starting the demo. The Piper arm may need to
   be recalibrated after a power cycle.

Verifying the Board Calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The verification drives the arm to every square in turn, so you can see whether the probe lands
on the square centers.

#. Copy the calibration file into your workspace, inside the cross-build container:

   .. code-block:: bash

      cd ~/ros2_ws/src/apps/renesas_demo_queens_hand/config
      scp ubuntu@<board_ip>:/tmp/calibrated_board.yaml .

#. Generate one probe pose per square. Adjust the paths to match your checkout:

   .. code-block:: bash

      python3 ~/ros2_ws/src/apps/renesas_demo_queens_hand/test/calculate.py \
        ~/ros2_ws/src/apps/renesas_demo_queens_hand/config/calibrated_board.yaml > cmd_probe.txt

#. Launch the arm with the probe end effector at a low speed:

   .. code-block:: bash

      ros2 launch agilex_piper_arm_bringup agilex_piper_native_cartesian_control.launch.py \
        end_effector:=probe speed:=10

#. In a second terminal, send each pose from ``cmd_probe.txt``, replacing the ``pose:`` block with
   that square's entry:

   .. code-block:: bash

      ros2 topic pub --once /agilex_piper_gpio_controller/target_pose geometry_msgs/msg/PoseStamped "
      {
        header: {frame_id: 'base_link'},
        pose: {
          position: {x: 0.2500, y: 0.0261, z: -0.0200},
          orientation: {x: -0.0520, y: 0.9986, z: 0.0000, w: 0.0000}
        }
      }"

The arm should stop at each square center. If it does not, repeat the calibration.

.. _queens_hand_camera_calibration:

Calibrating the Camera to the Board
"""""""""""""""""""""""""""""""""""

The probe-teach file above holds the board geometry in the *robot's* frame. The detector needs a
second calibration: the center of every square in the *camera image*, which is what it fits the
image-to-board homography to before it can turn detections into a FEN. That file is
``/tmp/board_square_points.yaml``, written by ``chessboard_analyzer`` on the board.

Run it on the RZ/V2H RDK board itself, with the chessboard empty and the camera already in its
final position.

#. Start the analyzer:

   .. code-block:: bash

      ros2 launch chessboard_analyzer camera_chessboard_perspective_transformation.launch.py

#. Once the camera is streaming, trigger the calculation from a second terminal:

   .. code-block:: bash

      ros2 service call /chessboard_analyzer/trigger_calculate_squares std_srvs/srv/Trigger

   A successful trigger writes the 64 square centers, keyed ``a1`` to ``h8`` with image-pixel and
   deprojected 3D positions, to ``/tmp/board_square_points.yaml``. That is exactly the path the
   detector loads from, so nothing has to be moved afterwards. Check the analyzer's log for the
   success message before continuing.

#. Stop the calibration launch.

   .. important::

      The analyzer owns the RealSense camera, and the perception launch cannot open the camera
      while the analyzer still holds it. Pass ``shutdown_after_calibration:=true`` to have the
      launch shut itself down one second after a successful calibration.

The launch accepts the following arguments:

.. list-table::
   :header-rows: 1
   :widths: 32 32 36

   * - Argument
     - Default
     - Description
   * - ``robot_plays_as``
     - ``white``
     - Which side the robot plays.
   * - ``shutdown_after_calibration``
     - ``false``
     - Shut the launch down one second after a successful calibration.
   * - ``save_debug_images``
     - ``false``
     - Save the corner-detection debug images.
   * - ``debug_output_dir``
     - ``/tmp/chessboard_analyzer_debug``
     - Where those debug images are written.

.. caution::

   ``/tmp`` does not survive a reboot. Keep a copy of the file and put it back before starting the
   perception launch, otherwise redo the calibration:

   .. code-block:: bash

      # Right after calibrating
      cp /tmp/board_square_points.yaml ~/board_square_points.yaml

      # After a reboot
      cp ~/board_square_points.yaml /tmp/board_square_points.yaml

   The detector picks the file up as soon as it appears and re-reads it whenever it changes on
   disk, so restoring the copy also works while the perception stack is already running.

.. note::

   The ``camera_position`` setting in ``chessboard_analyzer``'s ``config/chess_board.yaml`` must
   match where the camera actually sits, because it alone decides which image corner becomes
   ``a1``:

   .. list-table::
      :header-rows: 1
      :widths: 24 40 36

      * - Value
        - Camera placement
        - ``a1`` lands at
      * - ``white``
        - At white's seat, 0 degrees
        - Image bottom-left
      * - ``side_left``
        - Beside the board, 90 degrees
        - Image bottom-right
      * - ``black``
        - At black's seat, 180 degrees
        - Image top-right
      * - ``side_right``
        - Beside the board, 270 degrees
        - Image top-left

   Pick the value whose ``a1`` corner matches your live image. A wrong ``camera_position``
   produces a board state that looks plausible but is rotated.

Running the Demo
""""""""""""""""

Hardware-Free Run
~~~~~~~~~~~~~~~~~

To exercise the game logic and the behavior tree without any hardware, run the mock bringup. The
arm and hand run on the ros2_control ``mock_components/GenericSystem`` interface, so you still get
real TF, ``/joint_states``, inverse kinematics, and trajectory streaming, but no physical motion.

.. code-block:: bash

   ros2 launch renesas_demo_queens_hand behavior_bringup_mock.launch.py

The launch emits only the CONFIGURE lifecycle transition, so start the game explicitly:

.. code-block:: bash

   ros2 service call /chess/game_control chess_interfaces/srv/GameControl "{command: start}"

The robot plays white and moves through the mock arm. Stage the human reply with:

.. code-block:: bash

   ros2 run renesas_demo_queens_hand chess_move_client.py human e7e5

.. note::

   The human move is staged on the mock node's ``/mock/play_human_move``, not on
   ``/chess/update_move``. The tree must still detect the move as a FEN difference and commit it
   itself, exactly as it does with a real camera.

The mock bringup accepts the following arguments:

.. list-table::
   :header-rows: 1
   :widths: 24 20 56

   * - Argument
     - Default
     - Description
   * - ``eef_control_mode``
     - ``no_force``
     - ``no_force`` for the Ruiyan RH2, ``force`` for the Inspire RH56E2. Also selects the
       parameter file and which mock arm bringup is included.
   * - ``launch_arm``
     - ``true``
     - Bring the mock arm up here. Set it to ``false`` if you start the arm bringup yourself in
       another terminal.
   * - ``initial_fen``
     - empty
     - Full FEN to start or resume from. Empty means the standard opening. Ignored when
       ``pgn_file`` is set.
   * - ``pgn_file`` / ``ply``
     - empty / ``-1``
     - Replay a PGN file up to ``ply`` half-moves to get the start position. ``-1`` replays the
       whole game. Sample games are in ``config/pgn/``.

Software-Only Game against Stockfish
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For a game with no robot dependencies at all, running only the engine, the Stockfish opponent, the
board renderer, and the Foxglove bridge:

.. code-block:: bash

   ros2 launch renesas_demo_queens_hand chess_vs_stockfish.launch.py

Make your own moves with:

.. code-block:: bash

   ros2 run renesas_demo_queens_hand chess_move_client.py move e2e4

The software-only launch accepts the following arguments:

.. list-table::
   :header-rows: 1
   :widths: 32 16 52

   * - Argument
     - Default
     - Description
   * - ``human_color``
     - ``white``
     - The side you play.
   * - ``self_play``
     - ``false``
     - Stockfish plays both sides.
   * - ``depth``
     - ``12``
     - Stockfish search depth.
   * - ``move_delay``
     - ``0.8``
     - Pause before Stockfish replies, in seconds.
   * - ``loop_games``
     - ``true``
     - Auto-restart games in self-play mode.
   * - ``launch_foxglove_bridge`` / ``foxglove_port``
     - ``true`` / ``8765``
     - Foxglove bridge toggle and port.

.. note::

   The software-only demo runs without the game manager, because there is no behavior-tree engine
   to drive. The Foxglove game-control buttons are inactive there; pause and resume the opponent
   with ``chess_move_client.py pause`` and ``resume`` instead.

Full Demo
~~~~~~~~~

Complete both calibrations, :ref:`board to robot <queens_hand_board_calibration>` and
:ref:`camera to board <queens_hand_camera_calibration>`, before starting the demo. Then bring the
stack up in this order, each in its own terminal.

#. **Bring up the robot.** Initialize the hand's USB-to-CAN adapter first, then launch the arm
   and the hand together, naming the interface of each adapter:

   .. code-block:: bash

      # Ruiyan RH2 hand: initialize its CAN interface once per power cycle
      cd ~/ros2_ws
      ./install/ruiyan_rh2_hand_bringup/share/ruiyan_rh2_hand_bringup/setup/ruiyan_rh2_init.sh

      ros2 launch piper_arm_ruiyan_hand_bringup piper_arm_ruiyan_hand_joint_position.launch.py \
        arm_can_interface:=can2 hand_can_interface:=can3

   ``can2`` and ``can3`` are the interfaces the two USB-to-CAN adapters enumerate as, not the
   onboard CAN-FD header. With both adapters plugged in, run ``ip link show | grep can`` to
   confirm which name belongs to the arm and which to the hand, and pass them accordingly.

#. **Start perception.**

   .. code-block:: bash

      ros2 launch renesas_demo_queens_hand chess_perception_realsense_camera.launch.py

   This starts the RealSense camera, the chess-piece detector with ``convert_to_fen: true``, a
   confidence threshold of 0.7 and an IoU threshold of 0.45, the board renderer, the Foxglove
   bridge, and the overlay nodes. The launch sets ``TVM_NUM_THREADS=3`` for DRP-AI inference.

   The detector reads the square centers from ``/tmp/board_square_points.yaml``, produced by
   :ref:`the camera-to-board calibration <queens_hand_camera_calibration>`. Until a valid
   calibration exists it warns and publishes no FEN; the piece detections themselves are
   unaffected.

   The launch accepts the following arguments:

   .. list-table::
      :header-rows: 1
      :widths: 28 26 46

      * - Argument
        - Default
        - Description
      * - ``robot_plays_as``
        - ``white``
        - Which side the robot plays. Sets the board renderer's orientation.
      * - ``model_type``
        - ``yolov8x_chess_pieces``
        - Detection model folder used by ``rzv_chess_pieces_detection``.

   .. note::

      The Foxglove camera panel shows the latched JPEG snapshot the detector publishes on
      ``/yolov8_chess_pieces_detection/inference_image/compressed``. It refreshes on each
      ``/chessboard/detect_board_state`` call rather than continuously.

#. **Start the demo stack.**

   .. code-block:: bash

      ros2 launch renesas_demo_queens_hand behavior_bringup.launch.py

   This starts the execute-layer servers, ``chess_engine_node``, ``chess_board_geometry_node``,
   ``game_manager_node``, and the behavior-tree engine. It accepts the same ``eef_control_mode``,
   ``initial_fen``, ``pgn_file``, and ``ply`` arguments as the mock bringup, but no
   ``launch_arm``.

#. **Start the game.** Nothing moves until the game manager activates the engine. Wait for the
   behavior-tree engine to report that it configured the stack:

   .. code-block:: text

      [behavior_tree_engine_node-7] [INFO] [1783596452.058739786] [bt_engine]: Configured stack: Chess Playing

   Then start the game:

   .. code-block:: bash

      ros2 service call /chess/game_control chess_interfaces/srv/GameControl "{command: start}"

   Open Foxglove to see the board state, the detected pieces, and the 3D pose of each square. Load
   the ``renesas_demo_queens_hand/config/foxglove/chess_demo.json`` layout and control the game
   from its buttons.

Game Control
~~~~~~~~~~~~

The game manager serves ``/chess/game_control`` (``chess_interfaces/srv/GameControl``) with the
commands ``start``, ``pause``, ``resume``, ``restart``, and ``stop``, and drives the ``bt_engine``
lifecycle accordingly. ``restart`` optionally takes a ``fen:`` field. The manager also publishes a
latched ``/chess/game_paused``, consumed by the tree's ``GamePauseGate``, and
``/chess/game_control_result``.

.. code-block:: bash

   ros2 service call /chess/game_control chess_interfaces/srv/GameControl "{command: pause}"
   ros2 service call /chess/game_control chess_interfaces/srv/GameControl "{command: resume}"
   ros2 service call /chess/game_control chess_interfaces/srv/GameControl "{command: stop}"

Pause takes effect at the next turn boundary. The Foxglove layouts provide service-call buttons
for every command.

Configuration
"""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - File
     - Contents
   * - ``config/params_ruiyan_rh2.yaml``, ``config/params_inspire_rh56e2.yaml``
     - Per-hand parameter files selected by ``eef_control_mode``. The ``bt_engine:`` section holds
       the engine settings and the nested ``pick_place_module``, ``grasp_bt_plugins``, and
       ``chess_bt_plugins`` parameters; the remaining top-level sections configure the
       execute-layer servers.
   * - ``config/board_poses.yaml``
     - Board layout, ``teach_squares``, and the capture-bin and promotion poses for the geometry
       node.
   * - ``config/calibrated_board.yaml``
     - Installed output of the probe-teach calibration: the 64 square centroids.
   * - ``config/realsense/realsense_config.yaml``
     - Camera stream configuration.
   * - ``config/pgn/``
     - Sample games for ``pgn_file``: ``kasparov_deep_blue_1997.pgn`` and
       ``simple_test_game.pgn``.

The launch appends ``chess_bt_plugins.initial_fen``, resolved from ``initial_fen`` or from
``pgn_file`` and ``ply``, after the parameter file, so it wins and stays settable at runtime for
restarts.

.. seealso::

   :ref:`Foxglove Visualization <foxglove_visualization>` for the general Foxglove setup.

For more details about the Queen's Hand Chess Robot application, refer to the
`README.md in the renesas_demo_queens_hand package <https://github.com/renesas-rdk/renesas_demo_queens_hand>`_.

- v1.0.0 (2026-09-10): Initial release of the Queen's Hand Chess Robot application.
