.. _drp_ai_deployment:

Validate and Profile Compiled Models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After you compile a model with the DRP-AI TVM extension package and construct the model configuration for deployment, you can check how the model behaves on the RZ/V2H RDK board before you integrate it into an application.

The **DRP-AI Deployment** tool is a development and validation toolchain for AI models compiled for the DRP-AI accelerator. It answers two questions about a compiled model:

- Does the model produce correct results?
- Is the model fast and light enough for the target?

One command on the development machine drives the whole flow. The tool transfers the code, the model files, and the images to the board, runs inference on the board, retrieves the results, and then scores and profiles them on the development machine.

.. note::

   This tool does not compile models. Compile the model first, as described in :ref:`DRP-AI Model Compilation Tutorials <tutorial_prerequisites>` and :ref:`How to Deploy Model Files <how_to_deploy_model_files>`.

Features
""""""""

- **Detection mode**: Runs a compiled model on one or more images and returns the predictions, the latency breakdown, and the rendered overlays.
- **Evaluation mode**: Runs a model on a ground-truth dataset and reports task-specific accuracy metrics.
- **Profiling**: Reports latency percentiles, effective frames per second (FPS), CPU utilization, and memory usage, including the model-load cost.
- **Multiple models**: Runs any number of models defined in one settings file in sequence, and generates a comparison table across runs.

Supported Models
""""""""""""""""

The following models are supported. They correspond to the models compiled in the :ref:`DRP-AI Model Compilation Tutorials <tutorial_prerequisites>`.

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Model name
     - Task
     - Description
   * - ``mediapipe``
     - ``keypoint_2d``
     - MediaPipe hand landmark
   * - ``hrnetv2``
     - ``keypoint_2d``
     - HRNetV2 hand keypoints
   * - ``yolox``
     - ``detection``
     - YOLOX hand detection
   * - ``yolov8_detect``
     - ``detection``
     - YOLOv8 rock-paper-scissors gesture detection
   * - ``yolov8_obb``
     - ``obb_detection``
     - YOLOv8 oriented bounding box (OBB) detection

To add your own model, see ``docs/ADDING_A_MODEL.md`` in the DRP-AI Deployment repository.

System Overview
"""""""""""""""

The tool spans two machines. Neither machine runs a service: the development machine starts every interaction over ``ssh`` and ``rsync``, and the board runs a batch process that starts, runs inference, and exits.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Machine
     - Responsibility
   * - Development machine
     - Runs ``drpai.py``. Holds the settings file, the ground-truth datasets, and all reports. Performs scoring and profiling report generation.
   * - RZ/V2H RDK
     - Runs inference on the DRP-AI accelerator with the MERA runtime, and measures on-board resource usage. The board never receives the ground-truth data.

Requirements
""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 16 42 42

   * - Item
     - Development machine
     - RZ/V2H RDK
   * - OS
     - Ubuntu 24.04 LTS
     - Ubuntu 24.04
   * - Python
     - 3.12
     - 3.12 (system Python of Ubuntu 24.04)
   * - Packages
     - None (standard library only)
     - ``mera``, ``mera2_runtime``, and ``tvm`` wheels, ``opencv-python==4.10.0.84``
   * - Tools
     - ``ssh``, ``rsync``
     - SSH server, ``rsync``, ``python3-venv``

The ``mera``, ``mera2_runtime``, and ``tvm`` wheels are provided in the ``how-to/python`` directory of `DRP-AI TVM v2.8.0-hotfix2 <https://github.com/renesas-rz/rzv_drp-ai_tvm/tree/v2.8.0-hotfix2/how-to/python>`_.

Set Up the Environment
""""""""""""""""""""""

Set Up the RZ/V2H RDK
~~~~~~~~~~~~~~~~~~~~~

On Ubuntu 24.04, the system Python is protected from direct ``pip`` installation (PEP 668). Install the vendor wheels into a virtual environment. The board must have internet access while ``pip`` installs the wheel dependencies.

1. Install the Python ``venv`` module and ``rsync``. ``rsync`` must be installed on both the development machine and the board. Skip this step if both packages are already installed.

   .. code-block:: bash

      # skip if already installed
      sudo apt update
      sudo apt install -y python3-venv rsync

2. Create the workspace directory and the virtual environment. Make sure that ``python3 --version`` reports Python 3.12.

   .. code-block:: bash

      mkdir -p ~/drpai_ws
      cd ~/drpai_ws
      python3 --version
      python3 -m venv .venv
      source .venv/bin/activate

3. Download the MERA and TVM wheels from DRP-AI TVM ``v2.8.0-hotfix2``.

   .. code-block:: bash

      TVM_REF=v2.8.0-hotfix2
      BASE=https://raw.githubusercontent.com/renesas-rz/rzv_drp-ai_tvm/${TVM_REF}/how-to/python
      for whl in \
          mera-2.6.0-cp312-cp312-linux_aarch64.whl \
          mera2_runtime-2.6.0-cp312-cp312-linux_aarch64.whl \
          tvm-0.7.0.dev1619+g57142b014-cp312-cp312-linux_aarch64.whl; do
        curl -fL -o "$whl" "$BASE/$whl"
      done

4. Install the wheels and OpenCV into the virtual environment.

   .. code-block:: bash

      pip install ./*.whl opencv-python==4.10.0.84

Record the absolute path of the virtual environment, for example ``/home/ubuntu/drpai_ws/.venv``. You set it as ``board.venv`` in the settings file.

.. note::

   A non-interactive SSH session does not activate the virtual environment. The tool can only find the virtual environment through ``board.venv``.

The workspace on the board has the following layout. You prepare only the virtual environment and the wheels. The tool creates the repository mirror, the model folders, and its work directory automatically.

.. code-block:: text

   /home/ubuntu/drpai_ws/
   ├── drp-ai-deployment/             mirrored repository
   ├── .venv/                         virtual environment
   ├── *.whl                          MERA / TVM wheels
   ├── mp_handlandmark_fhd_crop/      compiled model folders
   ├── hrnetv2_hands_fhd_crop/
   ├── yolox_hand_fhd_crop/
   ├── yolov8_rps/
   └── yolov8n_obb_soft_objects/

Set Up the Development Machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Clone the DRP-AI Deployment repository.

   .. TBD: replace with the public repository URL before release.

   .. code-block:: bash

      git clone <drp-ai-deployment_repository_url>
      cd drp-ai-deployment

2. Make sure that you can log in to the board over SSH, for example:

   .. code-block:: bash

      ssh ubuntu@<board_ip>

3. Copy the compiled model folders to the development machine, for example the folders generated in the ``$TVM_ROOT/data`` directory by the compilation tutorials.

Configure the Settings File
"""""""""""""""""""""""""""

The ``settings.json`` file in the repository root describes the board and the models to run. It has three top-level keys: ``board``, ``defaults``, and ``runs``.

.. code-block:: json

   {
     "schema_version": "drpai_run_config_v1",

     "board": {
       "host": "<board_ip>",
       "user": "ubuntu",
       "workspace": "/home/ubuntu/drpai_ws",
       "venv": "/home/ubuntu/drpai_ws/.venv"
     },

     "defaults": {
       "models_config": "models_config.yaml",
       "debug": false,
       "visualize": true,
       "evaluation": {
         "conf_thres": 0.25,
         "iou_thres": 0.50,
         "pck_thres": 0.05,
         "strict": true,
         "export_json": true,
         "export_csv": true,
         "warmup_samples": 1,
         "profiling": true
       }
     },

     "runs": [
       {
         "name": "yolox_eval",
         "model": "yolox",
         "model_key": "yolox_hand",
         "model_path": "<path_to>/yolox_hand_fhd_crop",
         "detection": {
           "images": ["datasets_ground_truth/yolox_hand_bbox/images/IMG_00000002.jpg"]
         },
         "evaluation": {
           "dataset": "datasets_ground_truth/yolox_hand_bbox",
           "iou_thres": 0.45
         }
       }
     ]
   }

Relative paths on the development machine are resolved from the directory of the settings file.

**board**

.. list-table::
   :header-rows: 1
   :widths: 20 12 68

   * - Key
     - Required
     - Description
   * - ``host``
     - Yes
     - Host name or IP address of the board.
   * - ``user``
     - Yes
     - SSH login user.
   * - ``workspace``
     - Yes
     - Absolute path of the board-side directory for the repository mirror, the model folders, and the work directory.
   * - ``venv``
     - No
     - Absolute path of the Python virtual environment on the board. If it is not set, the system ``python3`` is used.
   * - ``ssh_key``
     - No
     - Path of the SSH private key.
   * - ``password_env``
     - No
     - **Name** of the environment variable that holds the SSH password. Requires ``sshpass`` on the development machine.

**runs**

Each entry in ``runs`` defines one model run. The entries are executed in order.

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Key
     - Required
     - Description
   * - ``name``
     - Yes
     - Name of the report directory. It must be a single path component.
   * - ``model``
     - Yes
     - One of ``hrnetv2``, ``mediapipe``, ``yolov8_detect``, ``yolov8_obb``, or ``yolox``.
   * - ``model_path``
     - Yes
     - Host-side path of the compiled model folder. The folder is copied to ``<workspace>/<folder_name>/`` on the board.
   * - ``model_key``
     - No
     - Key in ``models_config.yaml`` that provides the label map and the tuning parameters. Used by the detection models.
   * - ``detection.images``
     - Detection mode
     - Host-side image paths.
   * - ``evaluation.dataset``
     - Evaluation mode
     - Ground-truth dataset folder.
   * - ``evaluation.*``
     - No
     - Overrides any key in ``defaults.evaluation`` for this run.

The repository provides a ground-truth dataset for each supported model in the ``datasets_ground_truth/`` directory.

For the complete settings reference, see ``docs/USAGE.md`` in the DRP-AI Deployment repository.

Run the Tool
""""""""""""

Run the following commands from the repository root on the development machine.

1. Check the configuration and the execution plan without connecting to the board.

   .. code-block:: bash

      python3 drpai.py --dry-run

2. Run detection, evaluation, and profiling for every configured model.

   .. code-block:: bash

      python3 drpai.py

To run only part of the flow, use the following options:

.. code-block:: bash

   python3 drpai.py --list                  # List the configured runs
   python3 drpai.py --mode detection        # Model output on images only
   python3 drpai.py --mode evaluation       # Accuracy and performance reports only
   python3 drpai.py --run yolox_eval        # One configured run
   python3 drpai.py --no-visualize          # No overlay rendering, for clean profiling numbers
   python3 drpai.py --keep-going            # Continue to the next run after a failure

Run ``python3 drpai.py --help`` for the complete list of options.

Check the Results
"""""""""""""""""

The reports are written to the ``reports/`` directory on the development machine.

.. code-block:: text

   reports/
   ├── summary.md                          comparison across runs
   └── <run_name>/
       ├── detection/
       │   ├── embedded_prediction_set.json  predictions, latency, and profiling data
       │   ├── board_stderr.log            board-side error output
       │   └── visualizations/             prediction overlays
       └── evaluation/
           ├── evaluation_summary.md       accuracy metrics
           ├── evaluation_summary.json
           ├── evaluation_per_sample.csv
           ├── profiling_report.md         latency, CPU, and memory
           ├── profiling_visualization.html
           ├── board_stderr.log
           └── visualizations/

The evaluation reports contain the following metrics:

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Task
     - Metrics
   * - Detection, OBB detection
     - Precision, recall, F1 score, mean intersection over union (IoU), mean average precision (mAP) at an IoU of 0.50, and mAP averaged over IoU values from 0.50 to 0.95
   * - 2D keypoints
     - Mean end-point error (EPE) in pixels, normalized mean error (NME), and percentage of correct keypoints (PCK)

.. note::

   ``iou_thres`` does not affect mAP. mAP at 0.50 always uses an IoU of 0.50, and mAP at 0.50 to 0.95 uses its own IoU range.

``profiling_visualization.html`` is a self-contained file. Open it in a web browser; no web server or network access is required.

Troubleshooting
"""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Symptom
     - Solution
   * - ``ModuleNotFoundError: mera`` on the board
     - Set ``board.venv`` to the absolute path of the virtual environment.
   * - Exit code ``2``: connection refused or permission denied
     - Check ``host``, ``user``, and the credentials. Confirm that ``ssh <user>@<host>`` works manually, and that ``workspace`` is writable.
   * - Exit code ``2``: ``sshpass: command not found``
     - Install ``sshpass`` with ``sudo apt install sshpass``, or use ``ssh_key`` instead of ``password_env``.
   * - Exit code ``3``: the worker exits during model load
     - Check ``board_stderr.log``. Confirm that the model folder is copied correctly and that it matches the runtime on the board.
   * - Exit code ``4``: ``Cannot match prediction image to GT sample``
     - Make sure that each image file name (without extension) is the same as its ``sample_id`` in the dataset.
   * - Empty detection results
     - The model-side ``score_threshold`` in ``models_config.yaml`` and ``conf_thres`` both filter predictions, and the higher value applies. Align the two values.

Add ``--verbose`` to display every remote command and the full ``rsync`` output.

For more information, see the following documents in the DRP-AI Deployment repository:

- ``docs/USAGE.md``: Settings file, command line, output, and troubleshooting.
- ``docs/CONTRACTS.md``: Dataset formats, metric definitions, and dataset preparation.
- ``docs/ARCHITECTURE.md``: Host and board architecture, measurement windows, and exit codes.
- ``docs/ADDING_A_MODEL.md``: How to add a custom model.
