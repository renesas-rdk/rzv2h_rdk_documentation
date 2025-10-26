.. _drp_ai_with_rzv_model_tutorials:

DRP-AI with rzv_model tutorials
---------------------------------

This section provides tutorials on how to use the **rzv_model** package to deploy AI models on the RZ/V2H platform with DRP-AI acceleration.

Make sure you have completed the steps in the :ref:`BYOM AI model support <byom_drp_ai>` section to set up the DRP-AI TVM extension package in your development machine.

.. _tutorial_prerequisites:

**Prequisites**

On the development machine, which has the DRP-AI TVM environment, clone the `Hand Models <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/ai/hand_models.git>`_ repository, which contains the necessary model files and conversion scripts for the tutorials.

.. code-block:: bash

    $ git clone https://partnergitlab.renesas.solutions/sst1/industrial/ws078/ai/hand_models.git

Then, set the hand models directory environment variable:

.. code-block:: bash

    $ export HAND_MODELS_DIR=<path_to_cloned_hand_models_directory>

Next, change to the TVM tutorials directory and copy the compilation script:

.. code-block:: bash

    $ cd $TVM_ROOT/tutorials
    $ cp ${HAND_MODELS_DIR}/compilation/compile_onnx_model_quant_<model>.py .

Follow the specific tutorial instructions below to convert and deploy different hand detection and landmark models using the **rzv_model** package.

.. toctree::
    :maxdepth: 1

    tutorial_yolox_hand_detection
    tutorial_yolov8_rps_gesture_detection
    tutorial_media_pipe_hand_landmark
    tutorial_rtmpose_hand_landmark
    tutorial_hrnetv2_hand_landmark






