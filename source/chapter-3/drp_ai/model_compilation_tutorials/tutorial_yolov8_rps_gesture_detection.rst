YOLOv8 Object Detection Tutorial
""""""""""""""""""""""""""""""""""

This tutorial describes how to use the **YOLOv8n Rock Paper Scissors Gesture Detection** model for object detection with DRP-AI acceleration on the Renesas RZ/V2H platform.

Please complete the :ref:`Prerequisites step of the tutorial <tutorial_prerequisites>` before proceeding.

Installation Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~

- Ultralytics YOLOv8
- Dataset for Rock Paper Scissors Gesture Detection

  You can use the `Rock-Paper-Scissors Dataset <https://universe.roboflow.com/roboflow-58fyf/rock-paper-scissors-sxsw/dataset/14>`_ from Roboflow.

  Please download and prepare the dataset in YOLOv8 format for training.

Train the Model Using Ultralytics YOLOv8 and Transfer Learning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Please download the dataset above in YOLOv8 format and locate the ``data.yaml`` file. Replace the path in the Python script below and save it as ``train_model_yolov8.py``.

.. code-block:: python

   from ultralytics import YOLO

   # Loading pre-trained model
   # Can select between: yolov8n.pt yolov8s.pt yolov8m.pt yolov8l.pt yolov8x.pt
   model = YOLO('yolov8n.pt')

   # Display model information
   model.info()

   # Train the model with 80 epochs and an input image size of 640x640
   # The number of epochs can be changed to adapt to your requirements
   results = model.train(data="<path-to-the-dataset-folder>/rock-paper-scissors-14/data.yaml", epochs=80, imgsz=640)

Run the training process:

.. code-block:: bash

   python3 train_model_yolov8.py

For faster training time, a GPU with the appropriate driver should be installed.

The result will be located at ``<workspace-directory>/runs/detect/trainX/weights/best.pt``, where ``X`` increments each time you run the training process.

Export and Cut the Model
~~~~~~~~~~~~~~~~~~~~~~~~~

Since some parts of the post-processing phase of the YOLOv8 model cannot be handled by the DRP-AI hardware, certain steps need to be removed to ensure compatibility.

Please follow this documentation to learn how to export the model to ONNX format and cut the model:

- `How to convert YOLOv8 ONNX models for V2H <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/model_list/how_to_convert/How_to_convert_yolov8_onnx_models_V2H.md>`_

After exporting and cutting the model, you will get an ONNX model file named ``yolov8n_rps_cut.onnx`` (the name may differ based on your configuration).

Compile with DRP-AI TVM Extension Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run the compilation script provided in the ``hand_models/compilation`` directory to compile the ONNX models for DRP-AI on the RZ/V2H platform.

- YOLOv8n Rock Paper Scissors Gesture Detection

  .. code-block:: bash

     python3 compile_onnx_model_quant_yolov8_fhd_crop.py \
       $HAND_MODELS_DIR/palm_detection/yolov8/yolov8n_rps_cut.onnx \
       -o yolov8_rps \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,640,640 \
       -v 100 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_rps_hands