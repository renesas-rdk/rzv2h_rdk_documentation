YOLOX Object Detection Tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tutorial describes how to use the **YOLOX Hand Detection** model for object detection with DRP-AI acceleration on the Renesas RZ/V2H platform.

Please complete the :ref:`Prequisites step of the tutorial <tutorial_prerequisites>` before proceeding.

For this tutorial, we will utilize the pre-trained YOLOX model.

Compile with DRP-AI TVM Extension Package
""""""""""""""""""""""""""""""""""""""""""""""

Run the compilation script provided in the `hand_models/compilation` directory to compile the ONNX models for DRP-AI on the RZ/V2H platform.

- YOLOX Hand

  .. code-block:: bash

        $ python3 compile_onnx_model_quant_yolox_hand_fhd_crop.py \
            $HAND_MODELS_DIR/palm_detection/yolox/yolox_hand.onnx \
            -o ../data/yolox_hand_fhd_crop \
            -t $SDK \
            -d $TRANSLATOR \
            -c $QUANTIZER \
            -s 1,3,640,640 \
            -v 100 \
            --images $HAND_MODELS_DIR/dataset/scripts/selected_hands/

- Gold YOLO Nano

  .. code-block:: bash

        $ python3 compile_onnx_model_quant_gold_yolo_fhd_crop.py \
            $HAND_MODELS_DIR/palm_detection/gold_yolo/gold_yolo_n_hand_0303_0.4172_1x3x480x640.onnx \
            -o ../data/gold_yolo_n_fhd_crop \
            -t $SDK \
            -d $TRANSLATOR \
            -c $QUANTIZER \
            -s 1,3,480,640 \
            --images $HAND_MODELS_DIR/dataset/scripts/selected_hands/ \
            -v 100

Deploy and Test on RZ/V2H RDK
""""""""""""""""""""""""""""""""""""""""""""""

After compiling the models, transfer the generated model files to the RZ/V2H RDK and run inference using the **rzv_model** package.

For more detail, please reference to :ref:`the rzv_model package usage <how_to_use_rzv_model_package>`.