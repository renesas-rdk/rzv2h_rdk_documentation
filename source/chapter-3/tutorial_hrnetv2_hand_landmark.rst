HRNetV2 Hand Landmark Tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tutorial describes how to use the **HRNetV2 Hand Landmark** model for hand landmark estimation with DRP-AI acceleration on the Renesas RZ/V2H platform.

Please complete the :ref:`Prequisites step of the tutorial <tutorial_prerequisites>` before proceeding.

For this tutorial, we will utilize the pre-trained HRNetV2 model.

Compile with DRP-AI TVM Extension Package
""""""""""""""""""""""""""""""""""""""""""""""

Run the compilation script provided in the `hand_models/compilation` directory to compile the ONNX models for DRP-AI on the RZ/V2H platform.

- HRNetv2 Hand Landmark

  .. code-block:: bash

     $ python3 compile_onnx_model_quant_hrnetv2_fhd_crop.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/hrnetv2/hrnetv2_onehand10k.onnx \
       -o ../data/hrnetv2_hands_fhd_crop \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_hands/ \
       -v 100

- HRNetv2 Evaluation with Selected Hands Dataset

  .. code-block:: bash

     $ python3 compile_onnx_model_quant_hrnetv2.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/hrnetv2/hrnetv2_onehand10k.onnx \
       -o ../data/hrnetv2_selected_hands \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_hands/ \
       -v 100

- HRNetv2 Evaluation with FreiHand Dataset

  .. code-block:: bash

     $ python3 compile_onnx_model_quant_hrnetv2.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/hrnetv2/hrnetv2_onehand10k.onnx \
       -o ../data/hrnetv2_freihand \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/freihand_rois/ \
       -v 100

- HRNetv2 Evaluation with Entropy Calibration

  .. code-block:: bash

     $ SPARSE_ENABLE=false OPTIMIZER_ENABLE=false \
       python3 compile_onnx_model_quant_hrnetv2.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/hrnetv2/hrnetv2_onehand10k.onnx \
       -o ../data/hrnetv2_selected_hands_entropy \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_hands/ \
       -v 100 \
       -p "--calibrate_method Entropy"

Deploy and Test on RZ/V2H RDK
""""""""""""""""""""""""""""""""""""""""""""""

After compiling the models, transfer the generated model files to the RZ/V2H RDK and run inference using the **rzv_model** package.

For more detail, please reference to :ref:`the rzv_model package usage <how_to_use_rzv_model_package>`.