RTMPose Hand Landmark Tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tutorial describes how to use the **RTMPose Hand Landmark** model for hand landmark estimation with DRP-AI acceleration on the Renesas RZ/V2H platform.

Please complete the :ref:`Prequisites step of the tutorial <tutorial_prerequisites>` before proceeding.

For this tutorial, we will utilize the pre-trained RTMPose model.

Compile with DRP-AI TVM Extension Package
""""""""""""""""""""""""""""""""""""""""""""""

Run the compilation script provided in the `hand_models/compilation` directory to compile the ONNX models for DRP-AI on the RZ/V2H platform.

- RTMPose Hand Landmark

  .. code-block:: bash

     $ SPARSE_ENABLE=false OPTIMIZER_ENABLE=false \
       python3 compile_onnx_model_quant_rtmpose_fhd_crop.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/rtmpose/rtmpose.onnx \
       -o ../data/rtmpose_fhd_crop \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
       -v 100 \
       -p "--calibrate_method Entropy \
            --node_to_exclude \
            /Shape,/Slice,/Concat,/Reshape,\
            /mlp/mlp.0/ReduceL2,/mlp/mlp.0/Mul,\
            /mlp/mlp.0/Clip,/mlp/mlp.0/Div,\
            /mlp/mlp.0/Mul_1,/mlp/mlp.1/MatMul,\
            /gau/ln/ReduceL2,/gau/ln/Mul,\
            /gau/ln/Clip,/gau/ln/Div,\
            /gau/ln/Mul_1,/gau/uv/MatMul,\
            /gau/act_fn/Sigmoid,/gau/act_fn/Mul,\
            /gau/Split,/gau/Unsqueeze,\
            /gau/Mul,/gau/Add,/gau/Split_1,\
            /gau/Squeeze_1,/gau/Transpose,\
            /gau/Squeeze,/gau/MatMul,\
            /gau/Div,/gau/Relu,/gau/Mul_1,\
            /gau/MatMul_1,/gau/Mul_2,\
            /gau/o/MatMul,/gau/res_scale/Mul,\
            /gau/Add_1,/cls_x/MatMul,\
            /cls_y/MatMul"

- RTMPose Evaluation with basic compilation

  .. code-block:: bash

     $ python3 compile_onnx_model_quant_rtmpose.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/rtmpose/rtmpose.onnx \
       -o ../data/rtmpose_freihand \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
       -v 100

- RTMPose Evaluation with Optimized Compilation (Recommended)

  .. code-block:: bash

     $ SPARSE_ENABLE=false OPTIMIZER_ENABLE=false \
       python3 compile_onnx_model_quant_rtmpose.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/rtmpose/rtmpose.onnx \
       -o ../data/rtmpose_optimized \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,256,256 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
       -v 100 \
       -p "--calibrate_method Entropy \
            --node_to_exclude \
            /Shape,/Slice,/Concat,/Reshape,\
            /mlp/mlp.0/ReduceL2,/mlp/mlp.0/Mul,\
            /mlp/mlp.0/Clip,/mlp/mlp.0/Div,\
            /mlp/mlp.0/Mul_1,/mlp/mlp.1/MatMul,\
            /gau/ln/ReduceL2,/gau/ln/Mul,\
            /gau/ln/Clip,/gau/ln/Div,\
            /gau/ln/Mul_1,/gau/uv/MatMul,\
            /gau/act_fn/Sigmoid,/gau/act_fn/Mul,\
            /gau/Split,/gau/Unsqueeze,\
            /gau/Mul,/gau/Add,/gau/Split_1,\
            /gau/Squeeze_1,/gau/Transpose,\
            /gau/Squeeze,/gau/MatMul,\
            /gau/Div,/gau/Relu,/gau/Mul_1,\
            /gau/MatMul_1,/gau/Mul_2,\
            /gau/o/MatMul,/gau/res_scale/Mul,\
            /gau/Add_1,/cls_x/MatMul,\
            /cls_y/MatMul"

Deploy and Test on RZ/V2H RDK
""""""""""""""""""""""""""""""""""""""""""""""

After compiling the models, transfer the generated model files to the RZ/V2H RDK and run inference using the **rzv_model** package.

For more detail, please reference to :ref:`the rzv_model package usage <how_to_use_rzv_model_package>`.