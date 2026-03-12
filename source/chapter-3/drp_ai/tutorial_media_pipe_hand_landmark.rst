MediaPipe Hand Landmark Tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This guide describes the process for converting **MediaPipe Hand Landmark Detection** models from **TFLite** to **ONNX** format for compatibility with **DRP-AI**.

Please complete the :ref:`Prequisites step of the tutorial <tutorial_prerequisites>` before proceeding.

Download Models
"""""""""""""""""""""""

* Reference: `MediaPipe Models List <https://github.com/google-ai-edge/mediapipe/blob/master/setup.py>`_
* `MediaPipe Hand Landmarker Models Documentation <https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker#models>`_
* `Hand Landmark Full Model <https://storage.googleapis.com/mediapipe-assets/hand_landmark_full.tflite>`_
* `Hand Landmark Lite Model <https://storage.googleapis.com/mediapipe-assets/hand_landmark_lite.tflite>`_
* `Palm Detection Full Model <https://storage.googleapis.com/mediapipe-assets/palm_detection_full.tflite>`_
* `Palm Detection Lite Model <https://storage.googleapis.com/mediapipe-assets/palm_detection_lite.tflite>`_

Set Up Conversion Environment
""""""""""""""""""""""""""""""""""""""""""""""

* Reference: `tensorflow-onnx GitHub Repository <https://github.com/onnx/tensorflow-onnx>`_ — Convert TensorFlow, Keras, TensorFlow.js, and TFLite models to ONNX.
* Create a dedicated conda environment with Python 3.10:

  .. code-block:: bash

					$ conda create --name tf2onnx python=3.10
					$ conda activate tf2onnx

Conversion Process
""""""""""""""""""""""""""""""""""""""""""""""

* **Important:** The supported ONNX opset for the DRP-AI translator is **v12**.

* Install dependencies (downgrade NumPy to resolve compatibility issues):

  .. code-block:: bash

      $ pip install tensorflow-onnx
      $ pip install numpy==1.26.4

* Convert the models to ONNX format (using NCHW layout):

  .. code-block:: bash

     $ python -m tf2onnx.convert \
       --tflite hand_landmark_full.tflite \
       --opset 12 \
       --output hand_landmark_full.onnx \
       --inputs-as-nchw input_1
     $ python -m tf2onnx.convert \
       --tflite palm_detection_full.tflite \
       --opset 12 \
       --output palm_detection_full.onnx \
       --inputs-as-nchw input_1

* For the Lite version, use:

  .. code-block:: bash

					$ python -m tf2onnx.convert \
							--tflite hand_landmark_lite.tflite \
							--opset 12 \
							--output hand_landmark_lite.onnx \
							--inputs-as-nchw input_1
					$ python -m tf2onnx.convert \
							--tflite palm_detection_lite.tflite \
							--opset 12 \
							--output palm_detection_lite.onnx \
							--inputs-as-nchw input_1

Compile with DRP-AI TVM Extension Package
""""""""""""""""""""""""""""""""""""""""""""""

Run the compilation script provided in the `hand_models/compilation` directory to compile the ONNX models for DRP-AI on the RZ/V2H platform.

Best performance and accuracy:

.. code-block:: bash

			$ SPARSE_ENABLE=false python3 compile_onnx_model_quant_mp_handlandmark_fhd_crop.py \
					$HAND_MODELS_DIR/hand_landmark_estimation/mp_hand_landmark/hand_landmark_full.onnx \
					-o ../data/mp_handlandmark_fhd_crop \
					-t $SDK \
					-d $TRANSLATOR \
					-c $QUANTIZER \
					-s 1,3,224,224 \
					--images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
					-v 100 \
					-p "--calibrate_method Entropy"

MediaPipe Evaluation model:

- Full model:

  .. code-block:: bash

     $ SPARSE_ENABLE=false \
       python3 compile_onnx_model_quant_mediapipe.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/mp_hand_landmark/hand_landmark_full.onnx \
       -o ../data/mediapipe_entropy \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,224,224 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
       -v 100 \
       -p "--calibrate_method Entropy"

- Lite model:

  .. code-block:: bash

     $ SPARSE_ENABLE=false \
       python3 compile_onnx_model_quant_mediapipe.py \
       $HAND_MODELS_DIR/hand_landmark_estimation/mp_hand_landmark/hand_landmark_lite.onnx \
       -o ../data/mediapipe_lite_entropy \
       -t $SDK \
       -d $TRANSLATOR \
       -c $QUANTIZER \
       -s 1,3,224,224 \
       --images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
       -v 100 \
       -p "--calibrate_method Entropy"

Optional: MediaPipe Palm Detection

.. code-block:: bash

			$ SPARSE_ENABLE=false OPTIMIZER_ENABLE=false python3 compile_onnx_model_quant_mp_palm_det.py \
			$HAND_MODELS_DIR/palm_detection/mediapipe/palm_detection_full.onnx \
			-o ../data/mp_palm_det \
			-t $SDK \
			-d $TRANSLATOR \
			-c $QUANTIZER \
			-s 1,3,192,192 \
			--images $HAND_MODELS_DIR/dataset/scripts/selected_freihand/ \
			-v 100

Deploy and Test on RZ/V2H RDK
""""""""""""""""""""""""""""""""""""""""""""""

After compiling the models, transfer the generated model files to the RZ/V2H RDK and run inference using the **rzv_model** package.

For more detail, please reference to :ref:`the rzv_model package usage <how_to_use_rzv_model_package>`.