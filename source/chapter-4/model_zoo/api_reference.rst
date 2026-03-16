API Reference
-------------

This page provides a quick reference for all model classes, result types, and configuration methods available in the RZ/V2H RDK AI model packages.

For guidance on creating your own model, see :ref:`How to Add a New Model <how_to_add_new_model>`.

Core Data Types (rzv_model)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

These types are defined in ``rzv_model/base_model.hpp`` and ``rzv_model/utils.hpp``.

ModelInput
""""""""""

.. code-block:: cpp

   struct ModelInput
   {
     cv::Mat original_image;  // Input image (YUV422 or RGB format)
     cv::Rect roi;            // Region of interest within the image
   };

ModelResult (base class)
""""""""""""""""""""""""

All result types inherit from this. Contains timing information from each inference stage.

.. code-block:: cpp

   struct ModelResult
   {
     float score = 0.0f;
     float preprocess_ms = 0.0f;   // Time spent in preprocessing
     float inference_ms = 0.0f;    // Time spent in DRP-AI inference
     float postprocess_ms = 0.0f;  // Time spent in postprocessing
   };

KeyPoint / KeyPointResult
"""""""""""""""""""""""""

Used by pose estimation models (HRNetV2, RTMPose, MediaPipe).

.. code-block:: cpp

   struct KeyPoint
   {
     float x;
     float y;
     float confidence;
     int class_id;
   };

   struct KeyPointResult : public ModelResult
   {
     std::vector<KeyPoint> keypoints;
   };

ModelShapeInfo
""""""""""""""

Provides tensor shape information extracted from the loaded model.

.. code-block:: cpp

   struct ModelShapeInfo
   {
     std::vector<int64_t> input_shape;
     std::string input_dtype;
     std::vector<std::vector<int64_t>> output_shapes;
     std::vector<std::string> output_dtypes;

     int input_height() const;   // input_shape[2]
     int input_width() const;    // input_shape[3]
     int input_channels() const; // input_shape[1]
   };

YUV422Format
""""""""""""

.. code-block:: cpp

   enum class YUV422Format { YUYV, UYVY };

BaseModel Class
^^^^^^^^^^^^^^^

The base class for all AI models. Defined in ``rzv_model/base_model.hpp``.

**Public methods:**

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Method
     - Description
   * - ``bool load(const std::string & model_path)``
     - Load a DRP-AI model from the given directory path.
   * - ``bool is_loaded() const``
     - Check whether a model has been loaded.
   * - ``const ModelShapeInfo & get_shape_info() const``
     - Get input/output tensor shape information.
   * - ``std::unique_ptr<T> run<T>(const ModelInput & input)``
     - Run inference and return typed result. Returns ``nullptr`` on failure.

**Protected methods (override in subclasses):**

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Method
     - Description
   * - ``postprocess(output_tensors)`` **(required)**
     - Parse raw output tensors into a ``ModelResult``.
   * - ``preprocess(input)``
     - Custom preprocessing before inference.
   * - ``fallback_preprocess(input)``
     - CPU fallback when hardware preprocessing is unavailable.
   * - ``software_preprocess(input, imagenet, mean, std)``
     - CPU preprocessing with optional ImageNet normalization.
   * - ``extract_model_specific_shapes(shape_info)``
     - Extract custom shapes after model load.

**Protected helper methods:**

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Method
     - Description
   * - ``letterbox(im, new_shape, color, center_align, do_resize)``
     - Resize and pad image while maintaining aspect ratio.
   * - ``is_preprocess_loaded()``
     - Check if DRP-AI hardware preprocessing is available.
   * - ``map_coordinates_to_original(point)``
     - Map a point from preprocessed to original image coordinates.
   * - ``map_size_to_original(size)``
     - Map a size from preprocessed to original image coordinates.
   * - ``set_padding_color(color)``
     - Set the padding color for letterbox.
   * - ``MODEL_INFO / MODEL_DEBUG / MODEL_WARN / MODEL_ERROR``
     - Logging macros (uses spdlog).

Utils Class
^^^^^^^^^^^

Static utility functions defined in ``rzv_model/utils.hpp``.

.. list-table::
   :header-rows: 1
   :widths: 55 45

   * - Method
     - Description
   * - ``Utils::bgr_to_yuv422(bgr_image, format)``
     - Convert BGR image to YUV422 (YUYV or UYVY).
   * - ``Utils::rgba_to_yuv422(rgba_image, format)``
     - Convert RGBA image to YUV422 (YUYV or UYVY).
   * - ``Utils::non_maximum_suppression_batched(boxes, scores, class_ids, score_thresh, iou_thresh)``
     - Batched NMS for ``cv::Rect2f`` (axis-aligned) or ``cv::RotatedRect`` (oriented) boxes.

Object Detection Models
^^^^^^^^^^^^^^^^^^^^^^^

The following models are provided for object detection tasks. Each model class inherits from ``BaseModel`` and implements the required methods for loading, preprocessing, inference, and postprocessing.

rzv_yolox -- YoloxModel
"""""""""""""""""""""""

Header: ``rzv_yolox/yolox_model.hpp`` | Inherits: ``BaseModel``

**Result type:**

.. code-block:: cpp

   struct YOLOXDetection
   {
     cv::Rect bbox;
     int class_id;
     float confidence;
     bool is_valid = false;
     std::string class_name;
   };

   struct YOLOXDetectionResult : public ModelResult
   {
     std::vector<YOLOXDetection> detections;
   };

**Configuration methods:**

.. list-table::
   :header-rows: 1
   :widths: 55 45

   * - Method
     - Description
   * - ``set_class_names(class_names)``
     - Set class labels (must match model training).
   * - ``set_confidence_threshold(threshold)``
     - Set detection confidence threshold (0.0 - 1.0).
   * - ``set_iou_threshold(threshold)``
     - Set NMS IoU threshold (0.0 - 1.0).

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::YoloxModel>();
   model->set_class_names({"hand"});
   model->set_confidence_threshold(0.5f);
   model->set_iou_threshold(0.4f);
   model->load("path/to/yolox_model");

   auto result = model->run<rzv_model::YOLOXDetectionResult>(input);

**Model preparation:** `YOLOX <https://github.com/Megvii-BaseDetection/YOLOX>`_ |
`YOLOX - Convert for V2H <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/model_list/how_to_convert/How_to_convert_yolox_onnx_models_V2H.md>`_

rzv_yolov8 -- YOLOv8DetectModel
"""""""""""""""""""""""""""""""

Header: ``rzv_yolov8/yolov8_detect_model.hpp`` | Inherits: ``YOLOv8Base`` -> ``BaseModel``

**Result type:**

.. code-block:: cpp

   struct YOLOv8Detection
   {
     cv::Rect bbox;
     int class_id;
     float confidence;
     bool is_valid = false;
     std::string class_name;
   };

   struct YOLOv8DetectionResult : public ModelResult
   {
     std::vector<YOLOv8Detection> detections;
   };

**Configuration methods:**

.. list-table::
   :header-rows: 1
   :widths: 55 45

   * - Method
     - Description
   * - ``set_class_names(class_names)``
     - Set class labels (must match model training).
   * - ``set_confidence_threshold(threshold)``
     - Set detection confidence threshold (0.0 - 1.0).
   * - ``set_nms_threshold(threshold)``
     - Set NMS threshold (0.0 - 1.0).
   * - ``set_dfl_sigmoid_mode(mode)``
     - Set DFL sigmoid optimization mode (see below).
   * - ``set_cpu_dfl_multi_thread(enable)``
     - Enable/disable multi-threaded CPU DFL processing.

**DFL Sigmoid Modes** (``DFLSigmoidMode`` enum):

- ``InDfl`` -- Apply sigmoid during DFL processing (original).
- ``AfterArgmax`` -- Skip sigmoid in DFL, apply after argmax (faster).
- ``AfterThreshold`` -- Skip sigmoid in DFL, apply after threshold filtering (fastest, default).

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::YOLOv8DetectModel>();
   model->set_class_names({"paper", "rock", "scissor"});
   model->set_confidence_threshold(0.5f);
   model->set_nms_threshold(0.4f);
   model->set_dfl_sigmoid_mode(rzv_model::DFLSigmoidMode::AfterThreshold);
   model->set_cpu_dfl_multi_thread(false);
   model->load("path/to/yolov8_model");

   auto result = model->run<rzv_model::YOLOv8DetectionResult>(input);

**Model preparation:** `Ultralytics YOLO <https://docs.ultralytics.com/>`_ |
`YOLOv8 - Convert for V2H <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/v2.7.0/docs/model_list/how_to_convert/How_to_convert_yolov8_onnx_models_V2H.md>`_

rzv_yolov8 -- YOLOv8OBBModel
""""""""""""""""""""""""""""

Header: ``rzv_yolov8/yolov8_obb_model.hpp`` | Inherits: ``YOLOv8Base`` -> ``BaseModel``

For oriented bounding box detection (e.g., aerial/satellite imagery).

**Result type:**

.. code-block:: cpp

   struct YOLOv8OBBDetection
   {
     cv::RotatedRect obbox;  // Oriented bounding box
     int class_id;
     float confidence;
     bool is_valid = false;
     std::string class_name;
   };

   struct YOLOv8OBBDetectionResult : public ModelResult
   {
     std::vector<YOLOv8OBBDetection> detections;
   };

**Configuration methods:** Same as ``YOLOv8DetectModel`` (inherits from ``YOLOv8Base``).

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::YOLOv8OBBModel>();
   model->set_class_names({"ship", "plane", "vehicle"});
   model->set_confidence_threshold(0.6f);
   model->set_nms_threshold(0.5f);
   model->load("path/to/yolov8_obb_model");

   auto result = model->run<rzv_model::YOLOv8OBBDetectionResult>(input);

rzv_gold_yolo -- GoldYoloModel
""""""""""""""""""""""""""""""

Header: ``rzv_gold_yolo/gold_yolo_model.hpp`` | Inherits: ``BaseModel``

**Result type:**

.. code-block:: cpp

   struct GOLDYOLODetection
   {
     cv::Rect bbox;
     int class_id;
     float confidence;
     bool is_valid = false;
     std::string class_name;
   };

   struct GOLDYOLODetectionResult : public ModelResult
   {
     std::vector<GOLDYOLODetection> detections;
   };

**Configuration methods:** Same as ``YoloxModel`` (``set_class_names``, ``set_confidence_threshold``, ``set_iou_threshold``).

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::GoldYoloModel>();
   model->set_class_names({"hand"});
   model->set_confidence_threshold(0.5f);
   model->set_iou_threshold(0.4f);
   model->load("path/to/gold_yolo_model");

   auto result = model->run<rzv_model::GOLDYOLODetectionResult>(input);

Pose Estimation Models
^^^^^^^^^^^^^^^^^^^^^^

The following models are provided for pose estimation tasks. Each model class inherits from ``BaseModel`` and implements the required methods for loading, preprocessing, inference, and postprocessing.

rzv_hrnetv2 -- HRNetV2Model
"""""""""""""""""""""""""""

Header: ``rzv_hrnetv2/hrnetv2_model.hpp`` | Inherits: ``BaseModel``

Returns ``KeyPointResult``. No additional configuration methods beyond ``BaseModel``.

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::HRNetV2Model>();
   model->load("path/to/hrnetv2_model");

   auto result = model->run<rzv_model::KeyPointResult>(input);
   for (const auto & kp : result->keypoints) {
       std::cout << "x=" << kp.x << " y=" << kp.y
                 << " conf=" << kp.confidence << std::endl;
   }

**Model preparation:** `MMPose <https://github.com/open-mmlab/mmpose>`_ |
`Convert for V2H <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/model_list/how_to_convert/How_to_convert_mmpose_models_V2H.md>`_

rzv_rtmpose -- RTMPoseModel
"""""""""""""""""""""""""""

Header: ``rzv_rtmpose/rtmpose_model.hpp`` | Inherits: ``BaseModel``

Returns ``KeyPointResult``. No additional configuration methods beyond ``BaseModel``.

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::RTMPoseModel>();
   model->load("path/to/rtmpose_model");

   auto result = model->run<rzv_model::KeyPointResult>(input);

**Model preparation:** Same as HRNetV2 (MMPose).

rzv_mediapipe -- MediaPipeHandLandmarkModel
"""""""""""""""""""""""""""""""""""""""""""

Header: ``rzv_mediapipe/mediapipe_hand_landmark_model.hpp`` | Inherits: ``BaseModel``

Returns ``HandLandmarkResult`` which extends ``KeyPointResult`` with handedness classification.

**Result type:**

.. code-block:: cpp

   struct HandLandmarkResult : public KeyPointResult
   {
     float handedness;  // 0.0 = left hand, 1.0 = right hand
   };

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::MediaPipeHandLandmarkModel>();
   model->load("path/to/mediapipe_hand_landmark_model");

   auto result = model->run<rzv_model::HandLandmarkResult>(input);
   std::cout << "Hand: " << (result->handedness > 0.5 ? "Right" : "Left") << std::endl;

**Model preparation:** `MediaPipe <https://github.com/google-ai-edge/mediapipe/tree/master>`_

ROS 2 Utilities (rzv_model_utils_ros2)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Header: ``rzv_model_utils_ros2/model_utils.hpp``

Provides helper functions for integrating AI models into ROS 2 nodes.

ModelConfig
"""""""""""

.. code-block:: cpp

   struct ModelConfig
   {
     std::string model_path;
     std::vector<std::string> class_names;
   };

UtilsROS
""""""""

.. list-table::
   :header-rows: 1
   :widths: 55 45

   * - Method
     - Description
   * - ``UtilsROS::load_model_info(package, model_type, path_override, class_override)``
     - Load model configuration from YAML with optional parameter overrides.
   * - ``UtilsROS::encode_bboxes_to_pose_array(detections)``
     - Convert detection bounding boxes to ``geometry_msgs/PoseArray``.
   * - ``UtilsROS::encode_diagonal_timing(result)``
     - Encode inference timing into a diagnostic message.

**YAML configuration format** (``config/models/models_config.yaml``):

.. code-block:: yaml

   models:
     my_model:
       path: "models/my_model_name"
       names:
         0: class_a
         1: class_b