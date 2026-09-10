API Reference
^^^^^^^^^^^^^

This page provides a quick reference for all model classes, result types, and configuration methods available in the RZ/V2H RDK AI model packages.

For guidance on creating your own model, see :ref:`How to Add a New Model <how_to_add_new_model>`.

Core Data Types (rzv_model)
"""""""""""""""""""""""""""

These types are defined in ``rzv_model/base_model.hpp`` and ``rzv_model/utils.hpp``.

ModelInput
~~~~~~~~~~

.. code-block:: cpp

   struct ModelInput
   {
     cv::Mat original_image;  // Input image (YUV422 or RGB format)
     cv::Rect roi;            // Region of interest within the image
   };

ModelResult (base class)
~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~

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
~~~~~~~~~~~~

.. code-block:: cpp

   enum class YUV422Format { YUYV, UYVY };

BaseModel Class
"""""""""""""""

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
"""""""""""

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
"""""""""""""""""""""""

The following models are provided for object detection tasks. Each model class inherits from ``BaseModel`` and implements the required methods for loading, preprocessing, inference, and postprocessing.

rzv_yolox -- YoloxModel
~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
""""""""""""""""""""""

The following models are provided for pose estimation tasks. Each model class inherits from ``BaseModel`` and implements the required methods for loading, preprocessing, inference, and postprocessing.

rzv_hrnetv2 -- HRNetV2Model
~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Header: ``rzv_rtmpose/rtmpose_model.hpp`` | Inherits: ``BaseModel``

Returns ``KeyPointResult``. No additional configuration methods beyond ``BaseModel``.

**Quick example:**

.. code-block:: cpp

   auto model = std::make_unique<rzv_model::RTMPoseModel>();
   model->load("path/to/rtmpose_model");

   auto result = model->run<rzv_model::KeyPointResult>(input);

**Model preparation:** Same as HRNetV2 (MMPose).

rzv_mediapipe -- MediaPipeHandLandmarkModel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

ROS 2 Utilities (renesas_model_utils_ros2)
""""""""""""""""""""""""""""""""""""""""""

Header: ``renesas_model_utils_ros2/model_utils.hpp``

Provides helper functions for integrating AI models into ROS 2 nodes. Every symbol lives in the
``renesas_model_utils`` namespace, and the stateless helpers are static members of ``UtilsROS``.
The package builds a single shared library, ``librenesas_model_utils_ros2.so``.

.. note::

   The target platform is selected at configure time by the ``PRODUCT`` variable, which must be
   set to ``V2H`` for the RZ/V2H RDK. Configuration fails if it is unset. The selection is applied
   as a **PUBLIC** compile definition, ``PRODUCT_V2H``, so a consuming package automatically
   compiles against the same platform the library was built for.

   The cross-compilation environment exports ``PRODUCT`` by default. For a native build, pass it
   explicitly:

   .. code-block:: bash

      colcon build --packages-select renesas_model_utils_ros2 --cmake-args -DPRODUCT=V2H

V2HModelConfig
~~~~~~~~~~~~~~

Declared only when ``PRODUCT_V2H`` is defined.

.. code-block:: cpp

   struct V2HModelConfig
   {
     std::string model_path;
     std::vector<std::string> class_names;
     // Channel order the network expects on its input tensor: "rgb" (default)
     // or "bgr", set per model via the optional `input_order` key.
     std::string input_order = "rgb";
   };

DetectionMeta
~~~~~~~~~~~~~

Decoded counterpart of the metadata that the bounding-box encoders pack into a ``PoseArray``.
One entry per detection.

.. code-block:: cpp

   struct DetectionMeta
   {
     int class_id = 0;
     float confidence = 0.0f;
     // Up to about 15 characters, reconstructed from poses 1 to 4. Truncated
     // names are common; look up by class_id when the exact name matters.
     std::string class_name;
   };

load_v2h_model_config
~~~~~~~~~~~~~~~~~~~~~

A free function in the ``renesas_model_utils`` namespace, not a member of ``UtilsROS``.

.. code-block:: cpp

   V2HModelConfig load_v2h_model_config(
     const std::string & package_name, const std::string & model_name,
     const std::string & path_override = "",
     const std::vector<std::string> & class_names_override = {});

It reads ``<share>/<package_name>/config/models/models_config.yaml``. A non-empty
``path_override`` or ``class_names_override`` wins over the value in the YAML file. On any error
the function logs and returns a default-constructed config rather than throwing.

.. code-block:: cpp

   V2HModelConfig cfg = load_v2h_model_config("my_inference_pkg", "yolov8");

UtilsROS
~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 55 45

   * - Method
     - Description
   * - ``UtilsROS::encode_bounding_box_to_poses(pose_array, bbox, class_name, class_id, confidence)``
     - Encode an axis-aligned bounding box and its metadata as 8 poses in a
       ``geometry_msgs/PoseArray``.
   * - ``UtilsROS::encode_oriented_bounding_box_to_poses(pose_array, obbox, class_name, class_id, confidence)``
     - Encode a rotated bounding box and its metadata as 8 poses.
   * - ``UtilsROS::decode_detections_from_poses(pose_array, poses_per_detection)``
     - Decode the metadata packed by the two encoders above. Each detection occupies a fixed
       stride of ``poses_per_detection`` poses, 8 by default. Trailing poses that do not form a
       full detection block are ignored.
   * - ``UtilsROS::encode_inference_timing_diagnostic(name, pre_time, infer_time, post_time)``
     - Wrap preprocess, inference, and postprocess timings into a
       ``diagnostic_msgs/DiagnosticStatus`` with ``Preprocess Time (ms)``,
       ``Inference Time (ms)``, and ``Postprocess Time (ms)`` entries.
   * - ``UtilsROS::ros_image_to_bgr(msg)``
     - Convert a ``sensor_msgs/Image`` to a BGR ``cv::Mat``. Supports ``bgr8``, ``rgb8``,
       ``rgba8``, ``bgra8``, ``yuv422``/``uyvy``, ``yuv422_yuy2``/``yuyv``, and ``mono8``; other
       encodings fall back to a ``cv_bridge`` conversion attempt. Returns an empty ``cv::Mat`` on
       failure.

.. code-block:: cpp

   // Encode, axis-aligned or oriented
   UtilsROS::encode_bounding_box_to_poses(pose_array, bbox, "person", 0, 0.92f);
   UtilsROS::encode_oriented_bounding_box_to_poses(pose_array, obbox, "car", 2, 0.87f);

   // Decode the metadata on the subscriber side
   std::vector<DetectionMeta> dets = UtilsROS::decode_detections_from_poses(pose_array);

   // Convert an incoming image
   cv::Mat bgr = UtilsROS::ros_image_to_bgr(msg);

   // Package the timings for diagnostics
   auto status = UtilsROS::encode_inference_timing_diagnostic(
     "yolo_node", pre_ms, infer_ms, post_ms);

Detection layout in a PoseArray
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each detection occupies a fixed block of **8 poses**:

- The ``position`` of the 8 poses holds the box corners: the bottom face first, then the top
  face. For a 2D box the two faces are duplicated and ``z`` is 0.
- The ``orientation`` of the first 5 poses carries the metadata. Pose 0 holds ``class_id`` in
  ``x`` and ``confidence`` in ``y``. Poses 1 to 4 hold up to about 15 characters of the class
  name, one character per quaternion component.
- Poses 5 to 7 keep the identity quaternion.

Long class names are truncated by this layout. Look the exact name up by ``class_id`` when it
matters.

**YAML configuration format** (``config/models/models_config.yaml``):

.. code-block:: yaml

   models:
     my_model:
       path: "models/my_model_name"
       input_order: rgb
       names:
         0: class_a
         1: class_b
