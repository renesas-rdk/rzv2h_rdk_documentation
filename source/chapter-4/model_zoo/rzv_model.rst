.. _rzv_model_package:

The rzv_model Package
^^^^^^^^^^^^^^^^^^^^^^^^^^

The **rzv_model** package is a C++ library designed to facilitate the deployment of AI models on the RZ/V2H platform using the DRP-AI driver.

It provides a unified interface for loading, running, and managing various AI models optimized for the RZ/V2H architecture.

Base Framework
"""""""""""""""""""""""

The **rzv_model** package provides a flexible and modular framework for deploying AI models optimized on DRP-AI driver for the RZ/V2H platform.
It includes the following core features:

- Abstracted model interface with hardware acceleration support.
- Common pre-processing and post-processing utilities for image-based inference.
- DRP-AI runtime integration for efficient inference execution.
- Support multiple AI model running at the same time with DRP-AI driver.
- Support for both YUV422 and RGB image formats.

Package Structure
"""""""""""""""""""""""

The **rzv_model** package is organized into the following structure:

.. code-block:: console

    rzv_model/
      ├── CMakeLists.txt
      ├── include
      │   └── rzv_model
      │       ├── base_model.hpp
      │       ├── model_specific.hpp
      │       └── utils.hpp
      ├── package.xml
      ├── README.md
      └── src
          ├── base_model.cpp
          ├── model_specific.cpp
          ├── platform
          │   ├── MeraDrpRuntimeWrapper.cpp
          │   ├── MeraDrpRuntimeWrapper.h
          │   ├── PreRuntime.h
          │   └── PreRuntimeV2H.cpp
          └── utils.cpp

- The **include/rzv_model** directory contains the header files defining the base model class and utility functions.
- The **src** directory contains the implementation of the base model, platform-specific runtime wrappers, and utility functions.
- The **CMakeLists.txt** and **package.xml** files are used for building and packaging the library.

Architecture
"""""""""""""""""""""""

The **rzv_model** package follows a **modular architecture** designed for extensibility, maintainability, and efficient deployment on DRP-AI.

- **Base Model:**
  Provides the ``BaseModel`` class, which implements shared functionalities such as model loading, pre-processing, inference execution, and result handling.

- **Model-Specific Implementations:**
  Each AI model (e.g., YOLOX, YOLOv8, HRNet, RTMPose) inherits from the base class and extends it with task-specific logic such as detection parsing or key point extraction.

- **Utility Modules:**
  Contain helper functions for image pre-processing, tensor conversion, normalization, and post-processing visualization.

This modular design enables developers to easily integrate new AI models and customize pre-processing or inference pipelines for various use cases on the RZ/V2H platform.

.. _how_to_use_rzv_model_package:

How to Use the rzv_model Package
""""""""""""""""""""""""""""""""""

To use the **rzv_model** package, you need to prepare the model configuration files, including the compiled model files from the DRP-AI TVM conversion process.

Please follow the steps below to set up and use the **rzv_model** package effectively.

Input Requirements Files
~~~~~~~~~~~~~~~~~~~~~~~~

After completing the :ref:`Exchange AI model <exchange_ai_model>` step, the output should contain the compiled model files including:

Example of ``mera1`` model files:

.. code-block:: text

   output_directory/
      ├── addr_map.txt
      ├── deploy.json
      ├── deploy.params
      ├── deploy.so
      └── preprocess
         ├── addr_map.txt
         ├── aimac_cmd.bin
         ├── aimac_desc.bin
         ├── aimac_param_cmd.bin
         ├── aimac_param_desc.bin
         ├── drp_config.mem
         ├── drp_desc.bin
         ├── drp_param.bin
         ├── drp_param_info.txt
         └── weight.bin

Example of ``mera2`` model files:

.. code-block:: text

   output_directory/
      ├── addr_map.txt
      ├── mera.plan
      ├── preprocess
      │  ├── addr_map.txt
      │  ├── aimac_cmd.bin
      │  ├── aimac_desc.bin
      │  ├── aimac_param_cmd.bin
      │  ├── aimac_param_desc.bin
      │  ├── drp_config.mem
      │  ├── drp_desc.bin
      │  ├── drp_param.bin
      │  ├── drp_param_info.txt
      │  └── weight.bin
      └── sub_0000__CPU_DRP_TVM
         ├── deploy.json
         ├── deploy.params
         └── deploy.so

Note that, the top-level ``addr_map.txt`` file is required for multiple models running with DRP-AI driver.

This ``output_directory`` folder will be placed under the **config/models** directory of the application package that uses the model
(e.g., ``rzv_object_detection/config/models/``, ``rzv_pose_estimation/config/models/``),
and the path to the model will be specified in the application configuration.

Post-processing Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each model may have different post-processing requirements based on its specific task (e.g., object detection, pose estimation).

To customize the post-processing behavior, you can modify the corresponding model-specific implementation files located in the **src/** directory of the **rzv_ai_model_name** (rzv_yolov8,etc.) package.

The details of post-processing configuration are not covered in this section. Please refer to the example in the each package (rzv_yolox, etc.) for a clearer understanding.

.. hint::

    There are some sample applications from Renesas, which have the custom post-processing for the specific models.

    You can refer to these applications for reference on implementing the AI model post-processing logic:

    - `RZ/V AI Applications Repository <https://github.com/renesas-rz/rzv_ai_sdk/tree/main>`_.

    - `Ignitarium Renesas - RZ/V AI Applications <https://github.com/Ignitarium-Renesas/rzv_ai_apps>`_.

Example of Using rzv_model Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Load the model and perform inference using the **rzv_model** package with the following code snippet:

.. code-block:: cpp

    // Example using HRNetV2 model for pose estimation
    auto model = std::make_unique<rzv_model::HRNetV2Model>();
    model->load(model_path);

    // Prepare input
    rzv_model::ModelInput input{image, roi};

    // Run inference
    auto result = model->run<rzv_model::KeyPointResult>(input);

.. _how_to_add_new_model:

How to Add a New Model
""""""""""""""""""""""""""

This section provides a step-by-step guide for adding a new AI model to the RZ/V2H RDK ecosystem.

The process involves creating a new model package, implementing the model-specific logic, preparing the DRP-AI model files, and optionally integrating with ROS 2.

Step 1: Create a New Model Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a new ROS 2 / C++ package for your model. The recommended structure is:

.. code-block:: console

    rzv_my_model/
      ├── CMakeLists.txt
      ├── package.xml
      ├── include
      │   └── rzv_my_model
      │       └── my_model.hpp
      └── src
          └── my_model.cpp

In your ``CMakeLists.txt``, add ``rzv_model`` as a dependency:

.. code-block:: cmake

    find_package(rzv_model REQUIRED)
    ament_target_dependencies(rzv_model)

In your ``package.xml``, declare the dependency:

.. code-block:: xml

    <depend>rzv_model</depend>

Step 2: Define the Result Structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Define a result structure that inherits from ``ModelResult``. This structure holds the output of your model's postprocessing.

.. code-block:: cpp

    #include "rzv_model/base_model.hpp"

    namespace rzv_model {

    struct MyDetection
    {
        cv::Rect bbox;
        std::string class_name;
        float confidence;
        bool is_valid;
    };

    struct MyDetectionResult : public ModelResult
    {
        std::vector<MyDetection> detections;
    };

    } // namespace rzv_model

For pose estimation models, you can reuse the built-in ``KeyPointResult``:

.. code-block:: cpp

    // KeyPointResult is already defined in base_model.hpp:
    // struct KeyPoint { float x; float y; float confidence; int class_id; };
    // struct KeyPointResult : public ModelResult { std::vector<KeyPoint> keypoints; };

    // Or extend it for additional data:
    struct MyPoseResult : public KeyPointResult
    {
        float handedness;  // Example: additional output
    };

Step 3: Implement the Model Class
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a model class that inherits from ``BaseModel`` and override the required methods:

.. code-block:: cpp

    // my_model.hpp
    #include "rzv_model/base_model.hpp"

    namespace rzv_model {

    class MyModel : public BaseModel
    {
    public:
        MyModel();
        virtual ~MyModel();

        // Add configuration setters as needed
        void set_class_names(const std::vector<std::string> & class_names);
        void set_confidence_threshold(float threshold);

    protected:
        // Required: parse raw output tensors into your result structure
        std::unique_ptr<ModelResult> postprocess(
            const std::vector<cv::Mat> & output_tensors) override;

        // Optional: custom preprocessing (default uses DRP-AI hardware preprocessing)
        cv::Mat preprocess(const ModelInput & input) override;

        // Optional: CPU fallback when DRP-AI preprocessing is unavailable
        cv::Mat fallback_preprocess(const ModelInput & input) override;

        // Optional: extract model-specific shape information after loading
        void extract_model_specific_shapes(const ModelShapeInfo & shape_info) override;

    private:
        class Impl;
        std::unique_ptr<Impl> pimpl_;
    };

    } // namespace rzv_model

**Key methods to override:**

- ``postprocess()`` **(required)**: Parses the raw output tensors from DRP-AI inference into your result structure. This is where you decode bounding boxes, apply NMS, extract keypoints, etc.
- ``preprocess()``: Converts the input image into the format expected by your model. The base class handles DRP-AI-specific image format conversions; your override adds any model-specific transforms (e.g., resizing, normalization).
- ``fallback_preprocess()``: Provides a CPU-based fallback when DRP-AI preprocessing is unavailable.
- ``extract_model_specific_shapes()``: Called after model loading to extract shapes specific to your model from the ``ModelShapeInfo`` (e.g., number of keypoints from output tensor dimensions).

**Available helper methods from BaseModel:**

- ``letterbox()``: Resize and pad an image while maintaining aspect ratio, commonly used in YOLO-family preprocessing.
- ``software_preprocess()``: CPU-based preprocessing with optional ImageNet normalization (configurable mean and std values).
- ``is_preprocess_loaded()``: Check if DRP-AI hardware preprocessing is available.
- ``map_coordinates_to_original()``: Map a point from preprocessed image coordinates back to original image coordinates.
- ``map_size_to_original()``: Map a size from preprocessed image coordinates back to original image coordinates.
- ``set_padding_color()``: Set the padding color used by letterbox preprocessing.

**Available utility functions from Utils class:**

- ``Utils::bgr_to_yuv422()``: Convert BGR image to YUV422 format.
- ``Utils::rgba_to_yuv422()``: Convert RGBA image to YUV422 format.
- ``Utils::non_maximum_suppression_batched()``: Batched NMS for axis-aligned or oriented bounding boxes.

Step 4: Prepare DRP-AI Model Files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Convert your trained model to the DRP-AI compatible format and place the compiled files under the ``config/models`` directory of **your application package**:

.. code-block:: console

    rzv_my_application/          # Your application package (not rzv_model)
      └── config
          └── models
              └── my_model_name/
                  |-- addr_map.txt
                  |-- deploy.json
                  |-- deploy.params
                  |-- deploy.so
                  └── preprocess/
                      |-- addr_map.txt
                      |-- aimac_cmd.bin
                      |-- aimac_desc.bin
                      |-- aimac_param_cmd.bin
                      |-- aimac_param_desc.bin
                      |-- drp_config.mem
                      |-- drp_desc.bin
                      |-- drp_param.bin
                      |-- drp_param_info.txt
                      └── weight.bin

If you don't use ROS 2 or don't have an application package, you can create a separate folder for your model files and specify the path when loading the model in your code.

Step 5: Load and Run the Model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the model in your application:

.. code-block:: cpp

    #include "rzv_my_model/my_model.hpp"

    // Create and configure the model
    auto model = std::make_unique<rzv_model::MyModel>();
    model->set_class_names({"person", "car", "bicycle"});
    model->set_confidence_threshold(0.5f);

    // Load the DRP-AI model files
    model->load("path/to/my_model_name");

    // Prepare input image (convert to YUV422 format for DRP-AI)
    cv::Mat bgr_image = cv::imread("image.png");
    cv::Mat rgba_image;
    cv::cvtColor(bgr_image, rgba_image, cv::COLOR_BGR2RGBA);
    cv::Mat yuv422_image = rzv_model::Utils::rgba_to_yuv422(
        rgba_image, rzv_model::YUV422Format::YUYV);

    // Create input with image and ROI
    auto input = rzv_model::ModelInput{
        yuv422_image, cv::Rect(0, 0, yuv422_image.cols, yuv422_image.rows)};

    // Run inference (recommended: typed run)
    auto result = model->run<rzv_model::MyDetectionResult>(input);

    // Access results
    for (const auto & det : result->detections) {
      if (det.is_valid) {
        std::cout << det.class_name << ": " << det.confidence << std::endl;
      }
    }

    // Access timing information
    std::cout << "Preprocess: " << result->preprocess_ms << " ms" << std::endl;
    std::cout << "Inference: " << result->inference_ms << " ms" << std::endl;
    std::cout << "Postprocess: " << result->postprocess_ms << " ms" << std::endl;

Step 6 (Optional): Integrate with ROS 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To use your new model in a ROS 2 application, you can either:

- Add it to a new or existing ROS 2 application package (e.g., ``rzv_my_app ``, ``rzv_object_detection`` or ``rzv_pose_estimation``).
- Create a new ROS 2 node that uses ``rzv_model_utils_ros2`` for model configuration and message encoding.

In your application package, add ``rzv_model_utils_ros2`` and your model package as dependencies:

.. code-block:: cmake

    # CMakeLists.txt of the application package
    find_package(rzv_model REQUIRED)
    find_package(rzv_my_model REQUIRED)
    find_package(rzv_model_utils_ros2 REQUIRED)
    ament_target_dependencies(rzv_model rzv_my_model rzv_model_utils_ros2)

.. code-block:: xml

    <!-- package.xml of the application package -->
    <depend>rzv_model</depend>
    <depend>rzv_my_model</depend>
    <depend>rzv_model_utils_ros2</depend>

When using ``rzv_model_utils_ros2``, register your model in the YAML configuration file:

.. code-block:: yaml

    # config/models/models_config.yaml
    models:
      my_model_name:
        path: "models/my_model_name"
        names:
          0: class_name0
          1: class_name1
          2: class_name2

Then load the model configuration in your ROS 2 node:

.. code-block:: cpp

    auto model_config = rzv_model::UtilsROS::load_model_info(
        "rzv_my_app",          // ROS 2 package name contains the model files
        "my_model_name",       // Model type key in YAML
        model_path_param,      // Optional: path override
        class_names_param      // Optional: class name override
    );

    // Extract resolved configuration
    auto model_path = model_config.model_path;
    auto class_names = model_config.class_names;

Model Logging
~~~~~~~~~~~~~~

This package provides logging function to facilitate debugging and information output at runtime with `spdlog <https://github.com/gabime/spdlog>`_.

By default, the logging level is set to ``info``. You can change the logging level by setting the ``SPDLOG_LEVEL`` environment variable:

.. code-block:: bash

    export SPDLOG_LEVEL=debug

Available levels: ``trace``, ``debug``, ``info``, ``warn``, ``err``, ``critical``, ``off``.

Usage in your model implementation:

.. code-block:: cpp

    // Inside your model class (inherits from BaseModel)
    MODEL_INFO("Model loaded successfully.");
    MODEL_DEBUG("Input shape: {}x{}", width, height);
    MODEL_WARN("Confidence threshold too low: {}", threshold);
    MODEL_ERROR("Failed to load model: {}", error_msg);

ROS 2 Integration
~~~~~~~~~~~~~~~~~~

This package optionally provides a CMake integration module to simplify usage in ROS 2 packages.
When enabled, the exported CMake files allow other ROS 2 nodes to link against this model framework using ``find_package(rzv_model)`` and ``ament_target_dependencies``.

It can also be built as a standalone C++ library using make or CMake without ROS 2 dependencies.

Dependencies
~~~~~~~~~~~~~~

- OpenCV
- Renesas DRP-AI Runtime
- Memory Manager (MMNgr)
