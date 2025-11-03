The rzv_model package
--------------------------

Base Framework
^^^^^^^^^^^^^^^^^^^^^^^

The **rzv_model** package provides a flexible and modular framework for deploying AI models optimized on DRP-AI driver for the RZ/V2H platform.
It includes the following core features:

- Abstracted model interface with hardware acceleration support.
- Common pre-processing and post-processing utilities for image-based inference.
- DRP-AI runtime integration for efficient inference execution.
- Support multiple AI model running at the same time with DRP-AI driver.
- Support for both YUV422 and RGB image formats.

Implemented Models
^^^^^^^^^^^^^^^^^^^^^^^^^^

The following AI models are implemented, optimized and ready to use with DRP-AI acceleration.

**Object Detection Models**

- YOLOX Base Model
- YOLOX Hand Detection
- YOLOX Pascal VOC Detection
- Gold YOLOX Hand Detection
- YOLOv8 Base Model
- YOLOv8n Rock Paper Scissors Gesture Detection

**Pose Estimation Models**

- HRNetV2 Base Model
- HRNetV2 Hand Landmark Model
- RTMPose Base Model
- RTMPose Hand Model
- MediaPipe Hand Landmark Model

Package structure
^^^^^^^^^^^^^^^^^^^^^^^

The **rzv_model** package is organized into the following structure:


.. code-block:: console

    rzv_model/
      ├── CMakeLists.txt
      ├── config
      │   └── models
      ├── include
      │   └── rzv_model
      │       ├── base_model.hpp
      │       ├── model_specific.hpp
      │       └── utils.hpp
      ├── package.xml
      ├── README.md
      └── src
          ├── base_model.cpp
          ├── model_specific.cpp
          ├── platform
          │   ├── MeraDrpRuntimeWrapper.cpp
          │   ├── MeraDrpRuntimeWrapper.h
          │   ├── PreRuntime.h
          │   └── PreRuntimeV2H.cpp
          └── utils.cpp

- The **config/models** directory contains configuration files for each supported AI model, including the output from the DRP-AI TVM conversion step.
- The **include/rzv_model** directory contains the header files defining the base model class and utility functions.
- The **src** directory contains the implementation of the base model, platform-specific runtime wrappers, and utility functions.
- The **CMakeLists.txt** and **package.xml** files are used for building and packaging the library.

Architecture
^^^^^^^^^^^^^^^^^^^^^^^

The **rzv_model** package follows a **modular architecture** designed for extensibility, maintainability, and efficient deployment on DRP-AI.

- **Base Model:**
  Provides the `BaseModel` class, which implements shared functionalities such as model loading, pre-processing, inference execution, and result handling.

- **Model-Specific Implementations:**
  Each AI model (e.g., YOLOX, YOLOv8, HRNet, RTMPose) inherits from the base class and extends it with task-specific logic such as detection parsing or key point extraction.

- **Utility Modules:**
  Contain helper functions for image pre-processing, tensor conversion, normalization, and post-processing visualization.

This modular design enables developers to easily integrate new AI models and customize pre-processing or inference pipelines for various use cases on the RZ/V2H platform.

.. _how_to_use_rzv_model_package:

How to use the rzv_model package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Input requirements files
""""""""""""""""""""""""""""""""""

After complete the :ref:`Exchange AI model  <exchange_ai_model>` step, the output should contain the compiled model files including:

.. code-block:: console

    Ouput_Directory/
      |-- deploy.json
      |-- deploy.params
      |-- deploy.so
      |-- input_0.bin
      `-- preprocess
          |-- addr_map.txt
          |-- aimac_cmd.bin
          |-- aimac_desc.bin
          |-- aimac_param_cmd.bin
          |-- aimac_param_desc.bin
          |-- drp_config.mem
          |-- drp_desc.bin
          |-- drp_param.bin
          |-- drp_param_info.txt
          `-- weight.bin

Also, to enable multiple AI models running simultaneously with the DRP-AI driver, a special file called ``addr_map.txt`` is required.

This ``addr_map.txt`` file is for the model in the inference phase. The purpose is to get the memory size used as multiple models scenario each model shall be allocated a memory block in advance before running.

This file is different from the one generated in the **Ouput_Directory** folder.

To obtain the ``addr_map.txt`` file, locate it in the **temp** folder created during the model conversion process.

For example, if you compile the model in the **/drp-ai_tvm/tutorials/** directory, a **temp** folder will be generated automatically.

You can find the ``addr_map.txt`` file in the following path:

``/drp-ai_tvm/tutorials/temp/<date_time>/tvmgen_default_tvmgen_default_mera_drp_main_*/drp_compilation_output/``

There might be several subdirectories representing different inference stages (executed by DRP or CPU) that each contain an ``addr_map.txt`` file.

The correct file to use is the one with the **largest memory address allocation**, as it represents the final and complete memory size used by the AI Model with DRP-AI driver.

**How to calculate the memory size used by the model?**

- Open the ``addr_map.txt`` file with a text editor.

This file contains several lines, each representing a memory block with its start address and size. For example:

.. code-block:: console

    data_in 32ec4c0 dc00
    data 32fa0c0 41040
    data_out 333b100 12c00
    work 334dd00 80
    weight 334dd80 bd80
    drp_config 3359b00 87480
    aimac_param_cmd 33e0f80 140
    aimac_param_desc 33e10c0 50
    aimac_cmd 33e1140 1300
    aimac_desc 33e2440 170
    drp_param 33e2600 350
    drp_desc 33e2980 380

- Calculate the total memory size used by the model by summing up the sizes of ``drp_desc`` and subtract the ``data_in`` start address. For example:

  - In this example, the ``data_in`` start address is ``32ec4c0`` and the ``drp_desc`` end address is ``33e2d00`` (start address ``33e2980`` + size ``380``).

  - Therefore, the total memory size used by the model is:

    ``33e2d00 - 32ec4c0 = 0x0f8800`` (in hexadecimal) or ``1,028,608 bytes`` (in decimal).

- Do the same for other ``addr_map.txt`` files to get their memory sizes.

Construct the Model configuration files
""""""""""""""""""""""""""""""""""""""""""

To construct the model, import the **Input requirements files** above with the following structure:

.. code-block:: console

    Model_name/
      |-- addr_map.txt
      |-- deploy.json
      |-- deploy.params
      |-- deploy.so
      `-- preprocess
          |-- addr_map.txt
          |-- aimac_cmd.bin
          |-- aimac_desc.bin
          |-- aimac_param_cmd.bin
          |-- aimac_param_desc.bin
          |-- drp_config.mem
          |-- drp_desc.bin
          |-- drp_param.bin
          |-- drp_param_info.txt
          `-- weight.bin

Note that, the top-level ``addr_map.txt`` file is required for multiple models running with DRP-AI driver.

This `Model_name` folder will place under the **config/models** directory of the **rzv_model** package.

Post-processing configuration
""""""""""""""""""""""""""""""""""

Each model may have different post-processing requirements based on its specific task (e.g., object detection, pose estimation).

To customize the post-processing behavior, you can modify the corresponding model-specific implementation files located in the **src/** directory of the **rzv_model** package.

The details of post-processing configuration are not covered in this section. Please refer to the example in the next section for a clearer understanding.

.. hint::

    There are some sample applications from Renesas, which have the custom post-processing for the specific models.

    You can refer to these applications for reference on implementing the AI model post-processing logic:

    - `RZ/V AI Applications Repository <https://github.com/renesas-rz/rzv_ai_sdk/tree/main>`_.

    - `Ignitarium Renesas - RZ/V AI Applications <https://github.com/Ignitarium-Renesas/rzv_ai_apps>`_.

Example of using rzv_model package
"""""""""""""""""""""""""""""""""""""

Load the model and perform inference using the **rzv_model** package with the following code snippet:

.. code-block:: cpp

    // Example using HRNetV2 Hand Landmark model
    auto model = std::make_unique<rzv_model::HRNetV2HandLandmarkModel>();
    model->load(model_path);

    // Prepare input
    rzv_model::ModelInput input{image, roi};

    // Run inference
    auto result = model->run<rzv_model::KeyPointResult>(input);
