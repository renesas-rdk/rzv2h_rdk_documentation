.. _how_to_compile_your_own_model:

How to compile Your Own Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes how to compile your own AI model for the RZ/V2H RDK platform by using the DRP-AI TVM extension package.

.. seealso::

   Detailed steps are available here: `Compile with Sample Scripts (RZ/V2H, RZ/V2N) <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/tutorials/tutorial_RZV2H.md>`_.

   For information about MERA, see `MERA™ (Model Efficiency Runtime Accelerator) <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/About_mera.md>`_.

Compile your own model for RZ/V2H
"""""""""""""""""""""""""""""""""

You can compile your model by using `the sample script compile_onnx_model_quant.py <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/tutorials/compile_onnx_model_quant.py>`_.

However, because the camera input size, camera data format, and AI model input shape differ by application and model, you must update the script parameters and command options to match your use case.

ResNet18 is used in the :ref:`How to Compile Sample Model <how_to_compile_sample_model>` tutorial. This page explains how to adapt that flow from ResNet18 to YOLOX as an example.

This section also provides general guidance that you can apply to your own model.

.. figure:: ../../../images/HowToCompileYourOwnModel.png
   :width: 500px
   :alt: Overview of compiling your own model

   Overview of compiling your own model

Tutorial
""""""""

.. raw:: html

   <div class="ratio ratio-16x9">
     <iframe src="https://players.brightcove.net/5260471205001/default_default/index.html?videoId=6361753833112"
     allowfullscreen
     webkitallowfullscreen
     mozallowfullscreen
     width="100%"
     height="500px"
     ></iframe>
   </div>

.. warning::

   The AI application shown in the video cannot run on the RZ/V2H RDK platform because the RZ/V2H RDK uses the Ubuntu 24.04 LTS operating system, which is not compatible with the Yocto image used in the tutorial.

   Detailed steps for running the RZ/V2H AI application on the RZ/V2H RDK platform are provided in the :ref:`Renesas AI Applications <renesas_ai_application>` section.

Workflow
""""""""

This section first provides an overview of the process.

From this point on, follow the steps in this chapter.

In this chapter, YOLOX is used as an example.

.. figure:: ../../../images/compile_flow.png
   :width: 100%
   :alt: Compile workflow
   :align: center

   Compile workflow

Set up the environment
""""""""""""""""""""""

Refer to `Installing DRP-AI TVM1 with Docker (Mera2) <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/setup/README.md#installing-drp-ai-tvm1-with-docker-mera2>`_ to set up the environment before you begin this workflow.

.. note::

   There is no need to use Ubuntu 22.04 for the installation, as the DRP-AI TVM extension package can be installed on any Linux distribution that supports Docker.

   We recommend using an Ubuntu 24.04 machine for the DRP-AI TVM extension package, as it provides a consistent and isolated environment for ROS 2 Jazzy development.

From this point on, perform all work inside the Docker container.

Move to the working directory, ``$TVM_ROOT/tutorials/``.

.. code-block:: console

   root@docker_hostname:# cd $TVM_ROOT/tutorials/

Next, copy the model that you want to compile into your working directory.

The ``docker cp`` command is useful for copying files into the Docker container.

If you pruned the YOLOX model by using the DRP-AI Extension Pack on `this page <https://renesas-rz.github.io/rzv_drp-ai_tvm/pruning.html>`_, you can use the pruned model.

If you did not prune the model, you can use the prepared model: ``$TVM_ROOT/how-to/sample_app_v2h/app_yolox_cam/yolox-S_VOC.onnx``.

Confirm the model information
"""""""""""""""""""""""""""""

First, determine the input format of the model and the application.

`Netron <https://netron.app/>`_ is useful for checking model input information such as the input shape.

Also check the application input format in the application source code.

.. figure:: ../../../images/netron.png
   :alt: Netron model input information
   :align: center
   :width: 500px

   Netron model input information

In the YOLOX example, confirm the following parameters:

- Model input shape: ``[1, 3, 640, 640]``
- Application input shape: ``[1920, 1920, 2]``
- Application input format: ``YUYV_422``

Modify the sample script
""""""""""""""""""""""""

Next, modify the preprocessing statements in the sample script so that they match your model.

This step is required because, when you run inference with your own model, the input image must be preprocessed to match the model input.

The sample script already includes preprocessing, but you must update it according to the model information confirmed in the previous section.

Calibration
~~~~~~~~~~~

In `compile_onnx_model_quant.py line 101 <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/tutorials/compile_onnx_model_quant.py#L101>`_, the following process is used for calibration data.

See `Tips for INT8 Quantization <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/tutorials/tutorial_RZV2H.md#tips-for-int8-quantization>`_ for more information about calibration.

Change the code as follows to use YOLOX in this example.

.. code-block:: diff

   def pre_process_imagenet_pytorch(img, mean=[0.485, 0.456, 0.406], stdev=[0.229, 0.224, 0.225], dims=None, need_transpose=False):

       img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
       img = Image.fromarray(img)
   -   img = F.resize(img, 256, Image.BILINEAR)
   +   img = F.resize(img, 640, Image.BILINEAR)
   -   img = F.center_crop(img, 224)
   +   img = F.center_crop(img, 640)
   -   img = F.to_tensor(img)
   +   img = F.pil_to_tensor(img)
   -   std = stdev
   -   img = F.normalize(img, mean, std, inplace=False)
       if not need_transpose:
           img = img.permute(1, 2, 0) # NHWC
       img = np.asarray(img, dtype='float32')
       return img

For example, you can modify the script as follows.

.. code-block:: console

   root@docker_hostname:# sed -i -e 's/256/640/g' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e 's/ 224/ 640/g' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e 's/to_tensor/pil_to_tensor/g' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e '/std = stdev/d' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e '/F.normalize/d' compile_onnx_model_quant.py

.. note::

   These commands remove normalization and change the preprocessing size from 224 to 640.
   Review the script manually after using ``sed`` to make sure that only the intended lines were changed.

Pre-runtime
~~~~~~~~~~~

Pre-runtime is a runtime that executes preprocessing on DRP-AI before the AI model runs.

For clarity, describe the workflow as consisting of two preprocessing stages.

#. **Software-side preprocessing**

   The application first receives the input image from the source, such as a USB camera, MIPI camera, video stream, or image buffer.

   At this stage, the application may perform software-side handling before passing the data to DRP-AI Pre-runtime. For example:

   - Acquire the image from the camera
   - Arrange the input buffer
   - Handle source-specific input format differences
   - Perform optional cropping or other basic preprocessing in software

#. **DRP-AI Pre-runtime preprocessing (hardware-side preprocessing)**

   The image is then passed to DRP-AI Pre-runtime.

   Pre-runtime performs the preprocessing defined in the script, such as:

   - Resize
   - Color conversion
   - Normalization
   - Layout conversion

.. note::

   The input to Pre-runtime must match the actual application input format.

   The output from Pre-runtime must match the model input format.

For example, if the camera outputs ``YUYV_422`` data at ``1920 x 1080``, the following input settings must match that camera output:

- ``config.shape_in``
- ``config.format_in``
- ``config.order_in``
- ``config.type_in``

If the model is YOLOX and expects input such as ``640 x 640 x 3`` in ``RGB``, the following output settings must match the model input:

- ``config.shape_out``
- ``config.format_out``
- ``config.order_out``
- ``config.type_out``

You can think of Pre-runtime as a conversion stage between the application input and the model input.

.. code-block:: console

   Input image
   -> optional software-side preprocessing in the application
   -> DRP-AI Pre-runtime preprocessing
   -> model input tensor
   -> inference

In other words, there can be two preprocessing steps:

#. The application may first perform preprocessing in software.
#. DRP-AI Pre-runtime then converts that data into the exact format required by the model.

For YOLOX, a simple explanation is as follows:

- The camera may output ``1920 x 1080`` in ``YUYV_422``
- The model may require ``640 x 640 x 3`` in ``RGB``
- Pre-runtime converts the camera input into the model input format

In `compile_onnx_model_quant.py line 273 <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/tutorials/compile_onnx_model_quant.py#L273>`_, the preprocessing runtime is compiled by using the following process.

See `here <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/PreRuntime.md#2412-config>`_ for more information about PreRuntime.

Change the code as follows to use YOLOX in this example.

.. tip::

   If you are using a USB camera, you can define the preprocessing input data as ``1920 x 1920``, as shown in the following code.

   This is the maximum input size for Pre-runtime, and it allows you to use the same preprocessing settings for all USB cameras.

.. code-block:: diff

   # 4. Compile pre-processing using DRP-AI Pre-processing Runtime Only for RZ/V2H
   # 4.1. Define the pre-processing data
   config = preruntime.Config()

   # 4.1.1. Define input data of preprocessing
   - config.shape_in     = [1, 480, 640, 3]
   + config.shape_in     = [1, 1920, 1920, 2]
   - config.format_in    = drpai_param.FORMAT.BGR
   + config.format_in    = drpai_param.FORMAT.YUYV_422
   config.order_in     = drpai_param.ORDER.HWC
   config.type_in      = drpai_param.TYPE.UINT8

   # 4.1.2. Define output data of preprocessing (Will be model input)
   model_shape_in = list(opts["input_shape"])
   config.shape_out    = model_shape_in
   config.format_out   = drpai_param.FORMAT.RGB
   config.order_out    = drpai_param.ORDER.CHW
   config.type_out     = drpai_param.TYPE.FP32
   # Note: type_out depends on DRP-AI TVM[*1]. Usually FP32.

   # 4.1.3. Define operators to be run.
   r = 255
   - cof_add = [-m*r for m in mean]
   - cof_mul = [1/(s*r) for s in stdev]
   config.ops = [
       op.Resize(model_shape_in[3], model_shape_in[2], op.Resize.BILINEAR),
       op.Normalize(cof_add, cof_mul)
   ]

   # 4.2. Run DRP-AI Pre-processing Runtime
   preruntime.PreRuntime(config, opts["output_dir"]+"/preprocess", PRODUCT)

For example, you can modify the script as follows.

By default, the sample uses a USB camera. In this example, change it to use a MIPI camera.

.. code-block:: console

   root@docker_hostname:# sed -i -e 's/480, 640, 3/1920, 1920, 2/g' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e 's/FORMAT.BGR/FORMAT.YUYV_422/g' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e '/cof_add/d' compile_onnx_model_quant.py
   root@docker_hostname:# sed -i -e '/cof_mul/d' compile_onnx_model_quant.py

This completes the sample script modification.

.. tip::

   At ``# 4.1.1. Define input data of preprocessing``, this section defines the input format of Pre-runtime, such as the image format and input shape.

   This setting must match the actual output of software-side preprocessing in the application or the camera input format.

   At ``# 4.1.2. Define output data of preprocessing (Will be model input)``, this section defines the output format of Pre-runtime.

   This setting must match the model input format.

   For example, if YOLOX expects an input tensor corresponding to ``640 x 640 x 3`` in ``RGB``, the DRP-AI Pre-runtime output must be configured to generate that format.

.. note::

   A simple way to understand this flow is as follows:

   - The application receives the original input image
   - The application may perform software-side preprocessing
   - DRP-AI Pre-runtime then performs hardware-side preprocessing
   - The final Pre-runtime output must match the model input exactly

   Even if the camera input format and the model input format differ, Pre-runtime acts as the conversion stage between them.

Compile the AI model
""""""""""""""""""""

Using the modified sample script from the previous section, compile YOLOX with the following command.

.. code-block:: console

   root@docker_hostname:# python3 compile_onnx_model_quant.py \
      $TRANSLATOR/../onnx_models/YoloX-S_VOC_sparse70.onnx \
      -o yolox_cam \
      -t $SDK \
      -d $TRANSLATOR \
      -c $QUANTIZER \
      -s 1,3,640,640 \
      --images $TRANSLATOR/../GettingStarted/tutorials/calibrate_sample/

This sample script accepts options that specify the information required for compilation.

In this example, the following options are used.

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Option
     - Description
   * - argument
     - Model file
   * - ``-o``
     - Output directory name
   * - ``-t``
     - Path to the toolchain
   * - ``-d``
     - Path to DRP-AI Translator
   * - ``-c``
     - Path to DRP-AI Quantizer
   * - ``-s``
     - Input shape of the target model
   * - ``--images``
     - Directory that contains calibration images

Detailed option information is provided in the following table.

.. list-table::
   :widths: 28 47 25
   :header-rows: 1

   * - Option
     - Description
     - Example
   * - ``-o``, ``--output_dir``
     - Output directory to save compile results
     - ``-o DIR_NAME``
   * - ``-t``, ``--toolchain_dir``
     - Cross-compilation toolchain root directory
     - ``-t DIR_NAME``
   * - ``-d``, ``--drp_compiler_dir``
     - DRP-AI Translator root directory
     - ``-d DIR_NAME``
   * - ``-v``, ``--drp_compiler_version``
     - DRP-AI Translator version, such as ``091`` or ``100``
     - ``-v 100``
   * - ``-c``, ``--quantization_tool``
     - Quantization tool directory
     - ``-c DIR_NAME``
   * - ``-s``, ``--input_shape``
     - AI model input node shape
     - ``-s 1,3,224,224``
   * - ``-i``, ``--input_name``
     - AI model input node name. This option is not required for PyTorch models.
     - ``-i INPUT_NAME``
   * - ``-n``, ``--num_frame``
     - Number of images to use for calibration. Default: ``1``.
     - ``-n 10``
   * - ``--images``
     - Directory that contains calibration images
     - ``--images DIR_NAME``
   * - ``-r``, ``--record_dir``
     - Calibration data record directory
     - ``-r DIR_NAME``
   * - ``--level``
     - Optimization level at compile time.

       The default value is ``1``, which uses optimal settings.

       If you set this option to ``0``, more complex models can be deployed,

       but inference speed is slower.
     - ``--level 0``
   * - ``-q``, ``--fp16``
     - Convert to FP16
     - ``-q``
   * - ``-f``, ``--cpu_data_type``
     - CPU data type, such as ``float16`` or ``float32``. Default: ``float16``.
     - ``-f float16``
   * - ``-p``, ``--quantization_option``
     - DRP-AI quantization option
     - ``-p "-az"``

For example, you can compile your model by using the sample script with options such as the following.

.. code-block:: console

   python3 compile_onnx_model_quant.py \
      ./resnet50-v1-7.onnx \
      -o resnet50_v1_onnx \
      -t $SDK \
      -d $TRANSLATOR \
      -c $QUANTIZER \
      --images $TRANSLATOR/../GettingStarted/tutorials/calibrate_sample/ \
      -v 100

.. tip::

   If the compilation process fails, try adding the ``--mera1`` option to the command above to use the MERA1 compiler instead of the MERA2 compiler.

   If you have any questions or need further assistance, feel free to contact us on GitHub:
   `renesas-rdk <https://github.com/renesas-rdk/rzv2h_rdk_documentation>`_.