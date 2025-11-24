BYOM AI model support
------------------------

The DRP-AI supports BYOM (Bring Your Own Model) AI models, allowing users to deploy their custom-trained AI models on the RZ/V2H platform.

.. _byom_drp_ai:

Getting Started
^^^^^^^^^^^^^^^^^^^^^^^^

To enable BYOM support, users need to convert their AI models into a format compatible with the DRP-AI using the:

`Extension package of TVM Deep Learning Compiler for Renesas DRP-AI accelerators powered by EdgeCortix MERA™ (DRP-AI TVM)`.

This package provides the necessary tools and libraries to facilitate the conversion process, ensuring that the models can leverage the DRP-AI's capabilities effectively.

.. seealso::

   For more information about the DRP-AI TVM extension package, refer to the:

   - `DRP-AI TVM Extension Package <https://github.com/renesas-rz/rzv_drp-ai_tvm>`_.
   - `DRP-AI TVM on RZ/V series <https://renesas-rz.github.io/rzv_drp-ai_tvm/>`_.

Install the DRP-AI TVM extension package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To install the DRP-AI TVM extension package, follow the instructions provided in the `rzv_ai_toolchain_docker repository <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/ai/rzv_ai_toolchain_docker/-/tree/master?ref_type=heads>`_.

.. important::

   Please use the RZ/V2H SDK environment when installing the DRP-AI TVM extension package. Do not use SDKs provided by other platforms.

BYOM Development Flow
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The typical development flow for deploying BYOM AI models on the RZ/V2H platform using the DRP-AI TVM extension package involves the following steps:

.. figure:: images/BYOM_Flow.png
   :alt: BYOM Development Flow
   :align: center
   :width: 800px

   BYOM Development Flow

1. Training data collection: Gather and prepare the dataset required for training the AI model.
2. Model training: Use a deep learning framework (e.g., TensorFlow, PyTorch, ONNX) to train the AI model on the collected dataset.
3. Exchange AI model: Convert the trained AI model into a format compatible with the DRP-AI using the DRP-AI TVM extension package.
4. Deployment: Deploy the converted model onto the RZ/V2H platform and integrate it with the DRP-AI Driver for inference execution.

.. hint::

   In the deployment step, the ready to use **rzv_model** package is provided to simplify the integration of compiled models with the DRP-AI Driver.

   We also provide some example of complete flow about developing some popular AI models with DRP-AI TVM extension package. Refer to the :ref:`DRP-AI with rzv_model tutorials <drp_ai_with_rzv_model_tutorials>` for more details.

Training data collection and Model training
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For training data collection and model training, users can utilize popular deep learning frameworks such as `TensorFlow <https://www.tensorflow.org/>`_, `PyTorch <https://pytorch.org/>`_, or `ONNX <https://onnx.ai/>`_.

.. seealso::

   List of AI models that Renesas has verified for conversion with the DRP-AI TVM: `Model list for RZ/V2H <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/model_list/Model_List_V2H.md>`_.

   Note that, the above list is not exhaustive, and users can attempt to convert other models as well.

.. _exchange_ai_model:

Exchange AI model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To convert the trained AI model into a format compatible with the DRP-AI for V2H target device, follow the instructions provided in the DRP-AI TVM extension package repository: `Compile with Sample Scripts <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/tutorials/tutorial_RZV2H.md>`_.

.. tip::

   Follow the convert tips to easily convert your AI model: `How to convert <https://github.com/renesas-rz/rzv_drp-ai_tvm/tree/main/docs/model_list/how_to_convert>`_.

Deployment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For deploying the converted AI model onto the RZ/V2H platform, the `rzv_model` package is provided to facilitate the integration process.

The `rzv_model` package providing AI model abstractions and implementations for Renesas RZ/V MPU platforms.

This package implements various models for computer vision tasks using the DRP-AI accelerator.

Refer to the next section for more details about the `rzv_model` package and its usage.