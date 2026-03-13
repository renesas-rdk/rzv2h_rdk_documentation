BYOM AI model support
---------------------

The DRP-AI supports BYOM (Bring Your Own Model) AI models, allowing users to deploy custom-trained AI models on the RZ/V2H platform.

.. note::

   This section **covers only the compilation and deployment of BYOM AI models** by using the DRP-AI TVM extension package.

   For information about how to customize your AI application with BYOM AI models, refer to the BYOM AI application tutorial.

.. seealso::

   For more information about the DRP-AI TVM, also known as the RUHMI (Robust Unified Heterogeneous Model Integration) extension package, refer to the following resources:

   - `DRP-AI TVM Extension Package <https://github.com/renesas-rz/rzv_drp-ai_tvm>`_: Software package for converting and deploying AI models on the RZ/V2H platform with DRP-AI support.
   - **Recommended:** `DRP-AI TVM on RZ/V series <https://renesas-rz.github.io/rzv_drp-ai_tvm/>`_: Documentation and tutorials for using the DRP-AI TVM extension package to convert and deploy AI models on the RZ/V2H platform.
   - `DRP-AI Translator i8 <https://www.renesas.com/en/software-tool/drp-ai-translator-i8#documents>`_: Documentation for the DRP-AI Translator i8, a tool used for converting AI models into a format compatible with the DRP-AI.

   It is highly recommended that you refer to the documentation and tutorials at the link above for detailed instructions on how to convert and deploy your own AI models by using the DRP-AI TVM extension package.

.. _byom_drp_ai:

Getting Started
^^^^^^^^^^^^^^^

To enable BYOM support, users need to convert their AI models into a format compatible with the DRP-AI using the following package:

`Extension package of TVM Deep Learning Compiler for Renesas DRP-AI accelerators powered by EdgeCortix MERA™ (DRP-AI TVM) <https://github.com/renesas-rz/rzv_drp-ai_tvm>`_

This package provides the necessary tools and libraries to facilitate the conversion process, ensuring that the models can effectively leverage the capabilities of the DRP-AI.

Install the DRP-AI TVM extension package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To install the DRP-AI TVM extension package, follow the instructions provided in the `RZ/V DRP-AI TVM setup <https://github.com/renesas-rz/rzv_drp-ai_tvm/tree/main/setup>`_.

BYOM Development Flow
^^^^^^^^^^^^^^^^^^^^^

The typical development flow for deploying BYOM AI models on the RZ/V2H platform using the DRP-AI TVM extension package involves the following steps:

.. figure:: ../../../images/BYOM_Flow.png
   :alt: BYOM Development Flow
   :align: center
   :width: 800px

   BYOM Development Flow

#. **Training data collection**: Gather and prepare the dataset required for training the AI model.

#. **Model training**: Use a deep learning framework, such as TensorFlow, PyTorch, or ONNX, to train the AI model on the collected dataset.

   For training data collection and model training, users can use popular deep learning frameworks such as `TensorFlow <https://www.tensorflow.org/>`_, `PyTorch <https://pytorch.org/>`_, or `ONNX <https://onnx.ai/>`_.

   .. seealso::

      List of AI models that Renesas has verified for conversion with DRP-AI TVM: `Model list for RZ/V2H <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/model_list/Model_List_V2H.md>`_.

      Note that the above list is not exhaustive, and users can also attempt to convert other models.

#. **Exchange AI model**: Convert the trained AI model into a format compatible with the DRP-AI using the DRP-AI TVM extension package.

   Follow the next section for detailed instructions on how to convert AI models using the DRP-AI TVM extension package.

#. **Deployment**: Deploy the converted model onto the RZ/V2H platform and integrate it with the DRP-AI Driver for inference execution.

.. tip::

   In the deployment step, the ready-to-use **rzv_model** package is provided to simplify the integration of compiled models with the DRP-AI Driver.

.. _exchange_ai_model: