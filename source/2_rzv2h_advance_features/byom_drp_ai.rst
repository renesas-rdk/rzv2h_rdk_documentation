BYOM AI model support
------------------------

The DRP-AI supports BYOM (Bring Your Own Model) AI models, allowing users to deploy their custom-trained AI models on the RZ/V2H platform.

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

To install the DRP-AI TVM extension package, follow the instructions provided in the package's GitHub repository: `Installation <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/setup/SetupV2H.md#installation>`_.

We recommend installing `DRP-AI TVM with Docker <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/setup/SetupV2H.md#installing-drp-ai-tvm1-with-docker-rzv2h-and-rzv2n>`_ for ease of setup and to avoid dependency issues.