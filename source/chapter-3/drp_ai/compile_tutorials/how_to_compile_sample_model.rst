.. _how_to_compile_sample_model:

How to Compile Sample Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following is an example of how to compile the model with the DRP-AI TVM extension package.

.. figure:: ../../../images/overview.png
   :width: 100%
   :alt: Overview
   :align: center

   Overview

There are 3 tutorials about AI deployment tools for RZ/V2H RDK.

- For an easy understanding of DRP-AI TVM by using a sample model, follow the instructions on this page.
- To learn how to prune your own model, `click here <https://renesas-rz.github.io/rzv_drp-ai_tvm/pruning.html>`_.
- To learn how to compile your own model, :ref:`click here <how_to_compile_your_own_model>`.

Getting Started
"""""""""""""""

The next section provides a tutorial on how to compile a sample ONNX model using the DRP-AI TVM extension package.

Follow the instructions to set up the environment, compile the model using the sample script, and confirm the output.

Work flow
"""""""""

From this point on, follow these steps.

.. figure:: ../../../images/workflow.png
   :width: 800px
   :alt: Workflow
   :align: center

   Workflow

Setup Environment
"""""""""""""""""

Refer to `Installing DRP-AI TVM1 with Docker (Mera2) <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/setup/README.md#installing-drp-ai-tvm1-with-docker-mera2>`_ to set up the environment before you begin this workflow.

.. note::

   There is no need to use Ubuntu 22.04 for the installation, as the DRP-AI TVM extension package can be installed on any Linux distribution that supports Docker.

   We recommend using an Ubuntu 24.04 machine for the DRP-AI TVM extension package, as it provides a consistent and isolated environment for ROS 2 Jazzy development.

Compile AI models
"""""""""""""""""

All subsequent operations are performed within the container created in the above steps.

To enter the container, execute the following command in the terminal.

.. code-block:: bash

   docker start drp-ai_tvm_${PRODUCT,,}_container_${USER}
   docker exec -it drp-ai_tvm_${PRODUCT,,}_container_${USER} bash

Download an ONNX model
~~~~~~~~~~~~~~~~~~~~~~

In this example, ``resnet18`` is used.

First, download the ONNX model file from the following URL by using the following command.

.. code-block:: bash

   cd $TVM_ROOT/tutorials/
   wget https://github.com/onnx/models/raw/main/validated/vision/classification/resnet/model/resnet18-v1-7.onnx

Compile an ONNX model
~~~~~~~~~~~~~~~~~~~~~

Compile the ONNX model you just downloaded by using the prepared sample script.

.. code-block:: bash

   python3 compile_onnx_model_quant.py ./resnet18-v1-7.onnx -o resnet18_onnx -t $SDK -d $TRANSLATOR -c $QUANTIZER --images $TRANSLATOR/../GettingStarted/tutorials/calibrate_sample/

Confirm the output.

.. code-block:: bash

   ls resnet18_onnx

Example output:

.. code-block:: bash

   input_0.bin  preprocess  interpreter_out  project.mdp  mera.plan  sub_0000__CPU_DRP_TVM  model_subgraphs.json

They are the compiled model files that can be deployed on the RZ/V2H platform with the DRP-AI Driver.

To deploy the model files on the RZ/V2H platform, please refer to the :ref:`How to Deploy Model Files <how_to_deploy_model_files>` section for more details.