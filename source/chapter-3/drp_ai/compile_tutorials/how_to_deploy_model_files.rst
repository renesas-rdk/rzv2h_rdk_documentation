.. _how_to_deploy_model_files:

How to Deploy Model Files
^^^^^^^^^^^^^^^^^^^^^^^^^

After you compile your AI model by using the DRP-AI TVM extension package, you can deploy the generated model files to the RZ/V2H RDK.

This section provides a step-by-step guide for deploying the model files.

Collect the model files
"""""""""""""""""""""""

After compiling the model, you will obtain the following files.

There are two kinds of model files: one for the **mera1** model and one for the **mera2** model.

For more information about the differences between mera1 and mera2, see `About Mera <https://github.com/renesas-rz/rzv_drp-ai_tvm/blob/main/docs/About_mera.md>`_.

Example of ``mera1`` model files:

.. code-block:: text

   output_directory/
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

The other files in the output directory, such as ``input_0.bin`` and ``model_subgraphs.json``, are not required for deployment on the RZ/V2H RDK board.

They are using for other purposes, such as debugging or reference for the model structure, and can be ignored for deployment.

You can safely ignore them or copy them to the board when deploying the model files.

Calculate the memory size used by the model
"""""""""""""""""""""""""""""""""""""""""""

To enable multiple AI models running simultaneously with the DRP-AI driver, a special file called ``addr_map.txt`` is required, **it is different from the one generated in the output_directory** folder.

This ``addr_map.txt`` file is for the model in the inference phase. The purpose is to get the memory size used as multiple models scenario each model shall be allocated a memory block in advance before running.

To obtain the ``addr_map.txt`` file, locate it in the **temp** folder created during the model conversion process.

For example, if you compile the model in the **/drp-ai_tvm/tutorials/** directory, a **temp** folder will be generated automatically.

You can find the ``addr_map.txt`` file in the following path:

``/drp-ai_tvm/tutorials/temp/<date_time>/tvmgen_default_tvmgen_default_mera_drp_main_*/drp_compilation_output/``

There might be several sub-directories representing different inference stages (executed by DRP-AI or CPU) that each contain an ``addr_map.txt`` file.

The correct file to use is the one with the largest memory address allocation, corresponding to the **maximum drp_desc value**, as it represents the final and complete memory size used by the AI Model with DRP-AI driver.

**How to calculate the memory size used by the model?**

- Open the ``addr_map.txt`` file with a text editor.

  This file contains several lines, each representing a memory block with its start address and size. For example:

  .. code-block:: console
     :emphasize-lines: 12

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

- Calculate the total memory size by summing the sizes of all ``drp_desc`` entries:

  In this example, the description of ``drp_desc`` is: start address ``33e2980`` and size ``380``.

  Therefore, the total memory size for ``drp_desc`` is: ``33e2980`` (start address) + ``380`` (size) = ``33e2d00``.

- Find the correct ``addr_map.txt`` file that contains the largest ``drp_desc`` value, corresponding to the total memory size used by the AI model with the DRP-AI driver.

- Copy this ``addr_map.txt`` file to the top-level directory of the model configuration folder, which will be used for deployment on the RZ/V2H RDK board.

Final model configuration for deployment
""""""""""""""""""""""""""""""""""""""""

After you have the model files and the correct ``addr_map.txt`` file, you can construct the final model configuration for deployment on the RZ/V2H RDK board.

The final model configuration should have the following structure:

Example of ``mera1`` model files:

.. code-block:: text
   :emphasize-lines: 2

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
   :emphasize-lines: 2

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

Use this final model configuration to deploy the AI model on the RZ/V2H RDK with the DRP-AI driver.