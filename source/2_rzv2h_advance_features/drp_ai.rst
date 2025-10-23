DRP-AI
==============

Overview
-------------------

The RZ/V2H DRP-AI Driver enables the use of the DRP-AI (Dynamically Reconfigurable Processor for AI) hardware accelerator on the RZ/V2H platform.

This driver facilitates efficient execution of AI inference tasks by offloading computations to the DRP-AI, thereby improving performance and reducing CPU load and high power efficiency.

The DRP-AI device driver provides an interface to easily handle the AI inference execution function of DRP-AI. So that there is no hardware knowledge required for the user.

.. seealso::

   For more detail information about the DRP-AI Driver, refer to the `RZ/V2H DRP-AI Driver <https://github.com/renesas-rz/rzv2h_drp-ai_driver/tree/main>`_.

Concepts
-------------------

.. _drp_concepts:

What is Dynamically Reconfigurable Processor (DRP)?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DRP is the hardware IP (Intellectual Property) that can dynamically change its hardware (arithmetic logic circuit) configuration.

Its main advantages are the ability to reduce surface area and power consumption whilst maintaining high performance.

`Dynamic Reconfiguration` can change the configuration of the arithmetic circuit in execution.

This image below shows an example of dynamic reconfiguration:

.. figure:: images/DRP.png
   :alt: DRP Dynamic Reconfiguration
   :align: center
   :width: 500px

   DRP Dynamic Reconfiguration

What is DRP-AI?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

DRP-AI is a specialized version of DRP designed specifically for AI (Artificial Intelligence) processing tasks.

By combine the DRP and AI-MAC (AI Matrix Arithmetic Circuit) to accelerate AI inference tasks efficiently.

.. seealso::

    On the RZ/V2H platform, the DRP-AI3 is used as the DRP-AI hardware accelerator. For more information about the DRP-AI3, refer to the:

    **DRP-AI3 White Paper**: `Next Generation Highly Power-Efficient AI Accelerator (DRP AI3) <https://www.renesas.com/en/document/whp/next-generation-highly-power-efficient-ai-accelerator-drp-ai3-10x-faster-embedded-processing?srsltid=AfmBOor4_FWpRHSIG2IQFvCDBKsJY2GUBgCK2YG18EGS2dKGWvRw9YOL>`_.

    **Software Package**: `AI Accelerator: DRP-AI <https://www.renesas.com/en/software-tool/ai-accelerator-drp-ai>`_.

**DRP-AI Driver Architecture**:

- Buffer to reuse input data
- Switches to avoid zero data processing
- Controller to optimize operation flow (scheduling)

.. figure:: images/DRP-AI.png
   :alt: DRP-AI Driver Architecture
   :align: center
   :width: 500px

   DRP-AI Driver Architecture

.. toctree::
    :hidden:
    :maxdepth: 3

    byom_drp_ai