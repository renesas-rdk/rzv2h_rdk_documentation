Application Development with RZ/V2H RDK
=======================================

This section describes the development of ROS 2 applications for the Renesas RZ/V2H RDK board.

Target `ROS 2 Jazzy <https://docs.ros.org/en/jazzy/index.html>`_ running on Ubuntu 24.04 LTS and the RZ/V2H RDK.

When developing ROS 2 applications for the Renesas RZ/V2H RDK platform, it is essential to set up the development environment correctly.

This guide helps you get started with ROS 2 development, including setting up the necessary tools, **cross-building applications**, and deploying them to the RZ/V2H RDK board.

Sample applications are also available in the `Renesas AI Applications for RZ/V2H <https://renesas-rz.github.io/rzv_ai_sdk/latest/applications.html#page-top>`_.

This page also provides guidance on porting *Renesas AI Applications for RZ/V2H* to run on the RZ/V2H RDK board with Ubuntu 24.04, and these applications can be used as references or starting points for your own development.

The following topics are covered in this section:

- **Development Guide:** Setup, cross-build, debug, and deployment of applications on the RZ/V2H RDK platform.
- **Model Zoo:** AI model packages optimized with DRP-AI acceleration for the RZ/V2H architecture.
- **Repositories and Packages:** Quick reference for the ROS 2 packages available for development on the RZ/V2H RDK.
- **Sample Applications:** Examples of ROS 2 applications demonstrating various functionalities and use cases.
- **Renesas AI Applications:** Instructions for porting and building Renesas AI Applications using the cross-compilation environment.
- **Other Concepts:** Additional concepts relevant to RZ/V2H RDK ROS 2 development.

.. toctree::
   :maxdepth: 2
   :caption: Application Development

   development_guide/development_guide
   model_zoo/model_zoo
   sample_apps/sample_apps
   renesas_ai_apps/renesas_ai_apps
   other_concepts/other_concepts