.. _opencva:

OpenCV Accelerator
------------------

The RZ/V2H MPU platform includes the OpenCV Accelerator (OpenCVA), which uses the :ref:`DRP IP <drp_concepts>` to accelerate specific OpenCV functions and improve performance in computer vision applications.

Overview
^^^^^^^^

The OpenCVA library is designed to automatically use the DRP to accelerate supported OpenCV functions when the conditions for DRP usage are met.

.. note::

   The OpenCVA library version for the RZ/V2H RDK is **4.6.0**, which is compatible with OpenCV version 4.6.0 on Ubuntu 24.04 LTS.

.. seealso::

   For more detailed information about OpenCVA, refer to the `RZ/V2H OpenCV Accelerator <https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#22-how-to-use>`_.

How to Install the OpenCVA Library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To install the OpenCVA library on the RZ/V2H RDK, follow these steps:

#. Remove any existing OpenCV library installed.

   If you have **installed OpenCV using the apt package manager**, you need to remove it before installing the OpenCVA library, because the OpenCVA library provides its own version of OpenCV that is optimized for the RZ/V2H RDK on Ubuntu 24.04.

   .. code-block:: bash

      sudo apt remove -y libopencv* opencv* python3-opencv

#. Download and run the installation script:

   .. code-block:: bash

      wget -qO install_opencv_arm64.sh https://raw.githubusercontent.com/renesas-rdk/rzv2h_opencv_accelerated_debs/main/install_opencv_arm64.sh
      sudo bash install_opencv_arm64.sh

   The script will install all OpenCVA Debian packages, resolve any missing dependencies, and verify the installation automatically.

   .. note::

      The installation process may report missing dependencies during the ``dpkg`` step. This is expected and will be resolved automatically by the script.

   After script execution, the OpenCVA library and its dependencies will be installed on your RZ/V2H RDK, and you can start using it in your computer vision applications.

How to Use OpenCVA
^^^^^^^^^^^^^^^^^^

OpenCVA leverages the DRP's processing capability to enhance specific functions of the OpenCV library.

When you call an OpenCV function that is supported by OpenCVA, the library automatically determines whether the DRP can be used to accelerate that function based on the input data and the current system conditions.

If the conditions under which DRP can be used are met, OpenCVA automatically executes the function using the DRP.

For the DRP usage conditions, see `OpenCVA API Specification and Condition for Using DRP <https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#4-opencva-api-specification-and-condition-for-using-drp>`_.

OpenCVA can also disable DRP on a per-function basis. See `API Functions to Control OpenCVA <https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#5-api-functions-to-control-opencva>`_ for details.

The following table lists the OpenCV functions that can be executed using DRP in the OpenCVA:

.. list-table:: OpenCV Functions Supported by DRP
   :header-rows: 1
   :widths: 25 75

   * - OpenCV Function Name
     - Function
   * - resize
     - Image resize.
   * - cvtColor
     - Change color space.
   * - cvtColorTwoPlane
     - Change color space.
   * - GaussianBlur
     - Gaussian filter process.
   * - dilate
     - Areas of bright regions grow.
   * - erode
     - Areas of dark regions grow.
   * - morphologyEX
     - Combination of dilate and erode.
   * - filter2D
     - Image convolution.
   * - Sobel
     - Extracting image edges.
   * - adaptiveThreshold
     - Transforms a grayscale image to a binary image according to the formula.
   * - matchTemplate
     - Compares a template against overlapped image regions.
   * - wrapAffine
     - Transforms the source image using the 2x3 matrix.
   * - wrapPerspective
     - Transforms the source image using the 3x3 matrix.
   * - pyrDown
     - Downsampling step of the Gaussian pyramid construction.
   * - pyrUp
     - Upsampling step of the Gaussian pyramid construction.
   * - FAST
     - Detects corners using the FAST algorithm.
   * - remap
     - Applies a generic geometrical transformation to an image.
   * - StereoSGBM
     - Creates StereoSGBM object for the modified H. Hirschmuller algorithm.