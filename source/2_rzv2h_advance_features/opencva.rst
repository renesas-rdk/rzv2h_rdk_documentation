OpenCV Accelerator
============================

Overview
-------------------

The RZ/V2H OpenCV Accelerator (OpenCVA) leverages the OpenCV library to provide optimized computer vision functionalities on the RZ/V2H platform by using the :ref:`DRP IP <drp_concepts>`.

Based on the `OpenCV version 4.10.0 <https://github.com/opencv/opencv/releases/tag/4.10.0>`_, this feature enables efficient image processing and computer vision tasks by offloading some computations to the DRP, thereby enhancing performance and reducing CPU load.

.. seealso::

   For more detail information about OpenCVA, refer to the `RZ/V2H OpenCV Accelerator <https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#22-how-to-use>`_.

How to use OpenCVA
-------------------

OpenCVA leverages the DRP's processing capability to enhance specific functions of the OpenCV library .You can use OpenCVA same as OpenCV as usual and you do not need to consider of OpenCVA architecture.

OpenCVA is automatically executed by DRP as follows if it matches the conditions under which DRP can be used.

For the DRP using conditions, see `OpenCVA API specification and condition for using DRP <https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#4-opencva-api-specification-and-condition-for-using-drp>`_.

OpenCVA can disable DRP, for each function. See `API functions to control OpenCVA <https://github.com/renesas-rz/rzv2h_opencv_accelerator?tab=readme-ov-file#5-api-functions-to-control-opencva>`_ for details.

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
     - Image convolving.
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

.. note::

   On Ubuntu OS, OpenCVA library is installed in the following path:

   `/usr/lib/aarch64-linux-gnu/renesas/libopencv*`

   `/usr/include/opencv4/renesas/opencv4`